#!/usr/bin/env python3
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # nose -P
import threading
import time
import unittest

from helpers import MemoryPort
from fake_devices import FakeModule, FakeModuleBus
from sangbay_driver.modbus import ModbusError
from sangbay_driver.module import Module, Status, gas_name, unit_name


class TestTables(unittest.TestCase):
    def test_names(self):
        self.assertEqual(gas_name(0x02), 'H2S')
        self.assertEqual(gas_name(0x46), 'H2O2')
        self.assertEqual(gas_name(0x99), 'gas 0x99')
        self.assertEqual(unit_name(1), '%LEL')
        self.assertEqual(unit_name(4), '%VOL')


class TestStatus(unittest.TestCase):
    def test_preheating(self):
        s = Status(125)
        self.assertTrue(s.preheating)
        self.assertAlmostEqual(s.preheat_remaining, 12.5)
        self.assertFalse(s.alarm or s.fault)
        self.assertEqual(s.text, 'PREHEATING (12.5 s)')
        self.assertEqual(Status(0).text, 'PREHEATING (0.0 s)')

    def test_flags(self):
        self.assertEqual(Status(0x8000).text, 'NORMAL')
        s = Status(0x8000 | 0b110)
        self.assertEqual(s.text, 'STEL, HIGH')
        self.assertTrue(s.alarm)
        self.assertFalse(s.fault)
        s = Status(0x8000 | 1 << 5)
        self.assertTrue(s.fault)
        self.assertFalse(s.alarm)
        self.assertTrue(Status(0x8000 | 1 << 4).alarm)  # over range


class TestModule(unittest.TestCase):
    def setUp(self):
        self.h2s = FakeModule(1, gas_code=0x02, decimals=2, unit=2, value=7.25)
        self.o2 = FakeModule(2, gas_code=0x04, decimals=1, unit=4, value=20.9)
        self.port = MemoryPort(FakeModuleBus([self.h2s, self.o2]))

    def test_two_on_one_bus(self):
        lock = threading.RLock()
        a = Module(self.port, 1, inter_frame=0, lock=lock)
        b = Module(self.port, 2, inter_frame=0, lock=lock)
        r = a.read()
        self.assertEqual((r.gas, r.unit, r.concentration), ('H2S', 'ppm', 7.25))
        self.assertEqual(r.status, 'NORMAL')
        self.assertTrue(r.valid)
        r = b.read()
        self.assertEqual((r.gas, r.unit), ('O2', '%VOL'))
        self.assertAlmostEqual(r.concentration, 20.9)
        self.assertEqual(dict(r.values)['address'], '2')

    def test_alarm_threshold_preheat(self):
        m = Module(self.port, 1, inter_frame=0)
        self.h2s.set_alarm(0b10)
        r = m.read()
        self.assertTrue(r.alarm)
        self.assertEqual(r.status, 'HIGH')
        self.h2s.set_alarm(0)
        r = m.read().apply_threshold(5.0)
        self.assertTrue(r.alarm)
        self.assertEqual(r.status, 'THRESHOLD')
        self.assertFalse(m.read().apply_threshold(None).alarm)
        self.h2s.preheat_until = time.time() + 5
        r = m.read().apply_threshold(5.0)
        self.assertFalse(r.valid)
        self.assertFalse(r.alarm)
        self.assertTrue(r.status.startswith('PREHEATING'))

    def test_missing_and_retries(self):
        m = Module(self.port, 9, retries=2, inter_frame=0)
        self.assertRaisesRegex(ModbusError, 'no answer', m.read)
        self.h2s.silent = True
        m = Module(self.port, 1, retries=2, inter_frame=0)
        before = self.h2s.requests
        self.assertRaises(ModbusError, m.read_registers, 0x6001)
        self.assertEqual(self.h2s.requests - before, 3)

    def test_unit_optional(self):
        del self.h2s.registers[0x2030]
        del self.h2s.registers[0x202B]
        m = Module(self.port, 1, inter_frame=0)
        info = m.read_info()
        self.assertEqual(info['unit'], 'ppm')
        self.assertIsNone(info['range_raw'])

    def test_set_address(self):
        m = Module(self.port, 1, inter_frame=0)
        m.set_address(5)
        self.assertEqual(self.h2s.address, 5)
        self.assertEqual(Module(self.port, 5, inter_frame=0).read().gas, 'H2S')
        self.assertRaises(ValueError, m.set_address, 255)

    def test_set_address_needs_unlock(self):
        m = Module(self.port, 1, retries=0, inter_frame=0)
        self.assertRaisesRegex(ModbusError, 'exception', m.write_registers, 0x2011, [4])


if __name__ == '__main__':
    unittest.main()
