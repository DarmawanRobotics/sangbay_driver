#!/usr/bin/env python3
import os
import sys
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'src'))
sys.path.insert(0, HERE)

from sangbay_driver.modbus import (ModbusError, crc16, parse_read_response,  # noqa: E402
                                   read_request, with_crc)
from sangbay_driver.sensor import Sensor, Status, open_serial  # noqa: E402
from fake_sensor import FakeSM4T, PtyServer  # noqa: E402


class FakePort(object):
    """In-memory port in front of a FakeSM4T."""

    def __init__(self, sensor):
        self.sensor = sensor
        self.pending = b''

    def reset_input_buffer(self):
        self.pending = b''

    def write(self, data):
        self.pending = self.sensor.answer(bytes(data))

    def read(self, n):
        out, self.pending = self.pending[:n], self.pending[n:]
        return out


class TestModbus(unittest.TestCase):
    def test_crc(self):
        self.assertEqual(read_request(0x01, 0x0000, 1), bytes(bytearray.fromhex('010300000001840A')))
        self.assertEqual(crc16(b'123456789'), 0x4B37)

    def test_request_limits(self):
        self.assertRaises(ValueError, read_request, 256, 0)
        self.assertRaises(ValueError, read_request, 1, 0, 0)

    def test_parse(self):
        frame = with_crc([0x01, 0x03, 0x04, 0x00, 0x7D, 0x80, 0x00])
        self.assertEqual(parse_read_response(frame, 0x01, 2), [0x7D, 0x8000])
        self.assertEqual(parse_read_response(frame, 0xFF, 2), [0x7D, 0x8000])

    def test_errors(self):
        good = with_crc([0x01, 0x03, 0x02, 0x00, 0x01])
        self.assertRaisesRegex(ModbusError, 'no answer', parse_read_response, b'', 1, 1)
        self.assertRaisesRegex(ModbusError, 'short', parse_read_response, good[:5], 1, 1)
        bad = bytearray(good)
        bad[4] ^= 1
        self.assertRaisesRegex(ModbusError, 'CRC', parse_read_response, bytes(bad), 1, 1)
        self.assertRaisesRegex(ModbusError, 'address', parse_read_response, good, 2, 1)
        self.assertRaisesRegex(ModbusError, 'illegal data address', parse_read_response,
                               with_crc([0x01, 0x83, 0x02]), 1, 1)


class TestStatus(unittest.TestCase):
    def test_preheating(self):
        s = Status(125)
        self.assertTrue(s.preheating)
        self.assertAlmostEqual(s.preheat_remaining, 12.5)
        self.assertFalse(s.alarm)
        self.assertEqual(s.text, 'PREHEATING (12.5 s)')
        self.assertEqual(Status(0).text, 'PREHEATING (0.0 s)')  # the old script showed '?'

    def test_alarms(self):
        self.assertEqual(Status(0x8000).text, 'NORMAL')
        s = Status(0x8000 | 0b110)
        self.assertTrue(s.flags['high'] and s.flags['stel'] and not s.flags['low'])
        self.assertEqual(s.text, 'STEL, HIGH')
        self.assertTrue(s.alarm)


class TestSensor(unittest.TestCase):
    def test_read(self):
        fake = FakeSM4T(decimals=2, ppm=7.25)
        sensor = Sensor(FakePort(fake))
        r = sensor.read()
        self.assertEqual(r.gas, 'H2S')
        self.assertEqual(r.decimals, 2)
        self.assertEqual(r.raw, 725)
        self.assertAlmostEqual(r.concentration, 7.25)
        self.assertEqual(r.status.text, 'NORMAL')
        fake.set_alarm(0b1)
        self.assertTrue(sensor.read().status.flags['low'])

    def test_retries_then_fails(self):
        fake = FakeSM4T()
        fake.silent = True
        sensor = Sensor(FakePort(fake), retries=2, inter_frame=0)
        self.assertRaises(ModbusError, sensor.read_registers, 0x6001)
        self.assertEqual(fake.requests, 3)

    def test_wrong_address_ignored(self):
        sensor = Sensor(FakePort(FakeSM4T(address=5)), address=6, retries=0, inter_frame=0)
        self.assertRaisesRegex(ModbusError, 'no answer', sensor.read)

    def test_unknown_gas(self):
        r = Sensor(FakePort(FakeSM4T(gas_code=0x09))).read()
        self.assertEqual(r.gas, 'gas 0x09')


class TestSerial(unittest.TestCase):
    def test_over_pty(self):
        try:
            import serial  # noqa: F401
        except ImportError:
            self.skipTest('pyserial missing')
        server = PtyServer(FakeSM4T(address=3, ppm=12.5, preheat=0.5))
        try:
            port = open_serial(server.path)
            sensor = Sensor(port, address=3)
            r = sensor.read()
            self.assertTrue(r.status.preheating)
            self.assertAlmostEqual(r.concentration, 12.5)
            time.sleep(0.6)
            self.assertFalse(sensor.read().status.preheating)
            port.close()
        finally:
            server.close()


if __name__ == '__main__':
    unittest.main()
