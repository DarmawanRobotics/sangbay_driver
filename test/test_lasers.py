#!/usr/bin/env python3
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # nose -P
import unittest

from helpers import MemoryPort
from fake_devices import FakeGt, FakeSltd
from sangbay_driver import gt_ts01a, sltd
from sangbay_driver.modbus import crc16, hexdump


class TestGt(unittest.TestCase):
    def test_datasheet_example(self):
        self.assertEqual(gt_ts01a.parse_line(b'+000.00 +22.3 +101.67 00 37\r\n'), (0.0, 22.3, 101.67, 0))
        self.assertEqual(gt_ts01a.parse_line('+000.00+22.3+101.670037'), (0.0, 22.3, 101.67, 0))
        self.assertEqual(gt_ts01a.make_line(0.0, 22.3, 101.67), b'+000.00 +22.3 +101.67 00 37\r\n')
        self.assertEqual(len(gt_ts01a.make_line(0.0, 22.3, 101.67)), 29)

    def test_bad(self):
        self.assertRaises(gt_ts01a.FrameError, gt_ts01a.parse_line, b'+000.00 +22.3 +101.67 00 38\r\n')
        self.assertRaises(gt_ts01a.FrameError, gt_ts01a.parse_line, b'garbage\r\n')

    def test_status(self):
        r = gt_ts01a.to_reading(*gt_ts01a.parse_line(gt_ts01a.make_line(1.5, -5.0, 99.0, 0x11)))
        self.assertEqual(r.status, 'TEMP-CONTROL, PEAK-POSITION')  # datasheet: "11"
        self.assertTrue(r.fault)
        self.assertEqual((r.concentration, r.unit, r.extra['temperature']), (1.5, '%VOL', -5.0))

    def test_reader(self):
        fake = FakeGt(ch4=0.5, period=0)
        dev = gt_ts01a.GtTs01a(MemoryPort(fake))
        r = dev.read(1.0)
        self.assertEqual((r.gas, r.concentration, r.status), ('CH4', 0.5, 'NORMAL'))
        fake.silent = True
        self.assertIsNone(dev.read(0.2))


class TestSltd(unittest.TestCase):
    def test_frames(self):
        self.assertEqual(hexdump(sltd.start_frame('dynamic', True)), 'AA 03 10 01 00 00 35 55')
        self.assertEqual(hexdump(sltd.start_frame('static', False))[:14], 'AA 03 10 00 01')
        self.assertEqual(hexdump(sltd.stop_frame())[:14], 'AA 03 11 00 00')
        self.assertRaises(ValueError, sltd.start_frame, 'fast')
        f = sltd.data_frame(1234, 500, -3)
        self.assertEqual(len(f), 14)
        self.assertEqual((f[0], f[1], f[2], f[-1]), (0xCC, 9, 1, 0x77))

    def test_parser_resync_and_split(self):
        p = sltd.Parser()
        f = sltd.data_frame(100000, 10, 20)
        self.assertEqual(p.feed(b'\x00\x13' + f[:6]), [])
        out = p.feed(f[6:] + sltd.ack_frame(0x10, 1, 0) + f)
        self.assertEqual(out, [('data', (100000, 10, 20)), ('ack', (0x10, 1, 0, 0)), ('data', (100000, 10, 20))])
        r = sltd.to_reading(100000, 10, 20)
        self.assertTrue(r.alarm)
        self.assertEqual(r.status, 'OVER-RANGE')

    def test_crc_variants(self):
        frame = bytearray(sltd.data_frame(5, 6, 7))
        p = sltd.Parser()
        for c in (crc16(frame[0:11]), crc16(frame[2:11])):  # header..data, data only
            frame[11], frame[12] = c & 0xFF, c >> 8
            self.assertEqual(len(p.feed(bytes(frame))), 1)
        frame[11] ^= 0xFF
        self.assertEqual(p.feed(bytes(frame)), [])
        self.assertIn('CRC', p.last_error)
        self.assertEqual(len(sltd.Parser(check=False).feed(bytes(frame))), 1)

    def test_device(self):
        fake = FakeSltd(concentration=321, period=0)
        dev = sltd.Sltd(MemoryPort(fake), mode='static', pointer=False)
        self.assertIsNone(dev.read(0.1))  # not measuring yet
        dev.start()
        self.assertEqual((fake.measuring, fake.mode, fake.pointer), (True, 'static', False))
        r = dev.read(0.5)
        self.assertEqual((r.concentration, r.unit), (321.0, 'ppm*m'))
        self.assertEqual(dev.last_ack, (0x10, 0, 1, 0))
        dev.stop()
        self.assertFalse(fake.measuring)


if __name__ == '__main__':
    unittest.main()
