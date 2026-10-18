#!/usr/bin/env python3
import unittest

import helpers  # noqa: F401
from sangbay_driver.modbus import (ModbusError, crc16, hexdump, parse_read_response,
                                   parse_write_response, read_request, with_crc, write_request)


class TestFrames(unittest.TestCase):
    def test_crc(self):
        self.assertEqual(crc16(b'123456789'), 0x4B37)

    def test_datasheet_examples(self):
        # S-M4T / S-M4 / K-5S datasheets, section 7
        self.assertEqual(hexdump(read_request(0xFF, 0x6001)), 'FF 03 60 01 00 01 DE 14')
        self.assertEqual(hexdump(read_request(0xFF, 0x2031)), 'FF 03 20 31 00 01 CB DB')
        self.assertEqual(hexdump(read_request(0xFF, 0x6006)), 'FF 03 60 06 00 01 6F D5')
        self.assertEqual(hexdump(write_request(0xFF, 0x4FFF, [0x55AA])), 'FF 10 4F FF 00 01 02 55 AA FB D0')
        self.assertEqual(hexdump(write_request(0xFF, 0x2011, [1])), 'FF 10 20 11 00 01 02 00 01 0D 77')
        self.assertEqual(hexdump(write_request(0xFF, 0x6006, [0x1000, 0])),
                         'FF 10 60 06 00 02 04 10 00 00 00 E8 AC')
        self.assertEqual(parse_read_response(bytes.fromhex('FF0302000551 93'.replace(' ', '')), 0xFF, 1), [5])
        self.assertEqual(parse_read_response(bytes.fromhex('FF03020001 5050'.replace(' ', '')), 0xFF, 1), [1])
        parse_write_response(bytes.fromhex('FF104FFF000132F3'), 0xFF, 0x4FFF, 1)
        parse_write_response(bytes.fromhex('FF10201100014FD2'), 0xFF, 0x2011, 1)

    def test_limits(self):
        self.assertRaises(ValueError, read_request, 256, 0)
        self.assertRaises(ValueError, read_request, 1, 0, 0)
        self.assertRaises(ValueError, write_request, 1, 0, [])

    def test_errors(self):
        good = with_crc([0x01, 0x03, 0x02, 0x00, 0x01])
        self.assertRaisesRegex(ModbusError, 'no answer', parse_read_response, b'', 1, 1)
        self.assertRaisesRegex(ModbusError, 'short', parse_read_response, good[:5], 1, 1)
        bad = bytearray(good)
        bad[4] ^= 1
        self.assertRaisesRegex(ModbusError, 'CRC', parse_read_response, bytes(bad), 1, 1)
        self.assertRaisesRegex(ModbusError, 'address', parse_read_response, good, 2, 1)
        self.assertEqual(parse_read_response(good, 0xFF, 1), [1])  # factory address: any answer
        self.assertRaisesRegex(ModbusError, 'illegal data address', parse_read_response,
                               with_crc([0x01, 0x83, 0x02]), 1, 1)
        self.assertRaisesRegex(ModbusError, 'echo', parse_write_response,
                               with_crc([0x01, 0x10, 0x20, 0x11, 0x00, 0x02]), 1, 0x2011, 1)


if __name__ == '__main__':
    unittest.main()
