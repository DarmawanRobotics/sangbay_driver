"""Modbus RTU framing: read holding registers (0x03), write multiple registers (0x10).
No ROS, no serial."""

import struct


class ModbusError(Exception):
    """Bad, missing or refused answer."""


def crc16(data):
    """Modbus CRC-16 (poly 0xA001, init 0xFFFF)."""
    crc = 0xFFFF
    for byte in bytearray(data):
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def with_crc(body):
    body = bytes(bytearray(body))
    c = crc16(body)
    return body + bytes(bytearray([c & 0xFF, c >> 8]))


def _check(address, register, count):
    if not 0 <= address <= 0xFF:
        raise ValueError('address must be 0..255')
    if not 0 <= register <= 0xFFFF:
        raise ValueError('register must be 0..65535')
    if not 1 <= count <= 123:
        raise ValueError('count must be 1..123')


def read_request(address, register, count=1):
    _check(address, register, count)
    return with_crc([address, 0x03, register >> 8, register & 0xFF, count >> 8, count & 0xFF])


def write_request(address, register, values):
    values = list(values)
    _check(address, register, len(values))
    body = bytearray([address, 0x10, register >> 8, register & 0xFF, 0, len(values), 2 * len(values)])
    body += struct.pack('>%dH' % len(values), *values)
    return with_crc(body)


def read_response_length(count):
    return 5 + 2 * count


WRITE_RESPONSE_LENGTH = 8

EXCEPTIONS = {
    1: 'illegal function',
    2: 'illegal data address',
    3: 'illegal data value',
    4: 'device failure',
    6: 'device busy',
}


def _common(frame, address, function, expected):
    frame = bytes(bytearray(frame))
    if len(frame) == 0:
        raise ModbusError('no answer')
    if len(frame) >= 5 and frame[1] == function | 0x80:
        if crc16(frame[:3]) == struct.unpack_from('<H', frame, 3)[0]:
            code = frame[2]
            raise ModbusError('exception %d (%s)' % (code, EXCEPTIONS.get(code, 'unknown')))
    if len(frame) < expected:
        raise ModbusError('short answer (%d of %d bytes): %s' % (len(frame), expected, hexdump(frame)))
    frame = frame[:expected]
    if crc16(frame[:-2]) != struct.unpack_from('<H', frame, expected - 2)[0]:
        raise ModbusError('CRC error: %s' % hexdump(frame))
    # 0xFF is the factory default address; a module may answer with its own.
    if address != 0xFF and frame[0] != address:
        raise ModbusError('answer from address %d, expected %d' % (frame[0], address))
    if frame[1] != function:
        raise ModbusError('unexpected function 0x%02X' % frame[1])
    return frame


def parse_read_response(frame, address, count):
    """Registers (list of uint16) from an answer, or ModbusError."""
    frame = _common(frame, address, 0x03, read_response_length(count))
    if frame[2] != 2 * count:
        raise ModbusError('byte count %d, expected %d' % (frame[2], 2 * count))
    return list(struct.unpack('>%dH' % count, frame[3:3 + 2 * count]))


def parse_write_response(frame, address, register, count):
    frame = _common(frame, address, 0x10, WRITE_RESPONSE_LENGTH)
    reg, n = struct.unpack('>HH', frame[2:6])
    if (reg, n) != (register, count):
        raise ModbusError('write echo %04X/%d, expected %04X/%d' % (reg, n, register, count))


def hexdump(data):
    return ' '.join('%02X' % b for b in bytearray(data))
