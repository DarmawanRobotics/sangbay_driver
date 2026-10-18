"""Modbus RTU framing (function 0x03, read holding registers). No ROS, no serial."""

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


def read_request(address, register, count=1):
    if not 0 <= address <= 0xFF:
        raise ValueError('address must be 0..255')
    if not 0 <= register <= 0xFFFF:
        raise ValueError('register must be 0..65535')
    if not 1 <= count <= 125:
        raise ValueError('count must be 1..125')
    return with_crc([address, 0x03, register >> 8, register & 0xFF, count >> 8, count & 0xFF])


def response_length(count):
    """Length of a normal answer to a read of `count` registers."""
    return 5 + 2 * count


EXCEPTIONS = {
    1: 'illegal function',
    2: 'illegal data address',
    3: 'illegal data value',
    4: 'device failure',
    6: 'device busy',
}


def parse_read_response(frame, address, count):
    """Registers (list of uint16) from an answer, or ModbusError."""
    frame = bytes(bytearray(frame))
    if len(frame) == 0:
        raise ModbusError('no answer')
    if len(frame) >= 5 and frame[1] == 0x83:
        if crc16(frame[:3]) == struct.unpack_from('<H', frame, 3)[0]:
            code = frame[2]
            raise ModbusError('exception %d (%s)' % (code, EXCEPTIONS.get(code, 'unknown')))
    expected = response_length(count)
    if len(frame) < expected:
        raise ModbusError('short answer (%d of %d bytes): %s' % (len(frame), expected, hexdump(frame)))
    frame = frame[:expected]
    if crc16(frame[:-2]) != struct.unpack_from('<H', frame, expected - 2)[0]:
        raise ModbusError('CRC error: %s' % hexdump(frame))
    # 0xFF is the broadcast-like default address: the sensor may answer with its real one.
    if address != 0xFF and frame[0] != address:
        raise ModbusError('answer from address %d, expected %d' % (frame[0], address))
    if frame[1] != 0x03:
        raise ModbusError('unexpected function 0x%02X' % frame[1])
    if frame[2] != 2 * count:
        raise ModbusError('byte count %d, expected %d' % (frame[2], 2 * count))
    return list(struct.unpack('>%dH' % count, frame[3:3 + 2 * count]))


def hexdump(data):
    return ' '.join('%02X' % b for b in bytearray(data))
