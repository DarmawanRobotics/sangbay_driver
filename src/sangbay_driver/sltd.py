"""SLTD-CH4-100 laser methane remote sensor (TDLAS, up to 100 m, 0-100000 ppm*m). No ROS.

TTL UART 115200 8N1. After a "measurement on" command it sends a data frame every 50 ms:

    CC 09 01 c0 c1 c2 c3 l0 l1 t0 t1 crcL crcH 77
    concentration uint32 LE (ppm*m), light intensity uint16 LE, laser temperature int16 LE

Control: AA len cmd valH valL [data] crcL crcH 55, answered by BB len cmd valH valL result crcL crcH 66.
The checksum is CRC16 over length + command + values + data (the specification does not say
which CRC16; Modbus CRC16 is used and, for data frames, also accepted over the header or the
data alone, see `check`).
"""

import struct
import time

from sangbay_driver.modbus import crc16, hexdump
from sangbay_driver.reading import Reading

DEFAULT_BAUD = 115200
OUTPUT_PERIOD = 0.05
MAX_CONCENTRATION = 100000

CMD_ZERO = 0x09
CMD_TEMPERATURE_COMPENSATION = 0x0A
CMD_START = 0x10
CMD_STOP = 0x11
CMD_TEMPERATURE_CALIBRATION = 0x12

DATA_HEADER, DATA_TAIL = 0xCC, 0x77
ACK_HEADER, ACK_TAIL = 0xBB, 0x66
CONTROL_HEADER, CONTROL_TAIL = 0xAA, 0x55


def control_frame(command, value_high=0, value_low=0, data=b''):
    payload = bytearray([3 + len(data), command, value_high, value_low]) + bytearray(data)
    c = crc16(payload)
    return bytes(bytearray([CONTROL_HEADER]) + payload + bytearray([c & 0xFF, c >> 8, CONTROL_TAIL]))


def start_frame(mode='dynamic', pointer=True):
    """Measurement on. `pointer`: green indicator laser on."""
    if mode not in ('dynamic', 'static'):
        raise ValueError("mode must be 'dynamic' or 'static'")
    return control_frame(CMD_START, 1 if pointer else 0, 1 if mode == 'static' else 0)


def stop_frame():
    return control_frame(CMD_STOP)


def crc_ok(frame, length):
    """CRC over length..data (as specified), or header..data, or data only."""
    end = 2 + length
    received = frame[end] | frame[end + 1] << 8
    return received in (crc16(frame[1:end]), crc16(frame[0:end]), crc16(frame[2:end]))


class Parser(object):
    """Splits a byte stream into data frames and acknowledgements."""

    def __init__(self, check=True):
        self.check = check
        self.buffer = bytearray()
        self.errors = 0
        self.last_error = ''

    def feed(self, data):
        """[('data', (concentration, light, temperature)) | ('ack', (command, high, low, result))]"""
        self.buffer += bytearray(data)
        out = []
        while True:
            # drop bytes until a header
            while self.buffer and self.buffer[0] not in (DATA_HEADER, ACK_HEADER):
                self.buffer.pop(0)
            if len(self.buffer) < 2:
                return out
            header, length = self.buffer[0], self.buffer[1]
            size = length + 5
            if len(self.buffer) < size:
                return out
            frame = bytes(self.buffer[:size])
            tail = DATA_TAIL if header == DATA_HEADER else ACK_TAIL
            good_shape = frame[-1] == tail and (
                (header == DATA_HEADER and length == 9 and frame[2] == 0x01) or
                (header == ACK_HEADER and length == 4))
            if not good_shape or (self.check and not crc_ok(bytearray(frame), length)):
                self.errors += 1
                self.last_error = ('bad frame: %s' % hexdump(frame)) if not good_shape else \
                    ('CRC error: %s' % hexdump(frame))
                self.buffer.pop(0)  # resync on the next header
                continue
            del self.buffer[:size]
            if header == DATA_HEADER:
                out.append(('data', struct.unpack('<IHh', frame[3:11])))
            else:
                out.append(('ack', tuple(bytearray(frame[2:6]))))


def data_frame(concentration, light, temperature):
    """A frame as the sensor sends it (for tests and the simulator)."""
    payload = bytearray([9, 0x01]) + struct.pack('<IHh', concentration, light, temperature)
    c = crc16(payload)
    return bytes(bytearray([DATA_HEADER]) + payload + bytearray([c & 0xFF, c >> 8, DATA_TAIL]))


def ack_frame(command, high, low, result=0):
    payload = bytearray([4, command, high, low, result])
    c = crc16(payload)
    return bytes(bytearray([ACK_HEADER]) + payload + bytearray([c & 0xFF, c >> 8, ACK_TAIL]))


def to_reading(concentration, light, temperature):
    over = concentration >= MAX_CONCENTRATION
    values = [('gas', 'CH4 (methane, laser remote)'),
              ('concentration', '%d ppm*m' % concentration),
              ('light_intensity', str(light)), ('laser_temperature_raw', str(temperature))]
    return Reading(float(concentration), 'ppm*m', 'CH4', status='OVER-RANGE' if over else 'NORMAL',
                   alarm=over, values=values,
                   extra={'light_intensity': float(light), 'laser_temperature': float(temperature)})


class Sltd(object):
    """`port` needs write / read / in_waiting (a pyserial Serial)."""

    type = 'sltd_ch4'

    def __init__(self, port, mode='dynamic', pointer=True, check_crc=True):
        self.port = port
        self.mode = mode
        self.pointer = pointer
        self.parser = Parser(check_crc)
        self.last_ack = None
        self._pending = []
        start_frame(mode, pointer)  # validates mode

    @property
    def errors(self):
        return self.parser.errors

    @property
    def last_error(self):
        return self.parser.last_error

    def info(self):
        return {'gas': 'CH4', 'unit': 'ppm*m', 'gas_description': 'methane (SLTD-CH4-100 laser remote)'}

    def start(self):
        self.port.write(start_frame(self.mode, self.pointer))

    def stop(self):
        self.port.write(stop_frame())

    def read(self, timeout=1.0):
        """Next reading, or None after `timeout` s."""
        if self._pending:
            return to_reading(*self._pending.pop(0))
        deadline = time.time() + timeout
        while time.time() < deadline:
            data = self.port.read(max(1, getattr(self.port, 'in_waiting', 0) or 0))
            if not data:
                continue
            frames = self.parser.feed(data)
            for kind, value in frames:
                if kind == 'ack':
                    self.last_ack = value
            readings = [value for kind, value in frames if kind == 'data']
            if readings:
                self._pending.extend(readings[1:])  # same chunk: returned by the next calls
                return to_reading(*readings[0])
        return None
