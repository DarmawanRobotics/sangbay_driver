"""Sangbay S-M4T gas sensor over RS485 (Modbus RTU). No ROS."""

import threading
import time

from sangbay_driver.modbus import ModbusError, parse_read_response, read_request, response_length

REG_GAS_CODE = 0x2027
REG_DECIMALS = 0x2031
REG_STATUS = 0x6000
REG_CONCENTRATION = 0x6001

GAS_NAMES = {0x02: 'H2S'}

# Status word, bit 15 set = preheating finished.
ALARM_BITS = [
    (6, 'high_protection', 'HIGH-PROTECTION'),
    (5, 'sensor_error', 'SENSOR-ERROR'),
    (4, 'over_range', 'OVER-RANGE'),
    (3, 'twa', 'TWA'),
    (2, 'stel', 'STEL'),
    (1, 'high', 'HIGH'),
    (0, 'low', 'LOW'),
]


def gas_name(code):
    return GAS_NAMES.get(code, 'gas 0x%02X' % code)


class Status(object):
    def __init__(self, word):
        self.word = word
        self.preheating = not word & 0x8000
        self.preheat_remaining = (word & 0x7FFF) * 0.1 if self.preheating else 0.0
        self.flags = {}
        for bit, key, _ in ALARM_BITS:
            self.flags[key] = bool(not self.preheating and word & (1 << bit))

    @property
    def alarm(self):
        return any(self.flags.values())

    @property
    def text(self):
        if self.preheating:
            return 'PREHEATING (%.1f s)' % self.preheat_remaining
        names = [label for bit, key, label in ALARM_BITS if self.flags[key]]
        return ', '.join(names) if names else 'NORMAL'


class Reading(object):
    def __init__(self, raw, decimals, status_word, gas_code):
        self.raw = raw
        self.decimals = decimals
        self.concentration = raw / float(10 ** decimals)
        self.status = Status(status_word)
        self.gas_code = gas_code
        self.gas = gas_name(gas_code)


class Sensor(object):
    """One sensor on a serial port. `port` is anything with write/read/reset_input_buffer
    (a pyserial Serial), so tests can pass a fake."""

    def __init__(self, port, address=0xFF, retries=2, inter_frame=0.005):
        self.port = port
        self.address = address
        self.retries = retries
        self.inter_frame = inter_frame
        self.gas_code = None
        self.decimals = None
        self._lock = threading.Lock()

    def read_registers(self, register, count=1):
        last = None
        for _ in range(self.retries + 1):
            with self._lock:
                self.port.reset_input_buffer()
                self.port.write(read_request(self.address, register, count))
                answer = self.port.read(response_length(count))
                time.sleep(self.inter_frame)
            try:
                return parse_read_response(answer, self.address, count)
            except ModbusError as e:
                last = e
        raise last

    def read_info(self):
        """Gas code and decimal places, read once after connecting."""
        self.gas_code = self.read_registers(REG_GAS_CODE)[0]
        self.decimals = self.read_registers(REG_DECIMALS)[0]
        if self.decimals > 6:
            raise ModbusError('implausible decimal places %d' % self.decimals)
        return self.gas_code, self.decimals

    def read(self):
        if self.decimals is None:
            self.read_info()
        # Two single reads, as the vendor examples do.
        raw = self.read_registers(REG_CONCENTRATION)[0]
        status = self.read_registers(REG_STATUS)[0]
        return Reading(raw, self.decimals, status, self.gas_code)


def open_serial(device, baud=9600, timeout=0.5):
    import serial
    return serial.Serial(port=device, baudrate=baud, bytesize=serial.EIGHTBITS,
                         parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=timeout)
