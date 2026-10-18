"""Sangbay gas sensor modules with Modbus RTU: S-M4, S-M4T (TTL), S-M4T 485, K-5S.

All four share one register map (datasheets V1.3.x). The sensor heads (S4-H2S-100,
MJC4-2.8J, N-CH4, ...) plug into these modules. No ROS.
"""

import threading
import time

from sangbay_driver.modbus import (ModbusError, WRITE_RESPONSE_LENGTH, parse_read_response,
                                   parse_write_response, read_request, read_response_length,
                                   write_request)
from sangbay_driver.reading import Reading

REG_ADDRESS = 0x2011
REG_GAS_CODE = 0x2027
REG_RANGE = 0x202B
REG_UNIT = 0x2030
REG_DECIMALS = 0x2031
REG_UNLOCK = 0x4FFF
REG_STATUS = 0x6000
REG_CONCENTRATION = 0x6001
REG_AD = 0x6002
REG_COMMAND = 0x6006

UNLOCK_KEY = 0x55AA
DEFAULT_ADDRESS = 0xFF

# Table 1, gas name code: (short name, description)
GASES = {
    0x01: ('CO', 'carbon monoxide'), 0x02: ('H2S', 'hydrogen sulfide'),
    0x03: ('EX', 'combustible gas'), 0x04: ('O2', 'oxygen'),
    0x05: ('SO2', 'sulfur dioxide'), 0x06: ('CH4', 'methane'),
    0x07: ('NO', 'nitric oxide'), 0x08: ('NO2', 'nitrogen dioxide'),
    0x09: ('CL2', 'chlorine'), 0x0A: ('NH3', 'ammonia'),
    0x0B: ('H2', 'hydrogen'), 0x0C: ('HCN', 'hydrogen cyanide'),
    0x0D: ('HCL', 'hydrogen chloride'), 0x0E: ('PH3', 'phosphine'),
    0x0F: ('O3', 'ozone'), 0x10: ('CLO2', 'chlorine dioxide'),
    0x11: ('C2H4O', 'ethylene oxide'), 0x12: ('CO2', 'carbon dioxide'),
    0x13: ('C6H6', 'benzene'), 0x14: ('CH2O', 'formaldehyde'),
    0x15: ('VOC', 'volatile organic compounds'), 0x16: ('C7H8', 'toluene'),
    0x17: ('C8H10', 'xylene'), 0x18: ('NOx', 'nitrogen oxides'),
    0x19: ('C6H14', 'n-hexane'), 0x1A: ('HF', 'hydrogen fluoride'),
    0x1B: ('xF', 'fluoride'), 0x1C: ('CH3OH', 'methanol'),
    0x1D: ('C3H8', 'propane'), 0x1E: ('C7H8O', 'cresol'),
    0x1F: ('C2H6', 'ethane'), 0x20: ('C4H10O', 'diethyl ether'),
    0x21: ('CH4', 'coal gas'), 0x22: ('CS2', 'carbon disulfide'),
    0x23: ('SiH4', 'silane'), 0x24: ('C2H2', 'acetylene'),
    0x25: ('CH3CL', 'chloromethane'), 0x26: ('N2', 'nitrogen'),
    0x27: ('C8H8', 'styrene'), 0x28: ('AsH3', 'arsine'),
    0x29: ('C2H6O', 'ethanol'), 0x2A: ('C2H3CL', 'vinyl chloride'),
    0x2B: ('C2H4Cl2', 'dichloroethane'), 0x2C: ('C4H8O2', 'ethyl acetate'),
    0x2D: ('CH2Cl2', 'dichloromethane'), 0x2E: ('C3H3N', 'acrylonitrile'),
    0x2F: ('C6H7N', 'aniline'), 0x30: ('EX', 'natural gas'),
    0x31: ('Br2', 'bromine'), 0x32: ('COCL2', 'phosgene'),
    0x33: ('N2H4', 'hydrazine'), 0x34: ('CHOOH', 'formic acid'),
    0x35: ('HBr', 'hydrogen bromide'), 0x36: ('CH4S', 'methanethiol'),
    0x37: ('C4H8', 'butene'), 0x38: ('C2H4', 'ethylene'),
    0x39: ('C3H6', 'propylene'), 0x3A: ('C4H6O2', 'vinyl acetate'),
    0x3B: ('C3H8O', 'isopropanol'), 0x3C: ('C2H4O', 'acetaldehyde'),
    0x3D: ('C4H6', 'butadiene'), 0x3E: ('COS', 'carbonyl sulfide'),
    0x3F: ('C2H6S', 'dimethyl sulfide'), 0x40: ('C4H8S', 'tetrahydrothiophene'),
    0x41: ('C2H5OCL', 'epichlorohydrin'), 0x42: ('CHCL3', 'chloroform'),
    0x43: ('CCL4', 'carbon tetrachloride'), 0x44: ('WF6', 'tungsten hexafluoride'),
    0x45: ('SNCL4', 'tin tetrachloride'), 0x46: ('H2O2', 'hydrogen peroxide'),
}

# Table 2, unit code
UNITS = {
    1: '%LEL', 2: 'ppm', 3: 'ppb', 4: '%VOL', 5: 'umol/mol', 6: 'mg/m3', 7: 'ug/m3',
    8: 'C', 9: '%RH', 10: 'm/s', 11: 'kPa', 12: 'lx',
}

# Table 4, alarm status bits (bit 15 set: preheating done)
ALARM_BITS = [
    (6, 'high_protection', 'HIGH-PROTECTION'),
    (5, 'sensor_error', 'SENSOR-ERROR'),
    (4, 'over_range', 'OVER-RANGE'),
    (3, 'twa', 'TWA'),
    (2, 'stel', 'STEL'),
    (1, 'high', 'HIGH'),
    (0, 'low', 'LOW'),
]
FAULT_FLAGS = ('sensor_error',)

# Table 5, operating status
OPERATING_STATUS = {
    0x0000: 'busy', 0x0001: 'idle / done', 0x0002: 'failed', 0x0003: 'unsupported command',
    0x0004: 'invalid parameter', 0x0005: 'timeout',
}


def gas_name(code):
    return GASES.get(code, ('gas 0x%02X' % code, 'unknown'))[0]


def gas_description(code):
    return GASES.get(code, ('', 'unknown gas code 0x%02X' % code))[1]


def unit_name(code):
    return UNITS.get(code, 'unit %d' % code)


class Status(object):
    """Register 0x6000."""

    def __init__(self, word):
        self.word = word
        self.preheating = not word & 0x8000
        self.preheat_remaining = (word & 0x7FFF) * 0.1 if self.preheating else 0.0
        self.flags = {}
        for bit, key, _ in ALARM_BITS:
            self.flags[key] = bool(not self.preheating and word & (1 << bit))

    @property
    def fault(self):
        return any(self.flags[k] for k in FAULT_FLAGS)

    @property
    def alarm(self):
        return any(v for k, v in self.flags.items() if k not in FAULT_FLAGS)

    @property
    def text(self):
        if self.preheating:
            return 'PREHEATING (%.1f s)' % self.preheat_remaining
        names = [label for bit, key, label in ALARM_BITS if self.flags[key]]
        return ', '.join(names) if names else 'NORMAL'


class Module(object):
    """One module, addressed on a serial port shared with others (RS485) or alone (TTL).

    `port` is anything with write / read / reset_input_buffer (a pyserial Serial), so tests
    can pass a fake. Calls are serialized per instance; modules on one bus share a lock
    through `lock`.
    """

    type = 'module'

    def __init__(self, port, address=DEFAULT_ADDRESS, retries=2, inter_frame=0.01, lock=None):
        if not 0 <= address <= 0xFF:
            raise ValueError('address must be 0..255')
        self.port = port
        self.address = address
        self.retries = retries
        self.inter_frame = inter_frame
        self.lock = lock or threading.RLock()
        self.gas_code = None
        self.unit_code = None
        self.decimals = None
        self.range_raw = None

    # -- transactions --------------------------------------------------------

    def _transact(self, request, length):
        with self.lock:
            self.port.reset_input_buffer()
            self.port.write(request)
            answer = self.port.read(length)
            time.sleep(self.inter_frame)  # bus turnaround before the next frame
        return answer

    def read_registers(self, register, count=1):
        last = None
        for _ in range(self.retries + 1):
            answer = self._transact(read_request(self.address, register, count),
                                    read_response_length(count))
            try:
                return parse_read_response(answer, self.address, count)
            except ModbusError as e:
                last = e
        raise last

    def write_registers(self, register, values):
        values = list(values)
        last = None
        for _ in range(self.retries + 1):
            answer = self._transact(write_request(self.address, register, values),
                                    WRITE_RESPONSE_LENGTH)
            try:
                return parse_write_response(answer, self.address, register, len(values))
            except ModbusError as e:
                last = e
        raise last

    def _optional(self, register):
        try:
            return self.read_registers(register)[0]
        except ModbusError as e:
            if 'exception' in str(e):
                return None
            raise

    # -- reading -------------------------------------------------------------

    def read_info(self):
        """Gas, unit, decimal places and range; read once after connecting."""
        self.gas_code = self.read_registers(REG_GAS_CODE)[0]
        self.decimals = self.read_registers(REG_DECIMALS)[0]
        if self.decimals > 6:
            raise ModbusError('implausible decimal places %d' % self.decimals)
        self.unit_code = self._optional(REG_UNIT)
        self.range_raw = self._optional(REG_RANGE)
        return self.info()

    def info(self):
        return {
            'gas': gas_name(self.gas_code) if self.gas_code is not None else '?',
            'gas_description': gas_description(self.gas_code) if self.gas_code is not None else '',
            'gas_code': self.gas_code,
            'unit': unit_name(self.unit_code) if self.unit_code is not None else 'ppm',
            'decimals': self.decimals,
            'range_raw': self.range_raw,
            'address': self.address,
        }

    def read(self):
        if self.decimals is None:
            self.read_info()
        # Two single reads, as the datasheet examples do.
        raw = self.read_registers(REG_CONCENTRATION)[0]
        status = Status(self.read_registers(REG_STATUS)[0])
        info = self.info()
        concentration = raw / float(10 ** self.decimals)
        values = [
            ('gas', '%s (%s)' % (info['gas'], info['gas_description'])),
            ('concentration', '%g %s' % (concentration, info['unit'])),
            ('raw', str(raw)), ('decimals', str(self.decimals)),
            ('status_word', '0x%04X' % status.word),
            ('preheat_remaining', '%.1f' % status.preheat_remaining),
            ('address', str(self.address)),
        ]
        if self.range_raw is not None:
            values.append(('range_raw', str(self.range_raw)))
        values += [(key, str(status.flags[key])) for _, key, _ in ALARM_BITS]
        return Reading(concentration, info['unit'], info['gas'], status=status.text,
                       valid=not status.preheating, alarm=status.alarm, fault=status.fault,
                       values=values)

    # -- configuration -------------------------------------------------------

    def unlock(self):
        self.write_registers(REG_UNLOCK, [UNLOCK_KEY])

    def set_address(self, new_address):
        """Unlock, then write the Modbus address (0..254). Answers on the new address after."""
        if not 0 <= new_address <= 254:
            raise ValueError('new address must be 0..254')
        self.unlock()
        self.write_registers(REG_ADDRESS, [new_address])
        self.address = new_address

    def operating_status(self):
        word = self.read_registers(REG_COMMAND)[0]
        return word, OPERATING_STATUS.get(word, 'step %d' % (word & 0xFF) if word & 0x0100 else '?')
