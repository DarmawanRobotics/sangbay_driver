"""GT-TS01A laser (TDLAS) methane sensor, 0-5 %VOL. No ROS.

TTL UART 115200 8N1. The sensor sends one ASCII line every 2 s, unrequested:

    +000.00 +22.3 +101.67 00 37<CR><LF>
    CH4 %VOL, compensation temperature C, pressure kPa, status (hex), XOR of the bytes before it
"""

import re
import time

from sangbay_driver.reading import Reading

DEFAULT_BAUD = 115200
OUTPUT_PERIOD = 2.0

LINE = re.compile(r'^\s*([+-]?\d+\.\d+) ?([+-]?\d+\.\d+) ?([+-]?\d+\.\d+) ?([0-9A-Fa-f]{2}) ?([0-9A-Fa-f]{2})\s*$')

# Status code bits
STATUS_BITS = [
    (4, 'temperature_control', 'TEMP-CONTROL'),
    (3, 'air_pressure', 'PRESSURE'),
    (2, 'low_light', 'LOW-LIGHT'),
    (1, 'high_light', 'HIGH-LIGHT'),
    (0, 'peak_position', 'PEAK-POSITION'),
]


class FrameError(Exception):
    pass


def xor(data):
    x = 0
    for b in bytearray(data):
        x ^= b
    return x


def parse_line(line):
    """(ch4 %VOL, temperature C, pressure kPa, status) from one line, or FrameError."""
    if isinstance(line, bytes):
        line = line.decode('ascii', 'replace')
    m = LINE.match(line)
    if not m:
        raise FrameError('not a GT-TS01A line: %r' % line.strip())
    checked = line[:m.start(5)].lstrip()
    if xor(checked.encode('ascii')) != int(m.group(5), 16):
        raise FrameError('XOR mismatch: %r' % line.strip())
    return float(m.group(1)), float(m.group(2)), float(m.group(3)), int(m.group(4), 16)


def make_line(ch4, temperature, pressure, status=0):
    """The line the sensor would send (for tests and the simulator)."""
    body = '%+07.2f %+05.1f %+07.2f %02X ' % (ch4, temperature, pressure, status)
    return ('%s%02X\r\n' % (body, xor(body.encode('ascii')))).encode('ascii')


def to_reading(ch4, temperature, pressure, status):
    flags = dict((key, bool(status & (1 << bit))) for bit, key, _ in STATUS_BITS)
    names = [label for bit, key, label in STATUS_BITS if flags[key]]
    text = ', '.join(names) if names else 'NORMAL'
    values = [('gas', 'CH4 (methane, laser)'), ('concentration', '%.2f %%VOL' % ch4),
              ('temperature', '%.1f' % temperature), ('pressure', '%.2f' % pressure),
              ('status_code', '%02X' % status)]
    values += [(key, str(flags[key])) for _, key, _ in STATUS_BITS]
    return Reading(ch4, '%VOL', 'CH4', status=text, fault=bool(status), values=values,
                   extra={'temperature': temperature, 'pressure': pressure})


class GtTs01a(object):
    """Reads lines from `port` (anything with readline, e.g. a pyserial Serial)."""

    type = 'gt_ts01a'

    def __init__(self, port):
        self.port = port
        self.errors = 0
        self.last_error = ''

    def start(self):
        pass  # the sensor talks by itself

    def info(self):
        return {'gas': 'CH4', 'unit': '%VOL', 'gas_description': 'methane (GT-TS01A laser)'}

    def read(self, timeout=OUTPUT_PERIOD * 2):
        """Next valid reading, or None after `timeout` s."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            line = self.port.readline()
            if not line:
                continue
            try:
                return to_reading(*parse_line(line))
            except FrameError as e:
                self.errors += 1
                self.last_error = str(e)
        return None
