"""Sensor list -> buses (one per serial port). No ROS.

    sensors:
      - {name: h2s, port: /dev/ttyUSB0, address: 1, expected_gas: H2S}
      - {name: co,  port: /dev/ttyUSB0, address: 2, alarm_threshold: 35}
      - {name: ch4, type: gt_ts01a, port: /dev/ttyUSB1}
      - {name: ch4_remote, type: sltd_ch4, port: /dev/ttyUSB2, mode: dynamic, pointer: true}
"""

import re

from sangbay_driver import gt_ts01a, sltd

TYPES = {
    'module': {'baud': 9600, 'streaming': False},       # S-M4, S-M4T (TTL / 485), K-5S
    'gt_ts01a': {'baud': gt_ts01a.DEFAULT_BAUD, 'streaming': True},
    'sltd_ch4': {'baud': sltd.DEFAULT_BAUD, 'streaming': True},
}
ALIASES = {'s-m4': 'module', 's-m4t': 'module', 'k-5s': 'module', 'modbus': 'module',
           'gt-ts01a': 'gt_ts01a', 'sltd-ch4-100': 'sltd_ch4', 'sltd': 'sltd_ch4'}

KEYS = {
    'module': {'name', 'type', 'port', 'baud', 'address', 'expected_gas', 'alarm_threshold'},
    'gt_ts01a': {'name', 'type', 'port', 'baud', 'alarm_threshold'},
    'sltd_ch4': {'name', 'type', 'port', 'baud', 'alarm_threshold', 'mode', 'pointer', 'check_crc'},
}
NAME = re.compile(r'^[A-Za-z][A-Za-z0-9_]*$')


class ConfigError(ValueError):
    pass


class SensorConfig(object):
    def __init__(self, name, type, port, baud, address=0xFF, expected_gas='', alarm_threshold=None,
                 mode='dynamic', pointer=True, check_crc=True):
        self.name = name
        self.type = type
        self.port = port
        self.baud = baud
        self.address = address
        self.expected_gas = expected_gas
        self.alarm_threshold = alarm_threshold
        self.mode = mode
        self.pointer = pointer
        self.check_crc = check_crc

    @property
    def hardware_id(self):
        return '%s#%d' % (self.port, self.address) if self.type == 'module' else self.port

    def __repr__(self):
        return 'SensorConfig(%s, %s, %s)' % (self.name, self.type, self.hardware_id)


class BusConfig(object):
    def __init__(self, port, baud, streaming, sensors):
        self.port = port
        self.baud = baud
        self.streaming = streaming
        self.sensors = sensors


def parse_sensor(entry, index, default_port=None):
    if not isinstance(entry, dict):
        raise ConfigError('sensors[%d]: expected a mapping, got %r' % (index, entry))
    where = 'sensors[%d]' % index
    name = entry.get('name')
    if not isinstance(name, str) or not NAME.match(name):
        raise ConfigError('%s: name must be a ROS name (letters, digits, _), got %r' % (where, name))
    where = 'sensor %s' % name
    kind = str(entry.get('type', 'module')).lower()
    kind = ALIASES.get(kind, kind)
    if kind not in TYPES:
        raise ConfigError('%s: type must be one of %s' % (where, ', '.join(sorted(TYPES))))
    unknown = set(entry) - KEYS[kind]
    if unknown:
        raise ConfigError('%s: unknown keys for type %s: %s' % (where, kind, ', '.join(sorted(unknown))))
    port = entry.get('port', default_port)
    if not port:
        raise ConfigError('%s: no port' % where)
    try:
        baud = int(entry.get('baud', TYPES[kind]['baud']))
        address = int(entry.get('address', 0xFF))
        threshold = entry.get('alarm_threshold')
        threshold = None if threshold in (None, '') else float(threshold)
    except (TypeError, ValueError) as e:
        raise ConfigError('%s: %s' % (where, e))
    if not 0 <= address <= 0xFF:
        raise ConfigError('%s: address must be 0..255' % where)
    mode = entry.get('mode', 'dynamic')
    if mode not in ('dynamic', 'static'):
        raise ConfigError("%s: mode must be 'dynamic' or 'static'" % where)
    return SensorConfig(name, kind, str(port), baud, address, str(entry.get('expected_gas', '') or ''),
                        threshold, mode, bool(entry.get('pointer', True)),
                        bool(entry.get('check_crc', True)))


def parse(entries, default_port=None):
    """List of sensor mappings -> list of BusConfig. Raises ConfigError."""
    if not entries:
        raise ConfigError('no sensors configured')
    if not isinstance(entries, list):
        raise ConfigError('sensors must be a list')
    sensors = [parse_sensor(e, i, default_port) for i, e in enumerate(entries)]

    names = [s.name for s in sensors]
    duplicates = sorted(set(n for n in names if names.count(n) > 1))
    if duplicates:
        raise ConfigError('duplicate sensor names: %s' % ', '.join(duplicates))

    buses = []
    for port in sorted(set(s.port for s in sensors), key=lambda p: [s.port for s in sensors].index(p)):
        group = [s for s in sensors if s.port == port]
        bauds = set(s.baud for s in group)
        if len(bauds) > 1:
            raise ConfigError('%s: sensors with different baud rates %s' % (port, sorted(bauds)))
        streaming = TYPES[group[0].type]['streaming']
        if streaming and len(group) > 1:
            raise ConfigError('%s: a %s sensor needs the port for itself' % (port, group[0].type))
        if any(TYPES[s.type]['streaming'] for s in group) and not streaming:
            raise ConfigError('%s: streaming sensors cannot share a port with modules' % port)
        addresses = [s.address for s in group]
        if len(group) > 1:
            if 0xFF in addresses:
                raise ConfigError('%s: address 255 is the factory default; with several modules on '
                                  'one bus give each its own (sangbay_tool set-address)' % port)
            same = sorted(set(a for a in addresses if addresses.count(a) > 1))
            if same:
                raise ConfigError('%s: several modules on address %s; give each one its own '
                                  '(sangbay_tool set-address)' % (port, ', '.join(map(str, same))))
        buses.append(BusConfig(port, bauds.pop(), streaming, group))
    return buses
