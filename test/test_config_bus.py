#!/usr/bin/env python3
import threading
import time
import unittest

import helpers  # noqa: F401
from fake_devices import FakeGt, FakeModule, FakeModuleBus, FakeSltd, PtyServer
from sangbay_driver import config
from sangbay_driver.bus import Bus, Listener, open_serial


class TestConfig(unittest.TestCase):
    def test_grouping(self):
        buses = config.parse([
            {'name': 'a', 'port': '/dev/ttyUSB0', 'address': 1},
            {'name': 'b', 'port': '/dev/ttyUSB0', 'address': 2, 'alarm_threshold': '10'},
            {'name': 'c', 'type': 'GT-TS01A', 'port': '/dev/ttyUSB1'},
            {'name': 'd', 'type': 'sltd_ch4', 'port': '/dev/ttyUSB2', 'mode': 'static', 'pointer': False},
            {'name': 'e'},
        ], default_port='/dev/ttyS9')
        self.assertEqual([b.port for b in buses], ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyUSB2', '/dev/ttyS9'])
        self.assertEqual([b.baud for b in buses], [9600, 115200, 115200, 9600])
        self.assertEqual([len(b.sensors) for b in buses], [2, 1, 1, 1])
        self.assertEqual(buses[0].sensors[1].alarm_threshold, 10.0)
        self.assertEqual(buses[1].sensors[0].type, 'gt_ts01a')
        self.assertEqual((buses[2].sensors[0].mode, buses[2].sensors[0].pointer), ('static', False))
        self.assertEqual(buses[3].sensors[0].address, 255)
        self.assertEqual(buses[0].sensors[0].hardware_id, '/dev/ttyUSB0#1')

    def check(self, entries, message):
        with self.assertRaises(config.ConfigError) as cm:
            config.parse(entries)
        self.assertIn(message, str(cm.exception))

    def test_errors(self):
        p = '/dev/ttyUSB0'
        self.check([], 'no sensors')
        self.check([{'port': p}], 'name')
        self.check([{'name': 'my-sensor', 'port': p}], 'name')
        self.check([{'name': 'a'}], 'no port')
        self.check([{'name': 'a', 'port': p, 'type': 'x'}], 'type')
        self.check([{'name': 'a', 'port': p, 'mode': 'static'}], 'unknown keys')
        self.check([{'name': 'a', 'port': p, 'address': 300}], '0..255')
        self.check([{'name': 'a', 'port': p, 'address': 1}, {'name': 'a', 'port': p, 'address': 2}], 'duplicate')
        self.check([{'name': 'a', 'port': p, 'address': 1}, {'name': 'b', 'port': p, 'address': 1}], 'address 1')
        self.check([{'name': 'a', 'port': p}, {'name': 'b', 'port': p, 'address': 1}], 'factory default')
        self.check([{'name': 'a', 'port': p, 'address': 1}, {'name': 'b', 'port': p, 'address': 2, 'baud': 19200}],
                   'baud')
        self.check([{'name': 'a', 'port': p, 'type': 'gt_ts01a'}, {'name': 'b', 'port': p, 'type': 'gt_ts01a'}],
                   'for itself')
        self.check([{'name': 'a', 'port': p, 'baud': 115200}, {'name': 'b', 'port': p, 'type': 'gt_ts01a'}],
                   'share')
        self.check([{'name': 'a', 'port': p, 'type': 'sltd_ch4', 'mode': 'x'}], 'mode')


class Recorder(Listener):
    def __init__(self):
        self.readings = {}
        self.errors = {}
        self.infos = {}
        self.logs = []
        self.lock = threading.Lock()

    def on_log(self, error, text):
        self.logs.append(text)

    def on_info(self, sensor, info):
        self.infos[sensor.name] = info

    def on_reading(self, sensor, reading):
        with self.lock:
            self.readings.setdefault(sensor.name, []).append(reading)

    def on_error(self, sensor, message):
        with self.lock:
            self.errors.setdefault(sensor.name, []).append(message)

    def count(self, name):
        with self.lock:
            return len(self.readings.get(name, []))


def wait_for(cond, timeout=5.0):
    end = time.time() + timeout
    while time.time() < end:
        if cond():
            return True
        time.sleep(0.02)
    return cond()


class TestBus(unittest.TestCase):
    def setUp(self):
        self.servers = []

    def tearDown(self):
        for b in getattr(self, 'buses', []):
            b.stop()
        for s in self.servers:
            s.close()

    def serve(self, device):
        s = PtyServer(device)
        self.servers.append(s)
        return s.path

    def run_buses(self, entries, **kw):
        self.rec = Recorder()
        self.buses = [Bus(b, self.rec, **kw) for b in config.parse(entries)]
        for b in self.buses:
            b.start()
        return self.rec

    def test_everything(self):
        h2s = FakeModule(1, gas_code=0x02, decimals=1, value=12.5)
        co = FakeModule(2, gas_code=0x01, decimals=0, value=40)
        bus = self.serve(FakeModuleBus([h2s, co]))
        gt = self.serve(FakeGt(ch4=0.3, period=0.2))
        sl_dev = FakeSltd(concentration=77)
        sl = self.serve(sl_dev)
        rec = self.run_buses([
            {'name': 'h2s', 'port': bus, 'address': 1, 'expected_gas': 'CO'},
            {'name': 'co', 'port': bus, 'address': 2, 'alarm_threshold': 35},
            {'name': 'missing', 'port': bus, 'address': 9},
            {'name': 'ch4', 'type': 'gt_ts01a', 'port': gt},
            {'name': 'remote', 'type': 'sltd_ch4', 'port': sl},
        ], rate=10, retries=0, timeout=0.1)
        for name in ('h2s', 'co', 'ch4', 'remote'):
            self.assertTrue(wait_for(lambda: rec.count(name) >= 2), name)
        self.assertEqual(rec.readings['h2s'][-1].concentration, 12.5)
        self.assertTrue(rec.readings['co'][-1].alarm)
        self.assertEqual(rec.readings['ch4'][-1].concentration, 0.3)
        self.assertEqual(rec.readings['remote'][-1].concentration, 77.0)
        self.assertIn('no answer', rec.errors['missing'][-1])
        self.assertTrue(any('reports H2S, expected CO' in t for t in rec.logs))
        self.assertEqual(rec.infos['co']['gas'], 'CO')
        for b in self.buses:
            b.stop()
        self.assertTrue(wait_for(lambda: not sl_dev.measuring))  # laser off on stop

    def test_module_recovers(self):
        m = FakeModule(1, value=1.0)
        path = self.serve(FakeModuleBus([m]))
        rec = self.run_buses([{'name': 'a', 'port': path, 'address': 1}], rate=20, retries=0,
                             timeout=0.05, max_failures=2, reconnect_sec=0.1)
        self.assertTrue(wait_for(lambda: rec.count('a') >= 1))
        m.silent = True
        self.assertTrue(wait_for(lambda: len(rec.errors.get('a', [])) >= 3))
        self.assertTrue(wait_for(lambda: any('reopening' in t for t in rec.logs)))
        m.registers[0x2031] = 2  # replaced module: info is read again
        m.set_value(1.0)
        m.silent = False
        n = rec.count('a')
        self.assertTrue(wait_for(lambda: rec.count('a') > n + 1))
        self.assertEqual(rec.readings['a'][-1].concentration, 1.0)

    def test_port_appears_later(self):
        m = FakeModule(1, value=2.0)
        server = {}

        def opener(port, baud, timeout):
            if 'path' not in server:
                raise OSError('No such file or directory')
            return open_serial(server['path'], baud, timeout)
        self.rec = rec = Recorder()
        b = Bus(config.parse([{'name': 'a', 'port': '/dev/later', 'address': 1}])[0], rec,
                rate=20, retries=0, timeout=0.05, reconnect_sec=0.1, opener=opener)
        self.buses = [b]
        b.start()
        self.assertTrue(wait_for(lambda: 'a' in rec.errors))
        self.assertIn('cannot open', rec.errors['a'][0])
        server['path'] = self.serve(FakeModuleBus([m]))
        self.assertTrue(wait_for(lambda: rec.count('a') >= 1))

    def test_stream_silence(self):
        dev = FakeGt(period=0.1)
        path = self.serve(dev)
        import sangbay_driver.bus as busmod
        old = dict(busmod.STREAM_TIMEOUT)
        busmod.STREAM_TIMEOUT['gt_ts01a'] = 0.3
        try:
            rec = self.run_buses([{'name': 'g', 'type': 'gt_ts01a', 'port': path}], max_failures=2,
                                 reconnect_sec=0.1)
            self.assertTrue(wait_for(lambda: rec.count('g') >= 1))
            dev.silent = True
            self.assertTrue(wait_for(lambda: any('no data' in e for e in rec.errors.get('g', []))))
            self.assertTrue(wait_for(lambda: any('reopening' in t for t in rec.logs)))
            dev.silent = False
            n = rec.count('g')
            self.assertTrue(wait_for(lambda: rec.count('g') > n))
        finally:
            busmod.STREAM_TIMEOUT.update(old)


if __name__ == '__main__':
    unittest.main()
