"""One thread per serial port: polls the modules on it, or reads a streaming sensor. No ROS."""

import threading
import time

from sangbay_driver import gt_ts01a, sltd
from sangbay_driver.module import Module
from sangbay_driver.modbus import ModbusError

STREAM_TIMEOUT = {'gt_ts01a': 3 * gt_ts01a.OUTPUT_PERIOD, 'sltd_ch4': 1.0}


def open_serial(device, baud=9600, timeout=0.3):
    import serial
    return serial.Serial(port=device, baudrate=baud, bytesize=serial.EIGHTBITS,
                         parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=timeout,
                         exclusive=True)


class Listener(object):
    """What a bus reports. Called from the bus thread."""

    def on_log(self, error, text):
        pass

    def on_info(self, sensor, info):
        pass

    def on_reading(self, sensor, reading):
        pass

    def on_error(self, sensor, message):
        pass


class Bus(object):
    def __init__(self, config, listener, rate=1.0, timeout=0.3, retries=2, reconnect_sec=2.0,
                 max_failures=3, opener=open_serial):
        self.config = config
        self.listener = listener
        self.period = 1.0 / rate
        self.timeout = timeout
        self.retries = retries
        self.reconnect_sec = reconnect_sec
        self.max_failures = max_failures
        self.opener = opener
        self.port = None
        self.devices = {}
        self.failures = dict((s.name, 0) for s in config.sensors)
        self.running = False
        self.thread = None
        self._open_error = None

    # -- lifecycle -----------------------------------------------------------

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._run, name='sangbay %s' % self.config.port)
        self.thread.daemon = True
        self.thread.start()

    def stop(self, wait=5.0):
        self.running = False
        if self.thread:
            self.thread.join(wait)
        if self.port is not None and self.config.streaming:
            for dev in self.devices.values():
                if hasattr(dev, 'stop'):
                    try:
                        dev.stop()  # laser off
                        self.port.flush()  # sent before the port closes
                    except Exception:
                        pass
        self._close()

    def _sleep(self, seconds):
        end = time.time() + seconds
        while self.running and time.time() < end:
            time.sleep(min(0.05, max(0.0, end - time.time())))

    # -- port ----------------------------------------------------------------

    def _open(self):
        timeout = 0.1 if self.config.streaming else self.timeout
        try:
            self.port = self.opener(self.config.port, self.config.baud, timeout)
        except Exception as e:
            text = 'cannot open %s: %s' % (self.config.port, e)
            if text != self._open_error:
                self.listener.on_log(True, text + ' (retrying every %g s)' % self.reconnect_sec)
                for s in self.config.sensors:
                    self.listener.on_error(s, text)
            self._open_error = text
            return False
        self._open_error = None
        self.listener.on_log(False, 'opened %s at %d baud' % (self.config.port, self.config.baud))
        lock = threading.RLock()
        self.devices = {}
        for s in self.config.sensors:
            if s.type == 'module':
                self.devices[s.name] = Module(self.port, s.address, self.retries, lock=lock)
            elif s.type == 'gt_ts01a':
                self.devices[s.name] = gt_ts01a.GtTs01a(self.port)
            else:
                self.devices[s.name] = sltd.Sltd(self.port, s.mode, s.pointer, s.check_crc)
        return True

    def _close(self):
        if self.port is not None:
            try:
                self.port.close()
            except Exception:
                pass
        self.port = None

    def _lost(self, error):
        text = '%s: %s' % (self.config.port, error)
        self.listener.on_log(True, text + ', reopening')
        for s in self.config.sensors:
            self.listener.on_error(s, text)
        self._close()

    # -- loops ---------------------------------------------------------------

    def _run(self):
        while self.running:
            if self.port is None and not self._open():
                self._sleep(self.reconnect_sec)
                continue
            try:
                if self.config.streaming:
                    self._stream_step()
                else:
                    self._poll_cycle()
            except (OSError, IOError) as e:  # serial.SerialException included: unplugged
                self._lost(e)
                self._sleep(self.reconnect_sec)

    def _check_gas(self, sensor, info):
        if sensor.expected_gas and info.get('gas') != sensor.expected_gas:
            self.listener.on_log(True, '%s: module reports %s, expected %s'
                                 % (sensor.name, info.get('gas'), sensor.expected_gas))

    def _poll_cycle(self):
        start = time.time()
        for s in self.config.sensors:
            if not self.running:
                return
            dev = self.devices[s.name]
            try:
                if dev.decimals is None:
                    info = dev.read_info()
                    self._check_gas(s, info)
                    self.listener.on_info(s, info)
                reading = dev.read().apply_threshold(s.alarm_threshold)
            except ModbusError as e:
                self.failures[s.name] += 1
                if self.failures[s.name] >= self.max_failures:
                    dev.decimals = None  # read the info again when it answers
                self.listener.on_error(s, 'address %d: %s' % (s.address, e))
                continue
            self.failures[s.name] = 0
            self.listener.on_reading(s, reading)
        if all(n >= self.max_failures for n in self.failures.values()):
            self._lost('no module answers')
            for name in self.failures:
                self.failures[name] = 0
            self._sleep(self.reconnect_sec)
            return
        self._sleep(self.period - (time.time() - start))

    def _stream_step(self):
        s = self.config.sensors[0]
        dev = self.devices[s.name]
        if self.failures[s.name] == 0 and not getattr(dev, 'started', False):
            dev.start()
            dev.started = True
            self.listener.on_info(s, dev.info())
        timeout = STREAM_TIMEOUT[s.type]
        reading = dev.read(timeout)
        if reading is None:
            self.failures[s.name] += 1
            detail = ' (last: %s)' % dev.last_error if dev.last_error else ''
            self.listener.on_error(s, 'no data for %g s%s' % (timeout, detail))
            dev.start()  # SLTD: measurement on again; GT: nothing
            if self.failures[s.name] >= self.max_failures:
                self.failures[s.name] = 0
                self._lost('no data from %s' % s.type)
                self._sleep(self.reconnect_sec)
            return
        self.failures[s.name] = 0
        self.listener.on_reading(s, reading.apply_threshold(s.alarm_threshold))
