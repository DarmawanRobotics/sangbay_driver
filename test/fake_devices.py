#!/usr/bin/env python3
"""Simulated Sangbay devices on pseudo terminals, for tests and trying the node without hardware.

    fake_devices.py --bus /tmp/sangbay_bus --gt /tmp/sangbay_gt --sltd /tmp/sangbay_sltd
    roslaunch sangbay_driver sangbay.launch config:=$(rospack find sangbay_driver)/config/example_fake.yaml

--bus: modules on one RS485 line: address 1 H2S 12.5 ppm, address 2 CO 3 ppm, address 3 CH4 0 %LEL
--gt: GT-TS01A at 0.12 %VOL; --sltd: SLTD-CH4-100 at 250 ppm*m
"""

import argparse
import os
import pty
import select
import struct
import sys
import threading
import time
import tty

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))
from sangbay_driver import gt_ts01a, sltd  # noqa: E402
from sangbay_driver.modbus import crc16, with_crc  # noqa: E402


class FakeModule(object):
    def __init__(self, address=0xFF, gas_code=0x02, decimals=1, unit=2, value=0.0, preheat=0.0,
                 range_raw=1000):
        self.address = address
        self.registers = {0x2027: gas_code, 0x2030: unit, 0x2031: decimals, 0x202B: range_raw,
                          0x6000: 0x8000, 0x6002: 512, 0x6006: 0x0001, 0x2011: address}
        self.set_value(value)
        self.preheat_until = time.time() + preheat
        self.unlocked = False
        self.silent = False
        self.requests = 0

    def set_value(self, value):
        self.registers[0x6001] = int(round(value * 10 ** self.registers[0x2031]))

    def set_alarm(self, bits):
        self.registers[0x6000] = 0x8000 | bits

    def read(self, register):
        if register == 0x6000 and time.time() < self.preheat_until:
            return int((self.preheat_until - time.time()) * 10) & 0x7FFF
        return self.registers.get(register)

    def write(self, register, values):
        if register == 0x4FFF:
            self.unlocked = values == [0x55AA]
            return self.unlocked
        if register == 0x2011:
            if not self.unlocked or not 0 <= values[0] <= 254:
                return False
            self.address = values[0]
            self.registers[0x2011] = values[0]
            self.unlocked = False
            return True
        if register in (0x2030, 0x2031, 0x6006):
            self.registers[register] = values[0]
            return True
        return False


class FakeModuleBus(object):
    """Several modules on one line; each answers its own address (and 0xFF)."""

    def __init__(self, modules):
        self.modules = list(modules)
        self.buffer = b''

    def feed(self, data):
        self.buffer += data
        out = b''
        while len(self.buffer) >= 8:
            size = 8
            if self.buffer[1] == 0x10:
                if len(self.buffer) < 7:
                    break
                size = 9 + self.buffer[6]
                if len(self.buffer) < size:
                    break
            frame, self.buffer = self.buffer[:size], self.buffer[size:]
            if crc16(frame[:-2]) != struct.unpack_from('<H', frame, size - 2)[0]:
                self.buffer = b''
                break
            out += self.answer(frame)
        return out

    def poll(self):
        return b''

    def gap(self):
        self.buffer = b''  # a pause ends a frame

    def answer(self, frame):
        address, function = frame[0], frame[1]
        register, count = struct.unpack('>HH', frame[2:6])
        targets = [m for m in self.modules if address in (m.address, 0xFF)]
        if len(targets) != 1:
            return b''  # nobody, or a collision on the line
        m = targets[0]
        m.requests += 1
        if m.silent:
            return b''
        if function == 0x03:
            values = [m.read(r) for r in range(register, register + count)]
            if None in values:
                return with_crc([m.address, 0x83, 2])
            return with_crc(bytearray([m.address, 0x03, 2 * count]) + struct.pack('>%dH' % count, *values))
        if function == 0x10:
            values = list(struct.unpack('>%dH' % count, frame[7:7 + 2 * count]))
            reply_address = m.address
            if not m.write(register, values):
                return with_crc([reply_address, 0x90, 3])
            return with_crc(bytearray([reply_address, 0x10]) + frame[2:6])
        return with_crc([m.address, function | 0x80, 1])


class FakeGt(object):
    def __init__(self, ch4=0.12, temperature=25.0, pressure=101.3, status=0, period=0.5):
        self.ch4, self.temperature, self.pressure, self.status = ch4, temperature, pressure, status
        self.period = period
        self.next = time.time()
        self.silent = False

    def feed(self, data):
        return b''

    def gap(self):
        pass

    def poll(self):
        if self.silent or time.time() < self.next:
            return b''
        self.next = time.time() + self.period
        return gt_ts01a.make_line(self.ch4, self.temperature, self.pressure, self.status)


class FakeSltd(object):
    def __init__(self, concentration=250, light=3000, temperature=250, period=0.05):
        self.concentration, self.light, self.temperature = concentration, light, temperature
        self.period = period
        self.measuring = False
        self.pointer = None
        self.mode = None
        self.buffer = bytearray()
        self.next = time.time()

    def gap(self):
        pass

    def feed(self, data):
        self.buffer += bytearray(data)
        out = b''
        while len(self.buffer) >= 8:
            if self.buffer[0] != 0xAA:
                self.buffer.pop(0)
                continue
            size = self.buffer[1] + 5
            if len(self.buffer) < size:
                break
            frame = bytes(self.buffer[:size])
            del self.buffer[:size]
            payload = frame[1:-3]
            if frame[-1] != 0x55 or crc16(payload) != frame[-3] | frame[-2] << 8:
                continue
            command, high, low = frame[2], frame[3], frame[4]
            if command == sltd.CMD_START:
                self.measuring, self.pointer, self.mode = True, bool(high), 'static' if low else 'dynamic'
            elif command == sltd.CMD_STOP:
                self.measuring = False
            out += sltd.ack_frame(command, high, low, 0)
        return out

    def poll(self):
        if not self.measuring or time.time() < self.next:
            return b''
        self.next = time.time() + self.period
        return sltd.data_frame(self.concentration, self.light, self.temperature)


class PtyServer(object):
    """Serves a device on a pty; open `path` (or `link`) like a serial port."""

    def __init__(self, device, link=None):
        self.device = device
        self.master, slave = pty.openpty()
        tty.setraw(slave)
        self.path = os.ttyname(slave)
        self._slave = slave
        self.link = link
        if link:
            if os.path.lexists(link):
                os.remove(link)
            os.symlink(self.path, link)
        self.running = True
        self.thread = threading.Thread(target=self._run)
        self.thread.daemon = True
        self.thread.start()

    def _run(self):
        while self.running:
            ready, _, _ = select.select([self.master], [], [], 0.01)
            out = b''
            if ready:
                try:
                    out += self.device.feed(os.read(self.master, 256))
                except OSError:
                    return
            else:
                self.device.gap()
            out += self.device.poll()
            if out:
                try:
                    os.write(self.master, out)
                except OSError:
                    return

    def close(self):
        self.running = False
        self.thread.join(1.0)
        if self.link and os.path.islink(self.link):
            os.remove(self.link)
        os.close(self.master)
        os.close(self._slave)


def demo_bus():
    return FakeModuleBus([
        FakeModule(1, gas_code=0x02, decimals=1, unit=2, value=12.5),   # H2S ppm
        FakeModule(2, gas_code=0x01, decimals=0, unit=2, value=3),      # CO ppm
        FakeModule(3, gas_code=0x06, decimals=0, unit=1, value=0),      # CH4 %LEL
    ])


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--bus', help='link for the module bus')
    p.add_argument('--gt', help='link for the GT-TS01A')
    p.add_argument('--sltd', help='link for the SLTD-CH4-100')
    args, _ = p.parse_known_args()  # ignores roslaunch's __name:= etc.
    servers = []
    if args.bus:
        servers.append(PtyServer(demo_bus(), args.bus))
    if args.gt:
        servers.append(PtyServer(FakeGt(), args.gt))
    if args.sltd:
        servers.append(PtyServer(FakeSltd(), args.sltd))
    if not servers:
        p.error('give at least one of --bus, --gt, --sltd')
    for s in servers:
        print('%s -> %s (%s)' % (s.link, s.path, type(s.device).__name__))
    sys.stdout.flush()
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    for s in servers:
        s.close()


if __name__ == '__main__':
    main()
