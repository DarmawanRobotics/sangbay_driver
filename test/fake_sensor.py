#!/usr/bin/env python3
"""Simulated S-M4T on a pseudo terminal, for tests and trying the node without hardware.

    fake_sensor.py --link /tmp/sangbay_fake --ppm 12.5
    roslaunch sangbay_driver sangbay.launch port:=/tmp/sangbay_fake
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
from sangbay_driver.modbus import crc16, with_crc  # noqa: E402


class FakeSM4T(object):
    def __init__(self, address=0x01, gas_code=0x02, decimals=1, ppm=0.0, preheat=0.0):
        self.address = address
        self.registers = {0x2027: gas_code, 0x2031: decimals, 0x6000: 0x8000}
        self.set_ppm(ppm)
        self.preheat_until = time.time() + preheat
        self.requests = 0
        self.silent = False

    def set_ppm(self, ppm):
        self.registers[0x6001] = int(round(ppm * 10 ** self.registers[0x2031]))

    def set_alarm(self, bits):
        self.registers[0x6000] = 0x8000 | bits

    def answer(self, request):
        if len(request) != 8 or crc16(request[:6]) != struct.unpack_from('<H', request, 6)[0]:
            return b''
        address, function, register, count = struct.unpack('>BBHH', request[:6])
        if address not in (self.address, 0xFF):
            return b''
        self.requests += 1
        if self.silent:
            return b''
        if function != 0x03:
            return with_crc([self.address, function | 0x80, 1])
        values = []
        for r in range(register, register + count):
            if r == 0x6000 and time.time() < self.preheat_until:
                values.append(int((self.preheat_until - time.time()) * 10) & 0x7FFF)
            elif r in self.registers:
                values.append(self.registers[r])
            else:
                return with_crc([self.address, 0x83, 2])
        return with_crc(bytearray([self.address, 0x03, 2 * count]) + struct.pack('>%dH' % count, *values))


class PtyServer(object):
    """Serves a FakeSM4T on a pty; `path` is the device to open."""

    def __init__(self, sensor, link=None):
        self.sensor = sensor
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
        buf = b''
        while self.running:
            ready, _, _ = select.select([self.master], [], [], 0.1)
            if not ready:
                buf = b''  # a gap ends a frame
                continue
            try:
                buf += os.read(self.master, 256)
            except OSError:
                return
            while len(buf) >= 8:
                answer = self.sensor.answer(buf[:8])
                buf = buf[8:]
                if answer:
                    os.write(self.master, answer)

    def close(self):
        self.running = False
        self.thread.join(1.0)
        if self.link and os.path.islink(self.link):
            os.remove(self.link)
        os.close(self.master)
        os.close(self._slave)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--link', default='/tmp/sangbay_fake')
    p.add_argument('--address', type=int, default=1)
    p.add_argument('--ppm', type=float, default=12.5)
    p.add_argument('--decimals', type=int, default=1)
    p.add_argument('--preheat', type=float, default=0.0)
    args, _ = p.parse_known_args()  # ignores roslaunch's __name:= etc.
    server = PtyServer(FakeSM4T(args.address, decimals=args.decimals, ppm=args.ppm,
                                preheat=args.preheat), args.link)
    print('fake S-M4T on %s (%s), %.2f ppm' % (args.link, server.path, args.ppm))
    sys.stdout.flush()
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    server.close()


if __name__ == '__main__':
    main()
