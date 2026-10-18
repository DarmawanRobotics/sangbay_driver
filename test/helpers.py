"""Shared test helpers (no ROS)."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
for p in (os.path.join(HERE, '..', 'src'), HERE):
    if p not in sys.path:
        sys.path.insert(0, p)


class MemoryPort(object):
    """In-memory serial port in front of a fake device (feed/poll)."""

    def __init__(self, device):
        self.device = device
        self.pending = b''

    @property
    def in_waiting(self):
        self.pending += self.device.poll()
        return len(self.pending)

    def reset_input_buffer(self):
        self.pending = b''

    def write(self, data):
        self.pending += self.device.feed(bytes(data))
        self.device.gap()

    def read(self, n):
        self.pending += self.device.poll()
        out, self.pending = self.pending[:n], self.pending[n:]
        return out

    def readline(self):
        self.pending += self.device.poll()
        i = self.pending.find(b'\n')
        if i < 0:
            return b''
        out, self.pending = self.pending[:i + 1], self.pending[i + 1:]
        return out

    def close(self):
        pass
