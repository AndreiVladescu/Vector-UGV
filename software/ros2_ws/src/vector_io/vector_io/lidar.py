"""LDS01RR (Neato XV-11 protocol) packets -> full turns. A packet is 22 bytes: 0xFA, index
0xA0-0xF9 (90 packets a turn, 4 degrees each), u16 speed in 1/64 rpm, 4 x (u16 distance mm
with bit 15 = invalid and bit 14 = strength warning, u16 signal strength), u16 checksum.
Index i, reading j is at (4 i + j) degrees. Whether that runs clockwise or counter-clockwise
seen from above depends on the unit: check with an object on one side and set `clockwise`."""
import math
import struct

LEN = 22
_PKT = struct.Struct('<BBH' + 'HH' * 4 + 'H')


def checksum(p):
    c = 0
    for i in range(10):
        c = (c << 1) + (p[2 * i] | p[2 * i + 1] << 8)
    return ((c & 0x7FFF) + (c >> 15)) & 0x7FFF


def parse(p):
    """-> (rpm, [(angle deg, metres or 0, strength)] x 4), None if malformed."""
    if len(p) != LEN or p[0] != 0xFA or not 0xA0 <= p[1] <= 0xF9:
        return None
    f = _PKT.unpack(p)
    if checksum(p) != f[-1]:
        return None
    base = (p[1] - 0xA0) * 4
    pts = []
    for j in range(4):
        raw, strength = f[3 + 2 * j], f[4 + 2 * j]
        mm = 0 if raw & 0x8000 else raw & 0x3FFF
        pts.append((base + j, mm / 1000, strength))
    return f[2] / 64, pts


def packet(rpm, index, ranges_m, strength=100):
    """Build a packet (for tests and fakes): index 0-89, four ranges in metres (0 = invalid)."""
    fields = [0xFA, 0xA0 + index, int(round(rpm * 64))]
    for r in ranges_m:
        mm = int(round(r * 1000))
        fields += [mm & 0x3FFF if mm else 0x8000, strength]
    raw = bytearray(_PKT.pack(*fields, 0))
    raw[20:22] = struct.pack('<H', checksum(raw))
    return bytes(raw)


class Turns:
    """Collects packets into `bins` angle bins; feed() returns (ranges, intensities, rpm) each
    time the index wraps, ranges in ROS order (counter-clockwise from the lidar's 0 + yaw), inf
    where nothing came back."""

    def __init__(self, bins=360, yaw=0.0, clockwise=False, range_min=0.15, range_max=6.0):
        self.bins, self.yaw, self.clockwise = bins, yaw, clockwise
        self.range_min, self.range_max = range_min, range_max
        self.last_index = None
        self.rpm = 0.0
        self._clear()

    def _clear(self):
        self.ranges = [math.inf] * self.bins
        self.intens = [0.0] * self.bins
        self.packets = 0

    def feed(self, p):
        parsed = parse(p)
        if parsed is None:
            return None
        rpm, pts = parsed
        index = p[1] - 0xA0
        out = None
        if self.last_index is not None and index < self.last_index and self.packets:
            out = (self.ranges, self.intens, self.rpm)
            self._clear()
        self.last_index, self.rpm = index, rpm
        self.packets += 1
        for ang, r, strength in pts:
            if not self.range_min <= r <= self.range_max:
                continue
            a = math.radians(ang)
            a = ((-a if self.clockwise else a) + self.yaw) % (2 * math.pi)
            k = int(a / (2 * math.pi) * self.bins + 0.5) % self.bins
            if r < self.ranges[k]:
                self.ranges[k], self.intens[k] = r, float(strength)
        return out
