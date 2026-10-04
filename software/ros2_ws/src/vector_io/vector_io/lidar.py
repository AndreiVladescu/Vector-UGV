"""LD19 packets -> full turns. A packet: 0x54 0x2C, u16 speed deg/s, u16 start angle (0.01 deg),
12 x (u16 mm, u8 intensity), u16 end angle, u16 timestamp ms, CRC-8. The LD19 turns
clockwise seen from above, so ROS angles (counter-clockwise) are the negative."""
import math
import struct

LEN = 47
_PKT = struct.Struct('<BBHH' + 'HB' * 12 + 'HHB')


def crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x4D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def parse(p):
    """-> (speed deg/s, [(angle deg clockwise, metres or 0, intensity)] x 12), None if malformed."""
    if len(p) != LEN or p[0] != 0x54 or p[1] != 0x2C or crc8(p[:-1]) != p[-1]:
        return None
    f = _PKT.unpack(p)
    speed, start, end = f[2], f[3] / 100, f[28] / 100
    if end < start:
        end += 360
    step = (end - start) / 11
    pts = [((start + step * i) % 360, f[4 + 2 * i] / 1000, f[5 + 2 * i]) for i in range(12)]
    return speed, pts


def packet(speed, start, ranges_m, intensity=200, end=None, ts=0):
    """Build a packet (for tests and fakes): start / end in degrees clockwise."""
    end = (start + 8) % 360 if end is None else end
    fields = [0x54, 0x2C, speed, int(round(start * 100)) % 36000]
    for r in ranges_m:
        fields += [int(round(r * 1000)), intensity]
    raw = _PKT.pack(*fields, int(round(end * 100)) % 36000, ts, 0)
    return raw[:-1] + bytes([crc8(raw[:-1])])


class Turns:
    """Collects points into `bins` angle bins; turn() returns (ranges, intensities, speed)
    each time the start angle wraps past 0, ranges in ROS order (counter-clockwise from the
    lidar's 0 + yaw), inf where nothing came back."""

    def __init__(self, bins=450, yaw=0.0, range_min=0.02, range_max=12.0):
        self.bins, self.yaw = bins, yaw
        self.range_min, self.range_max = range_min, range_max
        self.last_start = None
        self.speed = 0
        self._clear()

    def _clear(self):
        self.ranges = [math.inf] * self.bins
        self.intens = [0.0] * self.bins
        self.points = 0

    def feed(self, p):
        """One packet; a finished turn as (ranges, intensities, speed deg/s) or None."""
        parsed = parse(p)
        if parsed is None:
            return None
        speed, pts = parsed
        out = None
        start = pts[0][0]
        if self.last_start is not None and start < self.last_start and self.points:
            out = (self.ranges, self.intens, self.speed)
            self._clear()
        self.last_start, self.speed = start, speed
        for ang, r, inten in pts:
            if not self.range_min <= r <= self.range_max:
                continue
            a = (-math.radians(ang) + self.yaw) % (2 * math.pi)
            k = int(a / (2 * math.pi) * self.bins + 0.5) % self.bins
            if r < self.ranges[k]:
                self.ranges[k], self.intens[k] = r, float(inten)
            self.points += 1
        return out
