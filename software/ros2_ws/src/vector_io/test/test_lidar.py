import math

from vector_io import lidar


def test_packet_roundtrip():
    p = lidar.packet(3600, 10.0, [1.0] * 12, end=18.8)
    assert len(p) == lidar.LEN
    speed, pts = lidar.parse(p)
    assert speed == 3600
    assert pts[0][0] == 10.0 and abs(pts[-1][0] - 18.8) < 1e-9 and pts[5][1] == 1.0
    bad = bytearray(p)
    bad[10] ^= 1
    assert lidar.parse(bytes(bad)) is None


def test_wrap_inside_a_packet():
    _, pts = lidar.parse(lidar.packet(3600, 355.0, [1.0] * 12, end=3.8))
    assert abs(pts[-1][0] - 3.8) < 1e-6 and pts[6][0] > 359 and pts[7][0] < 1


def feed_turn(t, ranges_at, start=0.0):
    out = []
    for k in range(45):
        a0 = (start + k * 8) % 360
        rs = [ranges_at((a0 + 8 / 11 * i) % 360) for i in range(12)]
        r = t.feed(lidar.packet(3600, a0, rs, end=(a0 + 8) % 360))
        if r:
            out.append(r)
    return out


def test_turns():
    t = lidar.Turns(bins=360)
    # a wall 2 m away, 30-60 deg clockwise from the lidar's 0; nothing elsewhere
    wall = lambda a: 2.0 if 30 <= a <= 60 else 0.0  # noqa: E731
    assert feed_turn(t, wall) == []
    turns = feed_turn(t, wall)
    assert len(turns) == 1
    ranges, intens, speed = turns[0]
    assert speed == 3600 and len(ranges) == 360
    # clockwise 45 deg is counter-clockwise 315 deg
    assert ranges[315] == 2.0 and ranges[305] == 2.0 and ranges[325] == 2.0
    assert math.isinf(ranges[45]) and math.isinf(ranges[0])
    assert intens[315] > 0


def test_yaw_offset():
    t = lidar.Turns(bins=360, yaw=math.radians(90))
    spot = lambda a: 1.5 if 359 <= a or a <= 1 else 0.0  # noqa: E731
    feed_turn(t, spot)
    ranges = feed_turn(t, spot)[0][0]
    assert ranges[90] == 1.5 and math.isinf(ranges[0])
