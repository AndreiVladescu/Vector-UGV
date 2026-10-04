import math

from vector_io import lidar


def test_packet_roundtrip():
    p = lidar.packet(300, 0, [1.0, 0.0, 2.5, 0.2])
    assert len(p) == lidar.LEN
    rpm, pts = lidar.parse(p)
    assert rpm == 300
    assert [a for a, _, _ in pts] == [0, 1, 2, 3]
    assert [r for _, r, _ in pts] == [1.0, 0.0, 2.5, 0.2]  # invalid reads as 0
    bad = bytearray(p)
    bad[6] ^= 1
    assert lidar.parse(bytes(bad)) is None
    assert lidar.parse(b'\xfa\x10' + p[2:]) is None  # index out of range


def test_same_check_value_as_the_firmware():
    # firmware/io-node/test/test_io.c checks the same packet
    p = bytearray(22)
    p[0], p[1] = 0xFA, 0xA0
    p[2], p[3] = (300 * 64) & 0xFF, (300 * 64) >> 8
    for i in range(4):
        p[4 + 4 * i], p[5 + 4 * i], p[6 + 4 * i] = 1000 & 0xFF, (1000 >> 8) & 0x3F, 100
    assert lidar.checksum(p) == 0x3063


def feed_turn(t, ranges_at, rpm=300):
    out = []
    for i in range(90):
        r = t.feed(lidar.packet(rpm, i, [ranges_at(4 * i + j) for j in range(4)]))
        if r:
            out.append(r)
    return out


def test_turns():
    t = lidar.Turns()
    wall = lambda a: 2.0 if 30 <= a <= 60 else 0.0  # noqa: E731
    assert feed_turn(t, wall) == []
    turns = feed_turn(t, wall)
    assert len(turns) == 1
    ranges, intens, rpm = turns[0]
    assert rpm == 300 and len(ranges) == 360
    assert ranges[45] == 2.0 and ranges[30] == 2.0 and ranges[60] == 2.0
    assert math.isinf(ranges[61]) and math.isinf(ranges[315])
    assert intens[45] > 0


def test_clockwise_and_yaw():
    t = lidar.Turns(clockwise=True, yaw=math.radians(90))
    spot = lambda a: 1.5 if a == 10 else 0.0  # noqa: E731
    feed_turn(t, spot)
    ranges = feed_turn(t, spot)[0][0]
    assert ranges[80] == 1.5  # 10 degrees clockwise, turned by 90
    assert sum(1 for r in ranges if not math.isinf(r)) == 1


def test_out_of_range_dropped():
    t = lidar.Turns()
    feed_turn(t, lambda a: 0.1)
    assert all(math.isinf(r) for r in feed_turn(t, lambda a: 0.1)[0][0])
