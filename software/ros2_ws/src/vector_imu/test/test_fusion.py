import math

import pytest

from vector_imu import fusion as f

G = 9.80665
DIP = math.radians(60)  # Bucharest-ish: the field points north and steeply down
FIELD_ENU = (0.0, 48e-6 * math.cos(DIP), -48e-6 * math.sin(DIP))


def body(q, v):
    return f.rotate(f.qconj(q), v)


def pose(roll, pitch, yaw):
    qz = f.from_axis_angle((0, 0, 1), yaw)
    qy = f.from_axis_angle((0, 1, 0), pitch)
    qx = f.from_axis_angle((1, 0, 0), roll)
    return f.qmul(f.qmul(qz, qy), qx)


def angle_between(a, b):
    d = abs(sum(x * y for x, y in zip(a, b)))
    return 2 * math.acos(min(1.0, d))


@pytest.mark.parametrize('rpy', [(0, 0, 0), (0.2, -0.1, 1.0), (-0.3, 0.25, -2.5), (0.0, 0.0, math.pi / 2)])
def test_static_orientation_with_compass(rpy):
    q = pose(*rpy)
    fu = f.Fusion()
    for _ in range(10):
        est = fu.update((0, 0, 0), body(q, (0, 0, G)), body(q, FIELD_ENU), 0.01)
    assert angle_between(est, q) < 1e-6
    r, p, y = f.euler(est)
    assert abs(r - rpy[0]) < 1e-6 and abs(p - rpy[1]) < 1e-6
    assert abs(math.remainder(y - rpy[2], 2 * math.pi)) < 1e-6


def test_facing_north_is_yaw_90():
    q = pose(0, 0, math.pi / 2)
    acc, mag = body(q, (0, 0, G)), body(q, FIELD_ENU)
    assert abs(mag[0] - 48e-6 * math.cos(DIP)) < 1e-9  # the body x axis points at magnetic north
    est = f.Fusion().update((0, 0, 0), acc, mag, 0.01)
    assert abs(f.euler(est)[2] - math.pi / 2) < 1e-6


def test_declination():
    # 6 deg east: facing magnetic north is facing 6 deg east of true north
    q = pose(0, 0, math.pi / 2)
    est = f.Fusion(declination=math.radians(6)).update((0, 0, 0), body(q, (0, 0, G)), body(q, FIELD_ENU), 0.01)
    assert abs(f.euler(est)[2] - math.radians(84)) < 1e-6


def test_gyro_tracks_a_turn_and_the_compass_holds_it():
    fu = f.Fusion(gain=0.5)
    q = pose(0.1, 0, 0)
    fu.update((0, 0, 0), body(q, (0, 0, G)), body(q, FIELD_ENU), 0.01)
    rate = 0.5  # rad/s about the body z
    dt = 0.005
    for k in range(400):  # 2 s: one radian
        q = f.qnorm(f.qmul(q, f.from_axis_angle((0, 0, 1), rate * dt)))
        est = fu.update((0, 0, rate + 0.01), body(q, (0, 0, G)), body(q, FIELD_ENU), dt)  # with a gyro bias
    assert angle_between(est, q) < math.radians(1.5)


def test_without_compass_yaw_comes_from_the_gyro():
    fu = f.Fusion()
    q = pose(0, 0, 0)
    fu.update((0, 0, 0), body(q, (0, 0, G)), None, 0.01)
    for _ in range(100):
        q = f.qnorm(f.qmul(q, f.from_axis_angle((0, 0, 1), 0.01)))
        est = fu.update((0, 0, 1.0), body(q, (0, 0, G)), None, 0.01)
    assert abs(f.euler(est)[2] - 1.0) < 1e-3


def test_tilt_corrects_gyro_drift():
    fu = f.Fusion(gain=1.0)
    q = pose(0.3, -0.2, 0.5)
    acc, mag = body(q, (0, 0, G)), body(q, FIELD_ENU)
    fu.update((0, 0, 0), acc, mag, 0.01)
    for _ in range(1000):
        est = fu.update((0.02, -0.02, 0.0), acc, mag, 0.01)  # biased gyro, still body
    r, p, _ = f.euler(est)
    assert abs(r - 0.3) < math.radians(2) and abs(p + 0.2) < math.radians(2)


def test_shaking_is_ignored():
    fu = f.Fusion(gain=5.0)
    q = pose(0, 0, 0)
    fu.update((0, 0, 0), (0, 0, G), body(q, FIELD_ENU), 0.01)
    est = fu.update((0, 0, 0), (15.0, 0, G), body(q, FIELD_ENU), 0.01)  # 1.8 g sideways kick
    assert angle_between(est, q) < 1e-9
