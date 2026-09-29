import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from vector_nav.follower import Follower, Limits, ll_to_local, steer, wrap  # noqa: E402


def test_wrap():
    assert abs(wrap(3 * math.pi) - math.pi) < 1e-9 or abs(wrap(3 * math.pi) + math.pi) < 1e-9
    assert abs(wrap(-0.5) + 0.5) < 1e-12


def test_straight_ahead_full_speed():
    vx, wz, dist = steer(0, 0, 0, 10, 0, Limits())
    assert abs(vx - 0.10) < 1e-9 and abs(wz) < 1e-9 and abs(dist - 10) < 1e-9


def test_behind_turns_in_place():
    vx, wz, _ = steer(0, 0, 0, -5, 0.1, Limits())
    assert vx == 0 and abs(wz) == Limits().max_w
    vx, wz, _ = steer(0, 0, 0, 0.1, -5, Limits())  # to the right: clockwise
    assert vx == 0 and wz < 0


def test_slows_down_near_the_point():
    vx, _, _ = steer(0, 0, 0, 0.3, 0, Limits())
    assert 0.02 < vx < 0.10


def test_follower_goes_through_the_points():
    f = Follower(Limits(arrive=1.0))
    f.start([(5, 0), (5, 5)])
    x = y = yaw = 0.0
    for _ in range(3000):  # 0.1 s steps
        vx, _, wz, status = f.update(x, y, yaw)
        if not f.active:
            break
        yaw += wz * 0.1
        x += vx * math.cos(yaw) * 0.1
        y += vx * math.sin(yaw) * 0.1
    assert status == 'done'
    assert math.hypot(x - 5, y - 5) <= 1.0


def test_ll_to_local():
    e, n = ll_to_local(44.4268 + 1e-4, 26.1025 + 1e-4, 44.4268, 26.1025)
    assert abs(n - 11.12) < 0.05 and abs(e - 7.94) < 0.05


def test_stops_then_steps_around():
    f = Follower(Limits())
    f.start([(10, 0)])
    vx, vy, wz, status = f.update(0, 0, 0, t=0.0, free=0.4, side=-1)
    assert vx == vy == 0 and 'blocked' in status
    vx, vy, _, status = f.update(0, 0, 0, t=2.5, free=0.4, side=-1)
    assert vx == 0 and vy < 0 and 'right' in status
    vx, vy, _, _ = f.update(0, 0, 0, t=3.0, free=2.0, side=-1)
    assert vx > 0 and vy == 0


def test_turning_in_place_ignores_what_is_ahead():
    f = Follower(Limits())
    f.start([(-10, 0)])
    vx, vy, wz, _ = f.update(0, 0, 0, t=0.0, free=0.2)
    assert vx == 0 and vy == 0 and wz != 0
