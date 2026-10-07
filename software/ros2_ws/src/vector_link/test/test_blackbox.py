import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from vector_link.blackbox import BlackBox, Triggers  # noqa: E402


def test_keeps_the_window_and_caps_the_rate():
    b = BlackBox(window_s=10, max_hz=10)
    for i in range(2000):  # 200 Hz for 10 s
        b.add(i / 200, '/joint_states', b'x' * 10)
    times = [m[0] for m in b.buf]
    assert 95 <= len(times) <= 101 and times[-1] - times[0] <= 10
    assert min(t2 - t1 for t1, t2 in zip(times, times[1:])) >= 0.1 - 1e-9


def test_byte_cap():
    b = BlackBox(window_s=1000, max_hz=0, max_bytes=1000)
    for i in range(50):
        b.add(i, '/a', b'x' * 100)
    assert b.bytes <= 1000 and len(b.buf) == 10


def test_saves_after_the_event_with_every_reason():
    b = BlackBox(post_s=10)
    for i in range(30):
        b.add(i, '/a', b'1')
        if i == 5:
            b.trigger(i, 'e-stop')
        if i == 8:
            b.trigger(i, 'fall')
        out = b.due(i)
        if out:
            reasons, msgs = out
            assert i == 15 and reasons == ['e-stop', 'fall'] and msgs[-1][0] == 15
            break
    else:
        assert False, 'never saved'
    assert b.due(40) is None


def test_triggers_once_per_event():
    t = Triggers()
    assert t.on_mode('walk') is None
    assert t.on_mode('sentinel') == 'e-stop'
    assert t.on_mode('powering') is None
    t.on_mode('walk')
    assert t.on_mode('stopping') is None and t.on_mode('spreading') is None and t.on_mode('lowering') is None
    assert t.on_mode('sentinel') is None  # sat down on purpose
    t.on_mode('walk')
    assert t.on_mode('halted') == 'leg fault' and t.on_mode('halted') is None
    assert t.on_diag('legs: L1', 2) == 'error: legs: L1'
    assert t.on_diag('legs: L1', 2) is None
    assert t.on_diag('imu: chip', 2) is None
    t.on_diag('legs: L1', 0)
    assert t.on_diag('legs: L1', 2) == 'error: legs: L1'
    assert t.on_tilt(0.1, 0.1) is None
    assert t.on_tilt(math.radians(60), 0) == 'fall' and t.on_tilt(math.radians(70), 0) is None
    t.on_tilt(0, 0)
    assert t.on_tilt(0, math.radians(-50)) == 'fall'
