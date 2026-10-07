import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from vector_link.manager import LinkManager  # noqa: E402


def test_radio_beats_teleop_beats_nav():
    m = LinkManager()
    m.on_heartbeat(0)
    m.on_cmd('nav', 0.1, 0, 0, 0)
    assert m.step(0.1)[0] == (0.1, 0, 0) and m.source == 'nav'
    m.on_cmd('teleop', 0.05, 0, 0.2, 0.1)
    assert m.step(0.2)[0] == (0.05, 0, 0.2) and m.source == 'teleop'
    m.on_radio(90, 0.2)
    m.on_cmd('elrs', 0, 0.04, 0, 0.2)
    assert m.step(0.3)[0] == (0, 0.04, 0) and m.source == 'elrs'


def test_stops_once_when_the_source_goes_quiet():
    m = LinkManager()
    m.on_heartbeat(0)
    m.on_cmd('teleop', 0.1, 0, 0, 0)
    assert m.step(0.1)[0] == (0.1, 0, 0)
    assert m.step(1.0)[0] == (0, 0, 0)   # stale: one zero
    assert m.step(1.1)[0] is None         # then quiet, so another node can drive


def test_no_links_stops_the_mission_then_sits_down():
    m = LinkManager(loss_s=5)
    m.on_heartbeat(0)
    for t in range(1, 30):
        m.on_cmd('nav', 0.1, 0, 0, t / 10)
        assert m.step(t / 10)[0] == (0.1, 0, 0)
    cmd, action = m.step(3.0)          # heartbeat 3 s old: operator gone, nav blocked
    assert cmd == (0, 0, 0) and action is None
    assert m.step(7.9)[1] is None
    assert m.step(8.1)[1] == 'sentinel'
    assert m.step(9.0)[1] is None        # once
    m.on_heartbeat(10)
    assert m.step(10.1)[1] is None and not m.sat_down


def test_missions_can_run_alone_if_asked():
    m = LinkManager(missions_alone=True)
    m.on_cmd('nav', 0.1, 0, 0, 100)
    assert m.step(100.1)[0] == (0.1, 0, 0)


def test_radio_link_keeps_it_up_without_an_operator():
    m = LinkManager(loss_s=1)
    for t in range(20):
        m.on_radio(50, t / 2)
        assert m.step(t / 2)[1] is None


def lost_with_home(m, t0=0):
    m.on_heartbeat(t0)
    assert m.step(t0 + 1, home_ready=True)[1] is None
    assert m.step(t0 + 3.1, home_ready=True)[1] is None  # heartbeat stale: the loss starts here
    return t0 + 3.1 + m.loss_s


def test_goes_home_instead_of_sitting_when_it_can():
    m = LinkManager(loss_s=5, return_home=True)
    t = lost_with_home(m)
    assert m.step(t - 0.2, home_ready=True)[1] is None
    assert m.step(t + 0.1, home_ready=True)[1] == 'home' and m.returning
    m.on_cmd('nav', 0.1, 0, 0, t + 0.5)  # the follower drives with no link
    m.on_mission_status('heading to 1/1, 40 m', t + 0.5)
    assert m.step(t + 0.6, home_ready=True) == ((0.1, 0, 0), None)
    m.on_mission_status('done', t + 60)
    assert m.step(t + 60.1, home_ready=True)[1] == 'sentinel' and m.sat_down and not m.returning
    assert m.step(t + 61, home_ready=True)[1] is None


def test_sits_down_without_home_or_fix():
    m = LinkManager(loss_s=5, return_home=True)
    t = lost_with_home(m)
    assert m.step(t + 0.1, home_ready=False)[1] == 'sentinel'


def test_a_link_coming_back_cancels_the_walk_home():
    m = LinkManager(loss_s=5, return_home=True)
    t = lost_with_home(m)
    assert m.step(t + 0.1, home_ready=True)[1] == 'home'
    m.on_heartbeat(t + 5)
    assert m.step(t + 5.1, home_ready=True)[1] == 'cancel' and not m.returning and not m.sat_down


def test_sits_down_when_the_way_home_is_lost():
    m = LinkManager(loss_s=5, return_home=True, stuck_s=30)
    t = lost_with_home(m)
    assert m.step(t + 0.1, home_ready=True)[1] == 'home'
    m.on_mission_status('waiting for position', t + 1)
    assert m.step(t + 30, home_ready=True)[1] is None
    assert m.step(t + 31.5, home_ready=True)[1] == 'sentinel'
    m2 = LinkManager(loss_s=5, return_home=True)
    t = lost_with_home(m2)
    m2.step(t + 0.1, home_ready=True)
    m2.on_mission_status('rejected: no GNSS fix yet', t + 1)
    assert m2.step(t + 1.1, home_ready=True)[1] == 'sentinel'
