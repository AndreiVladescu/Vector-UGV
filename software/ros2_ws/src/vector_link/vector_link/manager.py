"""Who drives the robot, and what to do when nobody can: no ROS here.

Sources, highest first: the ELRS radio (only while armed), an operator over Wi-Fi or LTE
(teleop / Foxglove, cmd_vel/teleop), a GPS mission (cmd_vel/nav). An operator counts as
connected while his heartbeat (or teleop commands) keep coming. With no radio link and no
operator for loss_s, the robot stops and sits down, and stays down until woken by hand."""
import math

SOURCES = ('elrs', 'teleop', 'nav')


class LinkManager:
    def __init__(self, loss_s=5.0, heartbeat_s=3.0, cmd_timeout=0.5, radio_timeout=1.0, missions_alone=False):
        self.loss_s, self.heartbeat_s = loss_s, heartbeat_s
        self.cmd_timeout, self.radio_timeout = cmd_timeout, radio_timeout
        self.missions_alone = missions_alone  # keep following a GPS mission with every link gone
        self.cmd = {s: (0.0, 0.0, 0.0) for s in SOURCES}
        self.cmd_t = {s: -math.inf for s in SOURCES}
        self.radio_t = self.operator_t = -math.inf
        self.lost_since = None
        self.sat_down = False
        self.source = None

    def on_cmd(self, source, vx, vy, wz, t):
        self.cmd[source] = (vx, vy, wz)
        self.cmd_t[source] = t
        if source == 'teleop':
            self.operator_t = t

    def on_radio(self, lq, t):
        if lq > 0:
            self.radio_t = t

    def on_heartbeat(self, t):
        self.operator_t = t

    def links(self, t):
        return {'radio': t - self.radio_t < self.radio_timeout, 'operator': t - self.operator_t < self.heartbeat_s}

    def step(self, t):
        """(cmd to publish or None, action or None). An action is 'sentinel' when the links
        have been gone for loss_s."""
        up = any(self.links(t).values())
        action = None
        if up:
            self.lost_since, self.sat_down = None, False
        else:
            if self.lost_since is None:
                self.lost_since = t
            if t - self.lost_since >= self.loss_s and not self.sat_down:
                self.sat_down, action = True, 'sentinel'

        fresh = [s for s in SOURCES if t - self.cmd_t[s] < self.cmd_timeout]
        if 'nav' in fresh and not up and not self.missions_alone:
            fresh.remove('nav')
        was, self.source = self.source, fresh[0] if fresh else None
        if self.source:
            return self.cmd[self.source], action
        return ((0.0, 0.0, 0.0) if was else None), action
