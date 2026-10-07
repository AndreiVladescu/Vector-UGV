"""Who drives the robot, and what to do when nobody can: no ROS here.

Sources, highest first: the ELRS radio (only while armed), an operator over Wi-Fi or LTE
(teleop / Foxglove, cmd_vel/teleop), a GPS mission (cmd_vel/nav). An operator counts as
connected while their heartbeat (or teleop commands) keep coming. With no radio link and no
operator for loss_s, the robot stops and sits down, and stays down until woken by hand. With
return_home it walks back to its home point first, when it has one and a GNSS fix, and sits
down there; a link coming back on the way cancels the walk and hands control back."""
import math

SOURCES = ('elrs', 'teleop', 'nav')


class LinkManager:
    def __init__(self, loss_s=5.0, heartbeat_s=3.0, cmd_timeout=0.5, radio_timeout=1.0, missions_alone=False,
                 return_home=False, stuck_s=30.0, return_max_s=900.0):
        self.loss_s, self.heartbeat_s = loss_s, heartbeat_s
        self.cmd_timeout, self.radio_timeout = cmd_timeout, radio_timeout
        self.missions_alone = missions_alone  # keep following a GPS mission with every link gone
        self.cmd = {s: (0.0, 0.0, 0.0) for s in SOURCES}
        self.cmd_t = {s: -math.inf for s in SOURCES}
        self.radio_t = self.operator_t = -math.inf
        self.lost_since = None
        self.sat_down = False
        self.source = None
        self.return_home, self.stuck_s, self.return_max_s = return_home, stuck_s, return_max_s
        self.returning = False
        self.return_t = self.status_t = 0.0
        self.status = ''

    def on_mission_status(self, status, t):
        if status != self.status:
            self.status, self.status_t = status, t

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

    def step(self, t, home_ready=False):
        """(cmd to publish or None, action or None). Actions: 'sentinel' when the links have
        been gone for loss_s; with return_home and home_ready (a home point and a GNSS fix)
        'home' instead (send the mission home), then 'sentinel' once it's there or can't get
        there; 'cancel' when a link comes back on the way."""
        up = any(self.links(t).values())
        action = None
        if up:
            if self.returning:
                self.returning, action = False, 'cancel'
            self.lost_since, self.sat_down = None, False
        else:
            if self.lost_since is None:
                self.lost_since = t
            if t - self.lost_since >= self.loss_s and not self.sat_down and not self.returning:
                if self.return_home and home_ready:
                    self.returning, self.return_t, action = True, t, 'home'
                    self.status, self.status_t = 'sent home', t
                else:
                    self.sat_down, action = True, 'sentinel'
            elif self.returning and self.arrived_or_stuck(t):
                self.returning, self.sat_down, action = False, True, 'sentinel'

        fresh = [s for s in SOURCES if t - self.cmd_t[s] < self.cmd_timeout]
        if 'nav' in fresh and not up and not self.missions_alone and not self.returning:
            fresh.remove('nav')
        was, self.source = self.source, fresh[0] if fresh else None
        if self.source:
            return self.cmd[self.source], action
        return ((0.0, 0.0, 0.0) if was else None), action

    def arrived_or_stuck(self, t):
        s = self.status
        if s in ('done', 'idle', 'cancelled') or s.startswith('rejected'):
            return True
        if s.startswith('waiting') and t - self.status_t > self.stuck_s:  # no position
            return True
        return t - self.return_t > self.return_max_s
