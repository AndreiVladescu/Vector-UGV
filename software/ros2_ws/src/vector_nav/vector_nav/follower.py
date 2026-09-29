"""Steering toward waypoints in the map frame, without ROS: the geometry and the command law."""
import math
from dataclasses import dataclass


@dataclass
class Limits:
    max_v: float = 0.10         # m/s, about what the tripod gait does at full stride
    max_w: float = 0.4          # rad/s
    turn_in_place: float = 0.6  # rad of heading error beyond which it turns on the spot
    k_w: float = 1.0            # 1/s
    k_v: float = 0.2            # 1/s, slows down over the last half metre
    arrive: float = 1.5         # m, GNSS is good to about that
    stop: float = 0.6           # m of free space ahead below which it stops
    sidestep_after: float = 2.0  # s blocked before it steps sideways (a hexapod can)
    sidestep_v: float = 0.05    # m/s


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def clamp(v, lim):
    return max(-lim, min(lim, v))


def steer(x, y, yaw, tx, ty, lim):
    """(vx, wz, distance) toward (tx, ty) from pose (x, y, yaw)."""
    dx, dy = tx - x, ty - y
    dist = math.hypot(dx, dy)
    err = wrap(math.atan2(dy, dx) - yaw)
    wz = clamp(lim.k_w * err, lim.max_w)
    vx = 0.0 if abs(err) > lim.turn_in_place else min(lim.max_v, lim.k_v * dist + 0.02) * math.cos(err)
    return vx, wz, dist


def ll_to_local(lat, lon, lat0, lon0):
    """(east, north) in metres from (lat0, lon0); equirectangular, fine over a few km."""
    r = 6371000.0
    return (math.radians(lon - lon0) * r * math.cos(math.radians(lat0)), math.radians(lat - lat0) * r)


class Follower:
    """Walks a list of map-frame points in order."""

    def __init__(self, lim=None):
        self.lim = lim or Limits()
        self.points = []
        self.index = 0
        self.blocked_since = None

    def start(self, points):
        self.points = list(points)
        self.index = 0

    def cancel(self):
        self.points = []
        self.index = 0

    @property
    def active(self):
        return self.index < len(self.points)

    def update(self, x, y, yaw, t=0.0, free=math.inf, side=1):
        """(vx, vy, wz, status); zeros once the last point is reached. free is the room
        straight ahead in metres, side the side with more room (+1 left, -1 right): with
        something in the way it stops, and after a while steps sideways until it's clear."""
        while self.active:
            tx, ty = self.points[self.index]
            vx, wz, dist = steer(x, y, yaw, tx, ty, self.lim)
            if dist <= self.lim.arrive:
                self.index += 1
                continue
            where = f'waypoint {self.index + 1}/{len(self.points)}, {dist:.1f} m'
            if vx > 0 and free < self.lim.stop:
                if self.blocked_since is None:
                    self.blocked_since = t
                if t - self.blocked_since < self.lim.sidestep_after:
                    return 0.0, 0.0, 0.0, f'{where}, blocked at {free:.1f} m'
                return 0.0, side * self.lim.sidestep_v, 0.0, f'{where}, stepping {"left" if side > 0 else "right"}'
            self.blocked_since = None
            return vx, 0.0, wz, where
        return 0.0, 0.0, 0.0, 'done' if self.points else 'idle'
