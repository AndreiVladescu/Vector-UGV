"""The black box's memory and its reasons to save, no ROS here.

Messages go in as (time, topic, serialized bytes); the last window_s seconds stay, at most
max_hz per topic and max_bytes in all. trigger() starts a save: the buffer up to post_s
seconds later is what gets written, so the moments after the event are in it too."""
import collections
import math


class BlackBox:
    def __init__(self, window_s=120.0, post_s=10.0, max_hz=50.0, max_bytes=64 << 20):
        self.window_s, self.post_s, self.max_bytes = window_s, post_s, max_bytes
        self.min_dt = 1.0 / max_hz if max_hz > 0 else 0.0
        self.buf = collections.deque()
        self.bytes = 0
        self.last = {}
        self.pending = None  # (save at, [reasons])

    def add(self, t, topic, data):
        if t - self.last.get(topic, -math.inf) < self.min_dt:
            return
        self.last[topic] = t
        self.buf.append((t, topic, data))
        self.bytes += len(data)
        while self.buf and (t - self.buf[0][0] > self.window_s or self.bytes > self.max_bytes):
            self.bytes -= len(self.buf.popleft()[2])

    def trigger(self, t, reason):
        if self.pending is None:
            self.pending = (t + self.post_s, [reason])
        elif reason not in self.pending[1]:
            self.pending[1].append(reason)

    def due(self, t):
        """[reasons] and the messages to write once a save is due, else None."""
        if self.pending is None or t < self.pending[0]:
            return None
        reasons, self.pending = self.pending[1], None
        return reasons, list(self.buf)


class Triggers:
    """Turns what the robot reports into reasons to save, each once per event."""

    def __init__(self, tilt_deg=45.0):
        self.tilt = math.radians(tilt_deg)
        self.mode = None
        self.errors = set()
        self.fallen = False

    def on_mode(self, mode):
        was, self.mode = self.mode, mode
        if mode == 'halted' and was != 'halted':
            return 'leg fault'
        if mode == 'sentinel' and was == 'walk':  # straight down without lowering: e-stop
            return 'e-stop'
        return None

    def on_diag(self, name, level):
        """level 2 = ERROR; only the leg, power and link reports count."""
        if not name.startswith(('legs', 'power', 'links')):
            return None
        if level >= 2 and name not in self.errors:
            self.errors.add(name)
            return f'error: {name}'
        if level < 2:
            self.errors.discard(name)
        return None

    def on_tilt(self, roll, pitch):
        tilted = math.acos(max(-1.0, min(1.0, math.cos(roll) * math.cos(pitch)))) > self.tilt
        hit = tilted and not self.fallen
        self.fallen = tilted
        return 'fall' if hit else None
