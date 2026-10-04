"""Orientation from gyro, accelerometer and (when there is one) compass: the gyro integrates,
and each step pulls the estimate a little toward the orientation the accelerometer (up)
and the compass (north) give directly. World frame ENU, quaternions (w, x, y, z) rotating
body vectors into the world."""
import math


def qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw)


def qnorm(q):
    n = math.sqrt(sum(c * c for c in q))
    return tuple(c / n for c in q)


def qconj(q):
    return (q[0], -q[1], -q[2], -q[3])


def rotate(q, v):
    """body vector -> world"""
    return qmul(qmul(q, (0.0, *v)), qconj(q))[1:]


def from_axis_angle(axis, angle):
    s = math.sin(angle / 2)
    return (math.cos(angle / 2), axis[0] * s, axis[1] * s, axis[2] * s)


def from_matrix(m):
    """rows of m: the world axes in body coordinates (a body -> world rotation matrix)"""
    (m00, m01, m02), (m10, m11, m12), (m20, m21, m22) = m
    tr = m00 + m11 + m22
    if tr > 0:
        s = math.sqrt(tr + 1) * 2
        q = (s / 4, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s)
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1 + m00 - m11 - m22) * 2
        q = ((m21 - m12) / s, s / 4, (m01 + m10) / s, (m02 + m20) / s)
    elif m11 > m22:
        s = math.sqrt(1 + m11 - m00 - m22) * 2
        q = ((m02 - m20) / s, (m01 + m10) / s, s / 4, (m12 + m21) / s)
    else:
        s = math.sqrt(1 + m22 - m00 - m11) * 2
        q = ((m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, s / 4)
    return qnorm(q)


def slerp(a, b, t):
    d = sum(x * y for x, y in zip(a, b))
    if d < 0:
        b, d = tuple(-c for c in b), -d
    if d > 0.9995:
        return qnorm(tuple(x + t * (y - x) for x, y in zip(a, b)))
    th = math.acos(d)
    s = math.sin(th)
    wa, wb = math.sin((1 - t) * th) / s, math.sin(t * th) / s
    return tuple(wa * x + wb * y for x, y in zip(a, b))


def _unit(v):
    n = math.sqrt(sum(c * c for c in v))
    return tuple(c / n for c in v) if n > 1e-12 else None


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def euler(q):
    """-> roll, pitch, yaw (ENU: yaw 0 = east, counter-clockwise)"""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


class Fusion:
    """gain: fraction of the way to the measured orientation per second; the accelerometer
    only counts when it reads close to 1 g (not while the body is being shaken about)."""

    def __init__(self, gain=1.0, declination=0.0):
        self.q = None
        self.gain = gain
        self.declination = declination  # rad, east positive: true north from magnetic north

    def measured(self, accel, mag):
        """Orientation from up and north alone; without a compass the yaw stays the estimate's."""
        up = _unit(accel)
        if up is None:
            return None
        east = None
        if mag is not None:
            east = _unit(_cross(mag, up))
            if east is not None and self.declination:
                # magnetic north lies the declination east of true north: turn back counter-clockwise
                east = rotate(from_axis_angle(up, self.declination), east)
        if east is None:
            if self.q is None:
                return None
            e = rotate(qconj(self.q), (1.0, 0.0, 0.0))  # the current estimate of east, in the body
            east = _unit(tuple(c - _dot(e, up) * u for c, u in zip(e, up)))
            if east is None:
                return None
        north = _cross(up, east)
        return from_matrix((east, north, up))

    def update(self, gyro, accel, mag, dt):
        g = math.sqrt(_dot(accel, accel))
        trust = 0.85 * 9.80665 < g < 1.15 * 9.80665
        if self.q is None:
            if trust:
                self.q = self.measured(accel, mag) or self.q
            if self.q is None:
                self.q = (1.0, 0.0, 0.0, 0.0)
            return self.q
        w = _dot(gyro, gyro) ** 0.5
        if w > 1e-9:
            self.q = qnorm(qmul(self.q, from_axis_angle(tuple(c / w for c in gyro), w * dt)))
        if trust:
            m = self.measured(accel, mag)
            if m is not None:
                self.q = qnorm(slerp(self.q, m, min(1.0, self.gain * dt)))
        return self.q
