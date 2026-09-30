"""Obstacle geometry, without ROS: where the camera's objects meet the ground, which points
stick up out of it, and how much room is left ahead. Frames: x forward, y left, z up;
base_link sits ground_z above the ground (ground at z = -ground_z)."""
import math

import numpy as np


def quat_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def mask_contacts(mask, step=4):
    """(u, v) of the lowest object pixel in every step-th column: where objects touch the
    ground, or at least the nearest part of them the camera can see."""
    cols = np.arange(0, mask.shape[1], step)
    sub = mask[:, cols] > 0
    has = sub.any(axis=0)
    lowest = mask.shape[0] - 1 - np.argmax(sub[::-1], axis=0)
    return np.stack([cols[has], lowest[has]], axis=1) if has.any() else np.zeros((0, 2), int)


def pixel_rays(uv, w, h, hfov, vfov):
    """Unit rays in the camera frame for pixel coordinates uv (n, 2)."""
    tx = (uv[:, 0] + 0.5 - w / 2) / (w / 2) * math.tan(hfov / 2)
    ty = (uv[:, 1] + 0.5 - h / 2) / (h / 2) * math.tan(vfov / 2)
    rays = np.stack([np.ones_like(tx), -tx, -ty], axis=1)
    return rays / np.linalg.norm(rays, axis=1, keepdims=True)


def ground_hits(rays, cam_pos, cam_rot, ground_z, max_range):
    """Where camera-frame rays meet the ground, in base_link; rays at or above the horizon
    and hits further than max_range are dropped."""
    d = rays @ cam_rot.T
    down = d[:, 2] < -1e-3
    t = (-ground_z - cam_pos[2]) / np.where(down, d[:, 2], -1.0)
    p = cam_pos + d * t[:, None]
    keep = down & (np.hypot(p[:, 0], p[:, 1]) <= max_range)
    return p[keep]


def raised(points, ground_z, min_height=0.05, max_range=3.0):
    """Points (base_link) that stand at least min_height above the ground."""
    if not len(points):
        return points
    keep = (points[:, 2] > -ground_z + min_height) & (np.hypot(points[:, 0], points[:, 1]) <= max_range)
    return points[keep]


def sectors(points_xy, n=12, max_range=2.0):
    """Nearest obstacle in each of n directions around the robot (base_link x, y), metres,
    inf where there's nothing; sector 0 is straight ahead, then counter-clockwise (left)."""
    out = np.full(n, np.inf)
    if not len(points_xy):
        return out
    d = np.hypot(points_xy[:, 0], points_xy[:, 1])
    a = np.arctan2(points_xy[:, 1], points_xy[:, 0])
    idx = np.round(a / (2 * math.pi / n)).astype(int) % n
    near = d <= max_range
    np.minimum.at(out, idx[near], d[near])
    return out


def clearance(points_xy, half_width=0.25, look=2.0):
    """(free distance straight ahead, side with more room: +1 left, -1 right) from
    base_link (x, y) obstacle points."""
    if not len(points_xy):
        return math.inf, 1
    x, y = points_xy[:, 0], points_xy[:, 1]
    ahead = (x > 0) & (np.abs(y) < half_width) & (x < look)
    free = float(x[ahead].min()) if ahead.any() else math.inf
    near = (x > -0.2) & (x < look)
    left = np.count_nonzero(near & (y > 0))
    right = np.count_nonzero(near & (y < 0))
    return free, 1 if left <= right else -1
