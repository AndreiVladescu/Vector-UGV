import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from vector_nav.obstacles import (clearance, ground_hits, mask_contacts, pixel_rays,  # noqa: E402
                                  quat_matrix, raised, sectors)

HFOV, VFOV = math.radians(53.5), math.radians(41.4)


def test_mask_contacts_takes_the_lowest_pixel():
    m = np.zeros((240, 320), np.uint8)
    m[100:180, 40:60] = 3
    c = mask_contacts(m, step=4)
    assert len(c) == 5 and (c[:, 1] == 179).all() and c[0, 0] == 40


def test_ground_hit_distance():
    # camera 0.1 m above base_link, base_link 0.08 m above the ground, looking level: the bottom
    # row sees the ground where the ray drops the 0.18 m
    rays = pixel_rays(np.array([[159, 239]]), 320, 240, HFOV, VFOV)
    p = ground_hits(rays, np.array([0.1, 0, 0.1]), np.eye(3), 0.08, 10)
    slope = (239.5 - 120) / 120 * math.tan(VFOV / 2)
    assert len(p) == 1
    assert abs(p[0, 0] - (0.1 + 0.18 / slope)) < 1e-6
    assert abs(p[0, 2] + 0.08) < 1e-9


def test_horizon_and_above_miss_the_ground():
    rays = pixel_rays(np.array([[160, 10], [160, 119]]), 320, 240, HFOV, VFOV)
    assert len(ground_hits(rays, np.array([0, 0, 0.1]), np.eye(3), 0.08, 10)) == 0


def test_pitched_down_camera_sees_closer():
    rays = pixel_rays(np.array([[160, 180]]), 320, 240, HFOV, VFOV)
    level = ground_hits(rays, np.zeros(3), np.eye(3), 0.1, 20)[0, 0]
    pitch = 0.2  # rotation about y, positive tips the view down
    r = quat_matrix(0, math.sin(pitch / 2), 0, math.cos(pitch / 2))
    down = ground_hits(rays, np.zeros(3), r, 0.1, 20)[0, 0]
    assert down < level


def test_raised_drops_the_ground():
    pts = np.array([[1.0, 0, -0.078], [1.0, 0, 0.0], [5.0, 0, 0.2]])
    assert len(raised(pts, 0.08)) == 1


def test_clearance_and_side():
    free, side = clearance(np.array([[1.2, 0.1], [0.8, 0.6], [1.0, 0.3], [0.9, 0.35]]))
    assert abs(free - 1.2) < 1e-9 and side == -1  # the left is crowded: go right
    assert clearance(np.zeros((0, 2)))[0] == math.inf


def test_sectors():
    s = sectors(np.array([[0.5, 0.0], [0.9, 0.05], [0.0, 1.2], [-0.4, -0.01], [3.0, 0.0]]))
    assert len(s) == 12
    assert abs(s[0] - 0.5) < 1e-9                         # ahead, the nearer of two
    assert abs(s[3] - 1.2) < 1e-9 and abs(s[6] - 0.4) < 0.01  # left, behind
    assert np.isinf(s[9])                                  # right: nothing; 3 m ahead is too far
