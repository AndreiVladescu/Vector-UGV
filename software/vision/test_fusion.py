"""python3 -m pytest software/vision/test_fusion.py (or run it directly)"""
import math

import numpy as np

from fusion import box_distance, box_point, box_zones

HFOV, VFOV, TOF = math.radians(53.5), math.radians(41.4), math.radians(45)


def test_centre_box_hits_the_middle_zones():
    rows, cols = box_zones((600, 440, 680, 520), 1280, 960, HFOV, VFOV, TOF)
    assert rows == [3, 4] and cols == [3, 4]


def test_box_outside_the_tof_view():
    assert box_zones((0, 400, 40, 500), 1280, 960, HFOV, VFOV, TOF)[1] == []


def test_nearest_surface_wins_over_background():
    depth = np.full((8, 8), 3.0, np.float32)
    depth[3:5, 3:5] = 1.2
    depth[0, 0] = np.nan
    d = box_distance((500, 380, 780, 580), 1280, 960, depth, HFOV, VFOV, TOF)
    assert abs(d - 1.2) < 0.01


def test_no_valid_zone():
    depth = np.full((8, 8), np.nan, np.float32)
    assert box_distance((600, 440, 680, 520), 1280, 960, depth, HFOV, VFOV, TOF) is None


def test_point_left_of_centre_is_positive_y():
    x, y, z = box_point((100, 460, 140, 500), 1280, 960, HFOV, VFOV, 2.0)
    assert y > 0 and abs(z) < 0.05 and abs(math.sqrt(x * x + y * y + z * z) - 2.0) < 1e-6


if __name__ == '__main__':
    for name, f in list(globals().items()):
        if name.startswith('test_'):
            f()
    print('ok')
