"""Distance for camera boxes from the nose ToF. The camera and the VL53L8CX sit side by side
on the nose board with parallel axes (a few cm apart, ignored), so the direction of a box
picks the ToF zones behind it."""
import math

import numpy as np


def box_zones(box, img_w, img_h, cam_hfov, cam_vfov, tof_fov, side=8):
    """Rows and columns of the ToF grid (row 0 top, col 0 left) that a pixel box covers."""
    x1, y1, x2, y2 = box

    def angle(p, size, fov):  # pinhole: pixel -> angle from the axis, right / down positive
        return math.atan((p - size / 2) / (size / 2) * math.tan(fov / 2))

    ax = sorted((angle(x1, img_w, cam_hfov), angle(x2, img_w, cam_hfov)))
    ay = sorted((angle(y1, img_h, cam_vfov), angle(y2, img_h, cam_vfov)))
    step = tof_fov / side
    edges = [-tof_fov / 2 + i * step for i in range(side + 1)]
    cols = [i for i in range(side) if edges[i + 1] > ax[0] and edges[i] < ax[1]]
    rows = [i for i in range(side) if edges[i + 1] > ay[0] and edges[i] < ay[1]]
    return rows, cols


def box_distance(box, img_w, img_h, depth, cam_hfov, cam_vfov, tof_fov, pct=20):
    """Metres to what's in the box, or None: a low percentile of the valid zones it covers,
    so the object wins over the background showing around it."""
    rows, cols = box_zones(box, img_w, img_h, cam_hfov, cam_vfov, tof_fov, depth.shape[0])
    if not rows or not cols:
        return None  # outside the ToF's view
    vals = depth[np.ix_(rows, cols)]
    vals = vals[np.isfinite(vals)]
    return float(np.percentile(vals, pct)) if vals.size else None


def box_point(box, img_w, img_h, cam_hfov, cam_vfov, dist):
    """(x forward, y left, z up) of the box centre at dist along its ray."""
    cx, cy = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
    tx = (cx - img_w / 2) / (img_w / 2) * math.tan(cam_hfov / 2)
    ty = (cy - img_h / 2) / (img_h / 2) * math.tan(cam_vfov / 2)
    n = math.sqrt(1 + tx * tx + ty * ty)
    return dist / n, -tx * dist / n, -ty * dist / n
