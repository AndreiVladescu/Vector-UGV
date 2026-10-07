#!/usr/bin/env python3
"""Local obstacles around the robot, from what the sensors see:
  /yolo/mask          the lowest object pixel per column, projected onto the ground (flat
                      ground assumed): where people, bikes, cars... stand
  /nose/tof/points    VL53L8CX zones that stand more than min_height above the ground
  /legs/<leg>/tof     the six leg rays, the same test
  /scan               the LDS01RR, one plane above the body; returns within lidar_self_m of
                      the body centre are its own legs and are left out
Points are kept keep_s seconds in odom, so they stay put while the robot walks, and give
  obstacles/grid       nav_msgs/OccupancyGrid around the robot (odom frame), for Foxglove
  obstacles/clearance  std_msgs/Float32MultiArray [free metres straight ahead (inf = clear),
                       side with more room: +1 left, -1 right], used by the waypoint follower
  obstacles/sectors    std_msgs/Float32MultiArray, the nearest obstacle in 12 directions (0 =
                       ahead, counter-clockwise, inf = nothing within 2 m), for the operator page
"""
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, LaserScan, PointCloud2, Range
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray
from tf2_ros import Buffer, TransformException, TransformListener

from vector_nav.obstacles import clearance, ground_hits, mask_contacts, pixel_rays, quat_matrix, raised, sectors

LEGS = ('L1', 'L2', 'L3', 'R1', 'R2', 'R3')


def pose(tf):
    t, q = tf.transform.translation, tf.transform.rotation
    return np.array([t.x, t.y, t.z]), quat_matrix(q.x, q.y, q.z, q.w)


class Obstacles(Node):
    def __init__(self):
        super().__init__('obstacles')
        p = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731
        self.camera_frame = p('camera_frame', 'nose_tof')
        self.fov = (math.radians(p('cam_hfov_deg', 53.5)), math.radians(p('cam_vfov_deg', 41.4)))
        self.ground_z = p('ground_z', 0.10)       # base_link above the ground, legs.yaml stand height
        self.lidar_self = p('lidar_self_m', 0.30)
        self.min_height = p('min_height', 0.05)   # lower than this is ground (or a step it can take)
        self.max_range = p('max_range', 3.0)
        self.keep_s = p('keep_s', 3.0)
        self.half_width = p('half_width', 0.25)
        self.size, self.res = p('grid_size', 4.0), p('resolution', 0.05)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.points = []  # (time, (n, 2) odom xy)
        self.create_subscription(Image, '/yolo/mask', self.on_mask, 2)
        self.create_subscription(PointCloud2, '/nose/tof/points', self.on_cloud, qos_profile_sensor_data)
        for leg in LEGS:
            self.create_subscription(Range, f'/legs/{leg}/tof', self.on_range, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.grid_pub = self.create_publisher(OccupancyGrid, 'obstacles/grid', 2)
        self.clear_pub = self.create_publisher(Float32MultiArray, 'obstacles/clearance', 10)
        self.sector_pub = self.create_publisher(Float32MultiArray, 'obstacles/sectors', 10)
        self.create_timer(1.0 / p('rate', 5.0), self.tick)

    def lookup(self, target, source):
        try:
            return pose(self.tf.lookup_transform(target, source, Time()))
        except TransformException:
            return None

    def store(self, base_pts):
        """base_link points -> odom xy, kept for keep_s."""
        odom = self.lookup('odom', 'base_link')
        if odom is None or not len(base_pts):
            return
        pos, rot = odom
        self.points.append((time.monotonic(), (base_pts @ rot.T + pos)[:, :2]))

    def on_mask(self, msg):
        cam = self.lookup('base_link', self.camera_frame)
        if cam is None or msg.encoding != 'mono8':
            return
        mask = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)[:, :msg.width]
        uv = mask_contacts(mask)
        if len(uv):
            rays = pixel_rays(uv, msg.width, msg.height, *self.fov)
            self.store(ground_hits(rays, cam[0], cam[1], self.ground_z, self.max_range))

    def to_base(self, frame, pts):
        tf = self.lookup('base_link', frame)
        if tf is None:
            return None
        return raised(pts @ tf[1].T + tf[0], self.ground_z, self.min_height, self.max_range)

    def on_cloud(self, msg):
        pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        base = self.to_base(msg.header.frame_id, np.asarray(pts, float).reshape(-1, 3))
        if base is not None:
            self.store(base)

    def on_range(self, msg):
        if not math.isfinite(msg.range) or msg.range > msg.max_range:
            return
        base = self.to_base(msg.header.frame_id, np.array([[msg.range, 0.0, 0.0]]))
        if base is not None:
            self.store(base)

    def on_scan(self, msg):
        r = np.asarray(msg.ranges, float)
        a = msg.angle_min + msg.angle_increment * np.arange(len(r))
        ok = np.isfinite(r) & (r >= max(msg.range_min, 0.01)) & (r <= min(msg.range_max, self.max_range))
        pts = np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok]), np.zeros(int(ok.sum()))], axis=1)
        base = self.to_base(msg.header.frame_id, pts)
        if base is not None and len(base):
            self.store(base[np.hypot(base[:, 0], base[:, 1]) > self.lidar_self])

    def tick(self):
        now = time.monotonic()
        self.points = [(t, p) for t, p in self.points if now - t < self.keep_s]
        odom = self.lookup('odom', 'base_link')
        if odom is None:
            return
        pos, rot = odom
        pts = np.concatenate([p for _, p in self.points]) if self.points else np.zeros((0, 2))
        # odom xy -> base_link xy (yaw only)
        yaw = math.atan2(rot[1, 0], rot[0, 0])
        c, s = math.cos(yaw), math.sin(yaw)
        rel = pts - pos[:2]
        base = np.stack([c * rel[:, 0] + s * rel[:, 1], -s * rel[:, 0] + c * rel[:, 1]], axis=1)
        free, side = clearance(base, self.half_width)
        self.clear_pub.publish(Float32MultiArray(data=[float(free), float(side)]))
        self.sector_pub.publish(Float32MultiArray(data=[float(v) for v in sectors(base)]))

        n = int(self.size / self.res)
        g = OccupancyGrid()
        g.header.stamp = self.get_clock().now().to_msg()
        g.header.frame_id = 'odom'
        g.info.resolution = self.res
        g.info.width = g.info.height = n
        g.info.origin.position.x = pos[0] - self.size / 2
        g.info.origin.position.y = pos[1] - self.size / 2
        g.info.origin.orientation.w = 1.0
        cells = np.zeros(n * n, np.int8)
        ij = np.floor((pts - (pos[:2] - self.size / 2)) / self.res).astype(int)
        ok = (ij >= 0).all(axis=1) & (ij < n).all(axis=1)
        cells[ij[ok, 1] * n + ij[ok, 0]] = 100
        g.data = cells.tolist()
        self.grid_pub.publish(g)


def main():
    rclpy.init()
    rclpy.spin(Obstacles())


if __name__ == '__main__':
    main()
