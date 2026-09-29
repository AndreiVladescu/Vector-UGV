#!/usr/bin/env python3
"""GPS waypoints.

  waypoints.py                               the follower node (started by nav.launch.py)
  waypoints.py send 44.43510,26.10310 44.43520,26.10290 ...
  waypoints.py cancel

The follower takes a geographic_msgs/GeoPath on `mission`, turns each point into the map
frame with navsat_transform's fromLL service (only once there is a GNSS fix), and drives cmd_vel from odometry/global
until each point is within arrive_m. It only publishes cmd_vel while a mission runs, so
teleop works the rest of the time. It stops and waits when the position is stale (no
filtered odometry for 1 s, no GNSS fix for 5 s). With obstacles/clearance (obstacles.py) it
stops for something in the way and after 2 s steps sideways around it. Progress goes to
mission/status.
"""
import math
import sys
import time

import rclpy
from geographic_msgs.msg import GeoPath, GeoPoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from robot_localization.srv import FromLL
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import Trigger

from vector_nav.follower import Follower, Limits

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        lim = Limits(max_v=self.declare_parameter('max_v', 0.10).value,
                     max_w=self.declare_parameter('max_w', 0.4).value,
                     arrive=self.declare_parameter('arrive_m', 1.5).value)
        self.follower = Follower(lim)
        self.pose = None
        self.pose_t = self.fix_t = 0.0
        self.waiting = False
        self.pending = None
        self.status = ''
        self.cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'mission/status', LATCHED)
        self.from_ll = self.create_client(FromLL, 'fromLL')
        self.create_subscription(Odometry, 'odometry/global', self.on_odom, 10)
        self.create_subscription(NavSatFix, 'gnss/fix', self.on_fix, 10)
        self.create_subscription(GeoPath, 'mission', self.on_mission, 10)
        self.free, self.side, self.free_t = math.inf, 1, 0.0
        self.create_subscription(Float32MultiArray, 'obstacles/clearance', self.on_clearance, 10)
        self.create_service(Trigger, 'mission/cancel', self.on_cancel)
        self.create_timer(1.0 / self.declare_parameter('rate', 10.0).value, self.tick)
        self.set_status('idle')

    def set_status(self, text):
        if text != self.status:
            self.status = text
            self.status_pub.publish(String(data=text))
            self.get_logger().info(text)

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        self.pose_t = time.monotonic()

    def on_clearance(self, msg):
        if len(msg.data) >= 2:
            self.free, self.side, self.free_t = msg.data[0], 1 if msg.data[1] >= 0 else -1, time.monotonic()

    def on_fix(self, msg):
        if msg.status.status >= NavSatStatus.STATUS_FIX:
            self.fix_t = time.monotonic()

    def on_mission(self, msg):
        if not msg.poses:
            return self.stop('idle')
        if time.monotonic() - self.fix_t > 5.0:
            # before the first fix navsat_transform has no datum and fromLL answers nonsense
            return self.set_status('rejected: no GNSS fix yet')
        if not self.from_ll.wait_for_service(timeout_sec=1.0):
            return self.set_status('rejected: no fromLL service (navsat_transform not running)')
        self.set_status(f'converting {len(msg.poses)} waypoints')
        futures = []
        for p in msg.poses:
            req = FromLL.Request()
            req.ll_point.latitude = p.pose.position.latitude
            req.ll_point.longitude = p.pose.position.longitude
            futures.append(self.from_ll.call_async(req))
        self.pending = futures

    def on_cancel(self, req, res):
        self.stop('cancelled')
        res.success = True
        return res

    def stop(self, status):
        was = self.follower.active
        self.follower.cancel()
        self.pending = None
        if was:
            self.cmd.publish(Twist())
        self.set_status(status)

    def tick(self):
        pending = self.pending
        if pending and all(f.done() for f in pending):
            self.pending = None
            points = [(f.result().map_point.x, f.result().map_point.y) for f in pending if f.result()]
            if len(points) != len(pending):
                return self.set_status('rejected: fromLL failed')
            self.follower.start(points)
        if not self.follower.active:
            return
        now = time.monotonic()
        if self.pose is None or now - self.pose_t > 1.0 or now - self.fix_t > 5.0:
            if not self.waiting:
                self.cmd.publish(Twist())
                self.waiting = True
            return self.set_status('waiting for position')
        self.waiting = False
        # no obstacle data for a second (no camera, no ToF): walk on what GNSS says
        free = self.free if now - self.free_t < 1.0 else math.inf
        vx, vy, wz, status = self.follower.update(*self.pose, t=now, free=free, side=self.side)
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = vx, vy, wz
        self.cmd.publish(t)
        self.set_status(status)


def send(args):
    rclpy.init()
    node = rclpy.create_node('waypoints_send')
    if args and args[0] == 'cancel':
        client = node.create_client(Trigger, 'mission/cancel')
        client.wait_for_service(timeout_sec=5.0)
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        print('cancelled' if future.done() else 'no follower running')
        return
    path = GeoPath()
    path.header.stamp = node.get_clock().now().to_msg()
    for a in args:
        lat, lon = (float(v) for v in a.split(','))
        p = GeoPoseStamped()
        p.pose.position.latitude, p.pose.position.longitude = lat, lon
        path.poses.append(p)
    pub = node.create_publisher(GeoPath, 'mission', 10)
    end = time.time() + 5
    while pub.get_subscription_count() == 0 and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    pub.publish(path)
    rclpy.spin_once(node, timeout_sec=0.5)
    print(f'sent {len(path.poses)} waypoints' if pub.get_subscription_count() else 'no follower running')


def main():
    if len(sys.argv) > 1 and sys.argv[1] in ('send', 'cancel'):
        return send(sys.argv[2:] if sys.argv[1] == 'send' else ['cancel'])
    rclpy.init()
    rclpy.spin(WaypointFollower())


if __name__ == '__main__':
    main()
