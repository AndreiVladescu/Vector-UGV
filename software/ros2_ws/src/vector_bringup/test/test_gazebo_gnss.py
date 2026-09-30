"""GNSS waypoints in Gazebo: the simulated receiver, the two EKFs and navsat_transform put the
robot on the map, and the follower walks it to a GPS point 5 m away."""
import math
import os
import subprocess
import sys
import time

import pytest
import rclpy
from std_msgs.msg import String
from rclpy.qos import DurabilityPolicy, QoSProfile

sys.path.insert(0, os.path.dirname(__file__))
from gz_helpers import gazebo_launch, GazeboTest  # noqa: E402

# world origin in the .sdf files
LAT0, LON0 = 44.4268, 26.1025


@pytest.mark.launch_test
def generate_test_description():
    return gazebo_launch(world='flat', nav='true')


class TestGnss(GazeboTest):

    def test_walks_to_a_gps_point(self):
        self.wait_for_sim()
        status = []
        self.node.create_subscription(String, '/mission/status', lambda m: status.append(m.data),
                                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.spin_for(8.0)  # the EKFs and navsat_transform settle on the first fixes
        start = self.odom.pose.pose.position
        # 4 m east, 3 m north of the start
        east, north = start.x + 4.0, start.y + 3.0
        lat = LAT0 + math.degrees(north / 6371000.0)
        lon = LON0 + math.degrees(east / (6371000.0 * math.cos(math.radians(LAT0))))
        send = subprocess.run(['ros2', 'run', 'vector_nav', 'waypoints.py', 'send', f'{lat:.8f},{lon:.8f}'],
                              capture_output=True, text=True, timeout=30)
        self.assertIn('sent 1', send.stdout, send.stdout + send.stderr)

        end = time.time() + 240
        while time.time() < end and (not status or status[-1] != 'done'):
            rclpy.spin_once(self.node, timeout_sec=0.1)
        p = self.odom.pose.pose.position
        err = math.hypot(p.x - east, p.y - north)
        walked = math.hypot(p.x - start.x, p.y - start.y)
        print(f'\n--- gnss waypoint: status {status[-1] if status else None}, '
              f'walked {walked:.2f} m, ended {err:.2f} m from the point ---')
        self.assertEqual(status[-1] if status else None, 'done')
        self.assertGreater(walked, 5.0 - 1.5 - 1.0)  # to within the arrive radius, less the same error
        self.assertLess(err, 1.5 + 1.0)  # the arrive radius, plus what GNSS and the EKF get wrong
