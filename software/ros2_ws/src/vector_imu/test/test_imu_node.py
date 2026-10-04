"""imu_node.py on simulated chips turning at 0.5 rad/s: ENU orientation, the turn rate, the
compass, diagnostics and the compass calibration service."""
import math
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, MagneticField
from std_srvs.srv import Trigger

from vector_imu import fusion


@pytest.mark.launch_test
def generate_test_description():
    node = launch_ros.actions.Node(package='vector_imu', executable='imu_node.py', output='screen',
                                   parameters=[{'bus': 'sim', 'sim_turn': 0.5, 'bias_s': 0.0,
                                                'declination_deg': 0.0, 'cal_s': 1.0}])
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()])


class TestImuNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('imu_test')
        cls.imu, cls.mag, cls.diag = [], [], {}
        cls.node.create_subscription(Imu, '/imu', lambda m: cls.imu.append((time.monotonic(), m)), qos_profile_sensor_data)
        cls.node.create_subscription(MagneticField, '/mag', lambda m: cls.mag.append(m), qos_profile_sensor_data)

        def diag(m):
            for s in m.status:
                cls.diag[s.name] = s
        cls.node.create_subscription(DiagnosticArray, '/diagnostics', diag, 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin(self, seconds, until=None):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if until and until():
                return True
        return False

    def test_1_orientation_follows_the_turn(self):
        self.assertTrue(self.spin(15, lambda: len(self.imu) > 150))
        (t0, a), (t1, b) = self.imu[-120], self.imu[-1]
        ya = fusion.euler((a.orientation.w, a.orientation.x, a.orientation.y, a.orientation.z))
        yb = fusion.euler((b.orientation.w, b.orientation.x, b.orientation.y, b.orientation.z))
        self.assertLess(abs(yb[0]), 0.02)  # level
        self.assertLess(abs(yb[1]), 0.02)
        turned = math.remainder(yb[2] - ya[2], 2 * math.pi)
        # the sim turns 0.5 rad/s of its own time, about 0.5 rad/s of ours
        self.assertAlmostEqual(turned / (t1 - t0), 0.5, delta=0.1)
        self.assertAlmostEqual(b.angular_velocity.z, 0.5, delta=0.01)
        self.assertAlmostEqual(b.linear_acceleration.z, 9.80665, delta=0.01)
        self.assertEqual(b.header.frame_id, 'base_link')
        self.assertLess(b.orientation_covariance[8], 1.0)  # yaw from the compass

    def test_2_compass_and_diagnostics(self):
        self.assertTrue(self.spin(5, lambda: self.mag and 'imu: compass' in self.diag))
        f = self.mag[-1].magnetic_field
        self.assertAlmostEqual(math.sqrt(f.x ** 2 + f.y ** 2 + f.z ** 2), 48e-6, delta=1e-6)
        self.assertEqual(self.diag['imu: lsm6dsv16x'].level, b'\x00')
        self.assertIn('uT', self.diag['imu: compass'].message)

    def test_3_calibration(self):
        cli = self.node.create_client(Trigger, '/imu/calibrate_mag')
        self.assertTrue(cli.wait_for_service(timeout_sec=5))
        fut = cli.call_async(Trigger.Request())
        self.assertTrue(self.spin(5, fut.done))
        self.assertTrue(fut.result().success)
        # turning about z only: x and y span, z doesn't -> "not turned enough", the field stays as it was
        self.spin(2.0)
        f = self.mag[-1].magnetic_field
        self.assertAlmostEqual(math.sqrt(f.x ** 2 + f.y ** 2 + f.z ** 2), 48e-6, delta=1e-6)
