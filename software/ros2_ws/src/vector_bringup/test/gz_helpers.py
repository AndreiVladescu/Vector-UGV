"""Shared bits for the Gazebo launch tests."""
import math
import os
import subprocess
import time
import unittest

import launch
import launch_testing.actions
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

KGCM = 100 / 9.81  # N·m -> kg·cm

# Headless gz server as started by sim_gazebo.launch.py. launch_testing signals only the
# `gz sim` wrapper, not its server child, so the server is killed explicitly after each test.
GZ_HEADLESS = 'gz sim -r -v 1 -s'


def gazebo_launch(**args):
    args.setdefault('gui', 'false')
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('vector_bringup'), 'launch', 'sim_gazebo.launch.py')),
        launch_arguments=args.items())
    return launch.LaunchDescription([
        # own gz partition, so a leftover or parallel sim can't feed this test
        SetEnvironmentVariable('GZ_PARTITION', f'vector_test_{os.getpid()}'),
        sim,
        launch_testing.actions.ReadyToTest()])


def roll_pitch(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    return roll, pitch


class GazeboTest(unittest.TestCase):
    """Subscribes to joints and ground truth, records torques and tilt while recording."""

    recording = False

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
        subprocess.run(['pkill', '-f', GZ_HEADLESS], check=False)

    def setUp(self):
        self.node = rclpy.create_node('gz_test', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.joints = None
        self.odom = None
        self.node.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.node.create_subscription(Odometry, '/ground_truth', self.on_odom, 10)
        self.cmd_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.mode = None
        self.node.create_subscription(
            String, '/gait_node/mode', lambda m: setattr(self, 'mode', m.data),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.reset_stats()

    def tearDown(self):
        self.node.destroy_node()

    def reset_stats(self):
        self.samples = {'coxa': [], 'femur': [], 'tibia': []}
        self.min_z = math.inf
        self.max_tilt = 0.0

    def on_joints(self, msg):
        self.joints = msg
        if self.recording:
            for name, effort in zip(msg.name, msg.effort):
                self.samples[name.split('_')[1]].append(abs(effort))

    def on_odom(self, msg):
        self.odom = msg
        if self.recording:
            self.min_z = min(self.min_z, msg.pose.pose.position.z)
            self.max_tilt = max(self.max_tilt, *map(abs, self.tilt()))

    def tilt(self):
        return roll_pitch(self.odom.pose.pose.orientation)

    def spin_for(self, seconds, cmd=None):
        end = time.time() + seconds
        while time.time() < end:
            if cmd is not None:
                self.cmd_pub.publish(cmd)
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def wait_for_sim(self):
        end = time.time() + 60
        while time.time() < end and (self.joints is None or self.odom is None):
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertIsNotNone(self.joints, 'no joint states from Gazebo')
        self.assertIsNotNone(self.odom, 'no ground truth from Gazebo')
        self.spin_for(3.0)  # settle on the feet

    def record(self, seconds, cmd=None):
        self.reset_stats()
        self.recording = True
        self.spin_for(seconds, cmd)
        self.recording = False

    def torque_table(self, title):
        print(title)
        for joint, values in self.samples.items():
            values = sorted(values)
            if not values:
                continue
            p50 = values[len(values) // 2] * KGCM
            p95 = values[int(len(values) * 0.95)] * KGCM
            at_limit = sum(v > 1.05 for v in values) / len(values) * 100
            print(f'  {joint:5s} median {p50:5.1f}  p95 {p95:5.1f} kg.cm, at limit {at_limit:4.1f}% of samples')

    def set_param(self, name, value):
        client = self.node.create_client(SetParameters, '/gait_node/set_parameters')
        self.assertTrue(client.wait_for_service(timeout_sec=5.0))
        req = SetParameters.Request()
        req.parameters = [Parameter(name, value=value).to_parameter_msg()]
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        return future.result().results[0].successful

    def call(self, service):
        client = self.node.create_client(Trigger, service)
        self.assertTrue(client.wait_for_service(timeout_sec=5.0))
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        return future.result().success

    def wait_mode(self, mode, timeout):
        end = time.time() + timeout
        while time.time() < end and self.mode != mode:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return self.mode == mode
