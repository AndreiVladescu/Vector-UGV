"""Brings up robot.launch.py with mock joints, headless, and checks that it stands, walks and stops."""
import os
import time
import unittest

import launch
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener


@pytest.mark.launch_test
def generate_test_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('vector_bringup'), 'launch', 'robot.launch.py')),
        launch_arguments={'rviz': 'false'}.items())
    return launch.LaunchDescription([sim, launch_testing.actions.ReadyToTest()])


class TestKinematicSim(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('sim_test')
        self.joints = {}
        self.node.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.cmd_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self.node)

    def tearDown(self):
        self.node.destroy_node()

    def on_joints(self, msg):
        self.joints = dict(zip(msg.name, msg.position))

    def spin_for(self, seconds, cmd=None, each=None):
        end = time.time() + seconds
        while time.time() < end:
            if cmd is not None:
                self.cmd_pub.publish(cmd)
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if each and self.joints:
                each()

    def odom_x(self):
        t = self.tf.lookup_transform('odom', 'base_link', rclpy.time.Time())
        return t.transform.translation

    def set_param(self, name, value):
        client = self.node.create_client(SetParameters, '/gait_node/set_parameters')
        self.assertTrue(client.wait_for_service(timeout_sec=5.0))
        req = SetParameters.Request()
        req.parameters = [Parameter(name, value=value).to_parameter_msg()]
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        return future.result().results[0].successful

    def test_stand_walk_stop(self):
        # Wait for the controllers, then remember the standing pose.
        end = time.time() + 30
        while time.time() < end and 'L1_femur' not in self.joints:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertIn('L1_femur', self.joints, 'no joint states')
        self.spin_for(3.0)
        standing = dict(self.joints)
        self.assertGreater(standing['L1_femur'], 0.1, 'not in the standing pose')

        # Walk forward: every femur must lift at some point, and the body must move.
        cmd = Twist()
        cmd.linear.x = 0.1
        lifted = {name: 0.0 for name in standing if name.endswith('_femur')}

        def track():
            for name in lifted:
                lifted[name] = max(lifted[name], self.joints[name] - standing[name])

        self.spin_for(4.0, cmd, track)
        for name, lift in lifted.items():
            self.assertGreater(lift, 0.2, f'{name} never lifted')
        self.assertGreater(self.odom_x().x, 0.2)

        # Stop: the robot settles back into the same standing pose.
        self.spin_for(3.0)
        for name, angle in standing.items():
            self.assertAlmostEqual(self.joints[name], angle, delta=0.05, msg=name)

        # Gait switch is allowed while standing, body pose tilts the body.
        self.assertTrue(self.set_param('gait', 'ripple'))
        self.assertTrue(self.set_param('body_pitch', 0.15))
        self.spin_for(1.5)
        t = self.tf.lookup_transform('odom', 'base_link', rclpy.time.Time())
        self.assertGreater(abs(t.transform.rotation.y), 0.05, 'body did not pitch')
