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

    def key(self, twist):
        """One message, like teleop_twist_keyboard sends per key press."""
        self.cmd_pub.publish(twist)
        rclpy.spin_once(self.node, timeout_sec=0.05)

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

        # Stop (teleop 'k' sends one zero twist): back into the same standing pose.
        self.key(Twist())
        self.spin_for(3.0)
        for name, angle in standing.items():
            self.assertAlmostEqual(self.joints[name], angle, delta=0.05, msg=name)

        # Gait switch is allowed while standing, body pose tilts the body.
        self.assertTrue(self.set_param('gait', 'ripple'))
        self.assertTrue(self.set_param('body_pitch', 0.15))
        self.spin_for(1.5)
        t = self.tf.lookup_transform('odom', 'base_link', rclpy.time.Time())
        self.assertGreater(abs(t.transform.rotation.y), 0.05, 'body did not pitch')

    def test_teleop_keys(self):
        """Single messages like teleop_twist_keyboard: the command holds until the next key."""
        end = time.time() + 30
        while time.time() < end and 'L1_femur' not in self.joints:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.spin_for(3.0)

        def pose():
            return self.tf.lookup_transform('odom', 'base_link', rclpy.time.Time()).transform.translation

        # 'i' once, then nothing for 2 s: still walking forward
        fwd = Twist()
        fwd.linear.x = 0.5
        self.key(fwd)
        self.spin_for(1.0)
        mid = pose()
        self.spin_for(1.0)
        self.assertGreater(pose().x - mid.x, 0.05, 'stopped after a single key press')

        # 't' four times while walking: body 20 mm up, and it keeps walking
        up = Twist()
        up.linear.z = 0.5
        for _ in range(4):
            self.key(up)
        before = pose()
        self.spin_for(1.5)
        self.assertGreater(pose().x - before.x, 0.05, "'t' stopped the walk")
        self.assertAlmostEqual(pose().z, 0.12, delta=0.002, msg='body not 20 mm up')

        # 'b' once: 15 mm up
        down = Twist()
        down.linear.z = -0.5
        self.key(down)
        self.spin_for(1.0)
        self.assertAlmostEqual(pose().z, 0.115, delta=0.002)

        # 'J' (shift, holonomic): strafe left
        left = Twist()
        left.linear.y = 0.5
        self.key(left)
        before = pose()
        self.spin_for(2.0)
        self.assertGreater(pose().y - before.y, 0.1, 'no sideways motion')

        # 'k': stop
        self.key(Twist())
        self.spin_for(3.0)
        stopped = pose()
        self.spin_for(1.0)
        self.assertAlmostEqual(pose().x, stopped.x, delta=0.002, msg='still walking after k')
        self.assertAlmostEqual(pose().y, stopped.y, delta=0.002, msg='still walking after k')

    def test_hold_to_move(self):
        """cmd_timeout 0.6: walks while a key is held (auto-repeat), stops when released."""
        end = time.time() + 30
        while time.time() < end and 'L1_femur' not in self.joints:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.spin_for(3.0)
        self.assertTrue(self.set_param('cmd_timeout', 0.6))

        def x():
            return self.tf.lookup_transform('odom', 'base_link', rclpy.time.Time()).transform.translation.x

        fwd = Twist()
        fwd.linear.x = 0.5
        start = x()
        # typical auto-repeat: first press, 0.5 s delay, then ~30 per second for 2 s
        self.key(fwd)
        self.spin_for(0.5)
        for _ in range(60):
            self.key(fwd)
            self.spin_for(1 / 30)
        held = x()
        self.assertGreater(held - start, 0.15, 'did not keep walking while held')
        self.spin_for(3.0)  # released
        stopped = x()
        self.spin_for(1.0)
        self.assertAlmostEqual(x(), stopped, delta=0.002, msg='kept walking after release')
        self.assertTrue(self.set_param('cmd_timeout', 0.0))
