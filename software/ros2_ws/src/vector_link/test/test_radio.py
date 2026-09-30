"""elrs.py and link_manager.py with a fake receiver on a pseudo-terminal: arming, sticks,
switches, telemetry back to the radio, and what happens when the link goes."""
import os
import pty
import time
import tty
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from vector_link import crsf

MASTER, SLAVE = pty.openpty()
tty.setraw(MASTER)
tty.setraw(SLAVE)
PORT = os.ttyname(SLAVE)


@pytest.mark.launch_test
def generate_test_description():
    radio = launch_ros.actions.Node(package='vector_link', executable='elrs.py', output='screen',
                                    parameters=[{'port': PORT, 'baud': 115200}])
    link = launch_ros.actions.Node(package='vector_link', executable='link_manager.py', output='screen',
                                   parameters=[{'loss_s': 2.0}])
    return launch.LaunchDescription([radio, link, launch_testing.actions.ReadyToTest()])


def channels(arm=False, fwd=0.0, sentinel=False, estop=False):
    ch = [crsf.CH_MID] * 16
    ch[1] = int(crsf.CH_MID + fwd * (crsf.CH_MAX - crsf.CH_MID))
    ch[2] = crsf.CH_MAX  # full speed
    ch[4] = crsf.CH_MAX if arm else crsf.CH_MIN
    ch[5] = crsf.CH_MAX if sentinel else crsf.CH_MIN
    ch[6] = crsf.CH_MAX if estop else crsf.CH_MIN
    return crsf.frame(crsf.T_RC_CHANNELS, crsf.pack_channels(ch))


class TestRadio(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('radio_test')
        cls.cmd = []
        cls.calls = []
        cls.active = None
        cls.node.create_subscription(Twist, '/cmd_vel', lambda m: cls.cmd.append(m), 50)
        cls.node.create_subscription(String, '/links/active', lambda m: setattr(cls, 'active', m.data),
                                     rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL))

        def trig(name):
            def cb(req, res):
                cls.calls.append(name)
                res.success = True
                return res
            return cb
        cls.node.create_service(Trigger, '/gait_node/sentinel', trig('sentinel'))
        cls.node.create_service(Trigger, '/gait_node/wake', trig('wake'))

        def estop(req, res):
            cls.calls.append(f'estop {req.data}')
            res.success = True
            return res
        cls.node.create_service(SetBool, '/gait_node/estop', estop)
        cls.battery = cls.node.create_publisher(BatteryState, '/battery', 10)
        os.set_blocking(MASTER, False)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def send_for(self, seconds, frame):
        end = time.time() + seconds
        while time.time() < end:
            os.write(MASTER, frame)
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def test_1_disarmed_sends_nothing(self):
        self.spin(3)  # nodes up
        self.send_for(1.0, channels(arm=False, fwd=1.0))
        self.assertFalse(any(m.linear.x > 0 for m in self.cmd))

    def test_2_armed_drives(self):
        self.send_for(1.0, channels(arm=True, fwd=1.0))
        self.assertTrue(any(abs(m.linear.x - 0.10) < 1e-6 for m in self.cmd))
        self.assertEqual(self.active, 'elrs')

    def test_3_switches(self):
        self.send_for(0.3, channels(arm=True))
        self.calls.clear()  # the link manager sat it down while the radio was quiet between tests
        self.send_for(0.5, channels(arm=True, sentinel=True))
        self.send_for(0.5, channels(arm=True, sentinel=False))
        self.send_for(0.5, channels(arm=True, estop=True))
        self.spin(0.5)
        self.assertEqual(self.calls[:3], ['sentinel', 'wake', 'estop True'])

    def test_4_telemetry(self):
        self.battery.publish(BatteryState(voltage=15.4, current=-3.0, percentage=0.7, capacity=8.0))
        data = b''
        end = time.time() + 3
        while time.time() < end:
            os.write(MASTER, channels(arm=True))
            rclpy.spin_once(self.node, timeout_sec=0.05)
            try:
                data += os.read(MASTER, 256)
            except BlockingIOError:
                pass
        frames = crsf.Parser().feed(data)
        bat = [p for t, p in frames if t == crsf.T_BATTERY]
        self.assertTrue(bat, 'no battery telemetry')
        self.assertEqual(int.from_bytes(bat[-1][0:2], 'big'), 154)
        self.assertEqual(bat[-1][7], 70)
        self.assertTrue(any(t == crsf.T_FLIGHT_MODE for t, _ in frames))

    def test_5_link_lost_stops_and_sits_down(self):
        self.cmd.clear()
        self.calls.clear()
        self.spin(1.0)  # the radio stops: one zero, then quiet
        self.assertTrue(self.cmd and self.cmd[-1].linear.x == 0)
        self.spin(3.0)  # loss_s = 2
        self.assertIn('sentinel', self.calls)
        self.assertEqual(self.active, 'idle')
