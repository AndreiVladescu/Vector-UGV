"""power_monitor and leg_config.py power against the power board firmware (sim_power) on vcan0.

The test node stands in for gait_node's sentinel service, to see when the robot would sit down.
"""
import os
import socket
import struct
import subprocess
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from launch.actions import ExecuteProcess
from sensor_msgs.msg import BatteryState
from std_srvs.srv import SetBool, Trigger

HAVE_VCAN = os.path.exists('/sys/class/net/vcan0')
LIB = os.path.join(get_package_share_directory('vector_hw'), '..', '..', 'lib', 'vector_hw')


@pytest.mark.launch_test
def generate_test_description():
    if not HAVE_VCAN:
        return launch.LaunchDescription([ExecuteProcess(cmd=['sleep', '60']), launch_testing.actions.ReadyToTest()])
    board = ExecuteProcess(cmd=[os.path.join(LIB, 'sim_power'), 'vcan0', '--soc', '50', '--halt-s', '1'],
                           output='screen')
    monitor = launch_ros.actions.Node(package='vector_hw', executable='power_monitor',
                                      parameters=[{'can_interface': 'vcan0'}], output='screen')
    return launch.LaunchDescription([board, monitor, launch_testing.actions.ReadyToTest()])


def write_key(key, value, seq):
    """A raw config write to the power board, as leg_config.py would send it."""
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind(('vcan0',))
    data = bytes([0xff, key]) + value.to_bytes(4, 'little', signed=True) + bytes([seq, 1])
    s.send(struct.pack('=IB3x8s', 0x057, 8, data))
    s.close()


@unittest.skipUnless(HAVE_VCAN, 'no vcan0 interface')
class TestCanSimPower(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_power')
        cls.battery = None
        cls.diag = {}
        cls.sentinel_calls = 0

        def sentinel(req, res):
            cls.sentinel_calls += 1
            res.success = True
            return res
        cls.node.create_service(Trigger, '/gait_node/sentinel', sentinel)
        cls.node.create_subscription(BatteryState, '/battery', lambda m: setattr(cls, 'battery', m), 10)
        cls.node.create_subscription(
            DiagnosticArray, '/diagnostics',
            lambda m: cls.diag.update({s.name: s for s in m.status if s.name.startswith('power: ')}), 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin_until(self, cond, timeout):
        end = time.time() + timeout
        while time.time() < end and not cond():
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return cond()

    def call(self, srv_type, name, req):
        client = self.node.create_client(srv_type, name)
        self.assertTrue(client.wait_for_service(timeout_sec=10))
        future = client.call_async(req)
        self.assertTrue(self.spin_until(future.done, 5))
        return future.result()

    def test_1_battery(self):
        self.assertTrue(self.spin_until(lambda: self.battery and 'power: board' in self.diag, 10))
        b = self.battery
        self.assertAlmostEqual(b.percentage, 0.5, delta=0.02)
        self.assertTrue(14.0 < b.voltage < 16.0, b.voltage)
        self.assertLess(b.current, 0)  # 5 V and both sides on
        self.assertEqual(len(b.cell_voltage), 4)
        self.assertEqual(b.power_supply_status, BatteryState.POWER_SUPPLY_STATUS_DISCHARGING)
        self.assertTrue(self.spin_until(lambda: self.battery.capacity > 0, 5))
        self.assertAlmostEqual(self.battery.capacity, 8.0)
        board = self.diag['power: board']
        # a fresh flash has no config saved: a warning, nothing worse
        self.assertEqual(board.level, DiagnosticStatus.WARN, board.message)
        self.assertTrue(board.message.startswith('on'), board.message)
        self.assertEqual(self.diag['power: battery'].level, DiagnosticStatus.OK)
        self.assertEqual(self.diag['power: charger'].message, 'not plugged in')

    def test_2_legs_service(self):
        side = lambda: {v.key: v.value for v in self.diag['power: board'].values}.get('left side')
        res = self.call(SetBool, '/power/legs', SetBool.Request(data=False))
        self.assertTrue(res.success, res.message)
        self.assertTrue(self.spin_until(lambda: side() == 'off', 3))
        res = self.call(SetBool, '/power/legs', SetBool.Request(data=True))
        self.assertTrue(res.success, res.message)
        self.assertTrue(self.spin_until(lambda: side() == 'on', 3))

    def test_3_leg_config_power(self):
        r = subprocess.run(['python3', os.path.join(LIB, 'leg_config.py'), '--channel', 'vcan0', 'power'],
                           capture_output=True, text=True, timeout=30)
        print(r.stdout, r.stderr)
        self.assertIn('power board: on', r.stdout)
        self.assertIn('capacity_mah 8000', r.stdout)
        r = subprocess.run(['python3', os.path.join(LIB, 'leg_config.py'), '--channel', 'vcan0', 'power', 'bms', '0x9304'],
                           capture_output=True, text=True, timeout=30)
        self.assertIn('0x9304: 07 02', r.stdout)

    def test_4_low_battery_then_shutdown(self):
        self.assertEqual(self.sentinel_calls, 0)
        write_key(38, 80, 201)  # SoC 8 %
        self.assertTrue(self.spin_until(lambda: self.sentinel_calls == 1, 3))
        self.assertEqual(self.diag['power: battery'].level, DiagnosticStatus.WARN)

        res = self.call(Trigger, '/power/shutdown', Trigger.Request())
        self.assertTrue(res.success, res.message)
        self.assertTrue(self.spin_until(lambda: self.sentinel_calls == 2, 3))  # halting: sit down
        # the simulated CM5 halts after 1 s, the board switches off, power_monitor goes stale
        self.assertTrue(self.spin_until(
            lambda: self.diag['power: board'].level == DiagnosticStatus.STALE, 6), self.diag['power: board'].message)
