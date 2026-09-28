"""Full loop over CAN: gait -> ros2_control -> VectorSystem -> vcan0 -> leg firmware logic (sim_legs) and back.

Needs a vcan0 interface (sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0);
skips itself otherwise.
"""
import math
import os
import sys
import time
import unittest

import yaml
import launch
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

sys.path.insert(0, os.path.dirname(__file__))
from test_kinematic import TestKinematicSim  # noqa: E402

HAVE_VCAN = os.path.exists('/sys/class/net/vcan0')


@pytest.mark.launch_test
def generate_test_description():
    if not HAVE_VCAN:
        # something has to keep the launch alive, or the skipped tests count as failed
        return launch.LaunchDescription([ExecuteProcess(cmd=['sleep', '60']), launch_testing.actions.ReadyToTest()])
    with open(os.path.join(get_package_share_directory('vector_description'), 'config', 'legs.yaml')) as f:
        lim = yaml.safe_load(f)['limits']
    limits = ','.join(f'{math.degrees(a):.1f}' for j in ('coxa', 'femur', 'tibia') for a in lim[j])
    legs = ExecuteProcess(
        cmd=[os.path.join(get_package_share_directory('vector_hw'), '..', '..', 'lib', 'vector_hw', 'sim_legs'),
             'vcan0', '--calibrated', '--limits', limits],
        output='screen')
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('vector_bringup'), 'launch', 'robot.launch.py')),
        launch_arguments={'hardware': 'can', 'can_interface': 'vcan0', 'rviz': 'false'}.items())
    return launch.LaunchDescription([legs, robot, launch_testing.actions.ReadyToTest()])


@unittest.skipUnless(HAVE_VCAN, 'no vcan0 interface')
class TestCanSimLegs(TestKinematicSim):
    """Same stand / walk / stop / tilt sequence, but the joint states now come back over CAN."""

    def test_leg_monitor(self):
        """Six legs in /diagnostics, active and without faults, and ToF ranges (sim_legs says 400 mm)."""
        status, ranges = {}, {}
        self.node.create_subscription(
            DiagnosticArray, '/diagnostics',
            lambda m: status.update({s.name: s for s in m.status if s.name.startswith('legs: ')}), 10)
        for leg in ('L1', 'L2', 'L3', 'R1', 'R2', 'R3'):
            self.node.create_subscription(
                Range, f'/legs/{leg}/tof', lambda m, leg=leg: ranges.update({leg: m}), qos_profile_sensor_data)
        self.wait_ready()
        end = time.time() + 5
        while time.time() < end and (len(status) < 6 or len(ranges) < 6):
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertEqual(len(status), 6, list(status))
        for name, s in status.items():
            self.assertEqual(s.level, DiagnosticStatus.OK, f'{name}: {s.message}')
            self.assertEqual(s.message, 'active')
        self.assertEqual(len(ranges), 6, list(ranges))
        for leg, r in ranges.items():
            self.assertAlmostEqual(r.range, 0.4, delta=0.001)
            self.assertEqual(r.header.frame_id, f'{leg}_tof')


del TestKinematicSim  # only run it through the CAN subclass here
