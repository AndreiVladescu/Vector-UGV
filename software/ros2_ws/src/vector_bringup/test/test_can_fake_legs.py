"""Full loop over CAN: gait -> ros2_control -> VectorSystem -> vcan0 -> fake_legs.py and back.

Needs a vcan0 interface (sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0);
skips itself otherwise.
"""
import os
import sys
import unittest

import launch
import launch_testing.actions
import pytest
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

sys.path.insert(0, os.path.dirname(__file__))
from test_kinematic import TestKinematicSim  # noqa: E402

HAVE_VCAN = os.path.exists('/sys/class/net/vcan0')


@pytest.mark.launch_test
def generate_test_description():
    if not HAVE_VCAN:
        return launch.LaunchDescription([launch_testing.actions.ReadyToTest()])
    fake_legs = ExecuteProcess(
        cmd=['python3', os.path.join(get_package_share_directory('vector_hw'), '..', '..',
                                     'lib', 'vector_hw', 'fake_legs.py'),
             '--channel', 'vcan0', '--quiet'],
        output='screen')
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('vector_bringup'), 'launch', 'robot.launch.py')),
        launch_arguments={'hardware': 'can', 'can_interface': 'vcan0', 'rviz': 'false'}.items())
    return launch.LaunchDescription([fake_legs, robot, launch_testing.actions.ReadyToTest()])


@unittest.skipUnless(HAVE_VCAN, 'no vcan0 interface')
class TestCanFakeLegs(TestKinematicSim):
    """Same stand / walk / stop / tilt sequence, but the joint states now come back over CAN."""


del TestKinematicSim  # only run it through the CAN subclass here
