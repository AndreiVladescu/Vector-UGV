"""Rough ground (1-3 cm slabs): walk across open loop, no foot contact sensing yet."""
import os
import sys

import pytest
from geometry_msgs.msg import Twist

sys.path.insert(0, os.path.dirname(__file__))
from gz_helpers import gazebo_launch, GazeboTest  # noqa: E402


@pytest.mark.launch_test
def generate_test_description():
    return gazebo_launch(world='rough', level='true')


class TestRough(GazeboTest):

    def test_walks_across(self):
        self.wait_for_sim()
        start = self.odom.pose.pose.position
        cmd = Twist()
        cmd.linear.x = 0.08
        self.record(15.0, cmd)
        end = self.odom.pose.pose.position

        print('\n--- gazebo rough, leveling on ---')
        self.torque_table('tripod walk, 80 mm/s:')
        print(f'walked {end.x - start.x:.3f} m, drift {end.y - start.y:+.3f} m, '
              f'lowest body {self.min_z * 1000:.0f} mm, max tilt {self.max_tilt * 57.3:.1f} deg')

        self.assertLess(self.max_tilt * 57.3, 25.0, 'robot tipped')
        self.assertGreater(self.min_z, 0.04, 'body hit the ground')
        self.assertGreater(end.x - start.x, 0.5, 'got stuck')
