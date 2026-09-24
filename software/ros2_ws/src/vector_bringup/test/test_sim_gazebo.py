"""Flat ground: stand, then walk; prints joint torques.

Peaks hit the effort limit during fast swings (the position loop saturates), so the
median and 95th percentile are the useful numbers.
"""
import os
import sys

import pytest
from geometry_msgs.msg import Twist

sys.path.insert(0, os.path.dirname(__file__))
from gz_helpers import gazebo_launch, GazeboTest  # noqa: E402


@pytest.mark.launch_test
def generate_test_description():
    return gazebo_launch(world='flat')


class TestFlat(GazeboTest):

    def test_walks_without_falling(self):
        self.wait_for_sim()
        self.record(2.0)
        print('\n--- gazebo flat ---')
        self.torque_table('standing:')
        start = self.odom.pose.pose.position
        self.assertGreater(start.z, 0.06, f'body sagged to {start.z:.3f} m while standing')

        cmd = Twist()
        cmd.linear.x = 0.08
        self.record(8.0, cmd)
        end = self.odom.pose.pose.position
        self.torque_table('tripod walk, 80 mm/s:')
        print(f'walked {end.x - start.x:.3f} m, drift {end.y - start.y:+.3f} m, '
              f'lowest body {self.min_z * 1000:.0f} mm, max tilt {self.max_tilt * 57.3:.1f} deg')

        self.assertGreater(self.min_z, 0.05, 'body hit the ground')
        self.assertLess(self.max_tilt * 57.3, 20.0, 'robot tipped')
        self.assertGreater(end.x - start.x, 0.2, 'did not walk forward')
