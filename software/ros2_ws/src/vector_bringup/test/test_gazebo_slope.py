"""8 degree slope: without leveling the body follows the ground, with it the body levels."""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(__file__))
from gz_helpers import gazebo_launch, GazeboTest  # noqa: E402


@pytest.mark.launch_test
def generate_test_description():
    return gazebo_launch(world='slope', level='false')


class TestSlope(GazeboTest):

    def test_leveling(self):
        self.wait_for_sim()
        self.spin_for(2.0)
        _, pitch_off = self.tilt()

        self.assertTrue(self.set_param('level', True))
        self.spin_for(5.0)
        _, pitch_on = self.tilt()

        print('\n--- gazebo slope, 8 deg ---')
        print(f'body pitch: leveling off {math.degrees(pitch_off):+.1f} deg, '
              f'on {math.degrees(pitch_on):+.1f} deg')
        self.assertGreater(abs(math.degrees(pitch_off)), 5.0, 'body should follow the slope')
        self.assertLess(abs(math.degrees(pitch_on)), 1.5, 'leveling did not level the body')
