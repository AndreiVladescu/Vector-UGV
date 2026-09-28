"""leg_config.py against the leg firmware logic (sim_legs) on vcan0, starting uncalibrated."""
import os
import re
import subprocess
import unittest

import launch
import launch_testing.actions
import pytest
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess

HAVE_VCAN = os.path.exists('/sys/class/net/vcan0')
LIB = os.path.join(get_package_share_directory('vector_hw'), '..', '..', 'lib', 'vector_hw')
REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), *['..'] * 5))


@pytest.mark.launch_test
def generate_test_description():
    # without vcan0 the launch still needs something running, or the skipped tests count as failed
    actions = [ExecuteProcess(cmd=[os.path.join(LIB, 'sim_legs'), 'vcan0'], output='screen') if HAVE_VCAN else
               ExecuteProcess(cmd=['sleep', '60'])]
    return launch.LaunchDescription(actions + [launch_testing.actions.ReadyToTest()])


def leg_config(*args, timeout=90):
    r = subprocess.run(['python3', os.path.join(LIB, 'leg_config.py'), '--channel', 'vcan0', *args],
                       capture_output=True, text=True, timeout=timeout)
    print(r.stdout, r.stderr)
    return r


@unittest.skipUnless(HAVE_VCAN, 'no vcan0 interface')
class TestLegConfig(unittest.TestCase):

    def test_1_starts_uncalibrated(self):
        out = leg_config('status').stdout
        self.assertEqual(out.count('uncalibrated'), 6, out)
        self.assertEqual(out.count(' off '), 6, out)

    def test_2_calibrate_one_joint_full(self):
        r = leg_config('calibrate', 'L2', '--joint', 'coxa')
        self.assertEqual(r.returncode, 0)
        row = re.search(r'\| L2_coxa \|.*?\| (\d+) \| ([\d.]+) \|', r.stdout)
        self.assertIsNotNone(row, r.stdout)
        self.assertTrue(1500 <= int(row.group(1)) <= 1670)
        self.assertTrue(1.34 <= float(row.group(2)) <= 1.51)

    def test_3_calibrate_leg_limits(self):
        r = leg_config('calibrate', 'R1', '--mode', 'limits', '--save')
        self.assertEqual(r.returncode, 0)
        self.assertEqual(len(re.findall(r'\| R1_(coxa|femur|tibia) \|', r.stdout)), 3, r.stdout)
        self.assertIn('saved', r.stdout)

    def test_4_push_from_docs(self):
        r = leg_config('push', '--servos', os.path.join(REPO, 'docs', 'servos.md'),
                       '--legs', os.path.join(get_package_share_directory('vector_description'), 'config', 'legs.yaml'))
        self.assertEqual(r.returncode, 0)
        self.assertEqual(r.stdout.count('saved'), 6, r.stdout)
        out = leg_config('status').stdout
        self.assertNotIn('uncalibrated', out)

    def test_5_flash(self):
        # any bytes with a plausible stack pointer and reset vector pass as an image
        image = (0x200077F0).to_bytes(4, 'little') + (0x08004101).to_bytes(4, 'little') + os.urandom(20000)
        path = os.path.join(os.environ.get('TMPDIR', '/tmp'), f'leg_image_{os.getpid()}.bin')
        with open(path, 'wb') as f:
            f.write(image)
        try:
            r = leg_config('flash', 'L3', path)
        finally:
            os.remove(path)
        self.assertEqual(r.returncode, 0, r.stdout)
        self.assertIn('L3: running', r.stdout)
        out = leg_config('status').stdout
        self.assertNotIn('no answer', out)
        self.assertNotIn('uncalibrated', out)  # the saved config survived the restart

    def test_6_selftest(self):
        r = leg_config('selftest', 'R2')
        self.assertEqual(r.returncode, 0, r.stdout)
        self.assertIn('pass', r.stdout)
        out = leg_config('read', 'R2').stdout
        self.assertIn('last reset power', out)
