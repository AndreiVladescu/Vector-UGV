"""demo.launch.py end to end: the simulated IO MCU through io_bridge, imu_node on simulated
chips and the mock robot all reach the operator page, and driving from the page moves the
robot and its GNSS position."""
import json
import time
import unittest
import urllib.request

import launch
import launch_testing.actions
import pytest
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

BASE = 'http://127.0.0.1:8080/api/'


@pytest.mark.launch_test
def generate_test_description():
    demo = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        get_package_share_directory('vector_bringup') + '/launch/demo.launch.py'),
        launch_arguments={'gnss_scale': '50'}.items())
    return launch.LaunchDescription([demo, launch_testing.actions.ReadyToTest()])


def get(path):
    try:
        with urllib.request.urlopen(BASE + path, timeout=2) as r:
            return json.loads(r.read())
    except OSError:
        return None


def post(path, body):
    req = urllib.request.Request(BASE + path, data=json.dumps(body).encode(),
                                 headers={'Content-Type': 'application/json'}, method='POST')
    with urllib.request.urlopen(req, timeout=2) as r:
        return r.status


class TestDemo(unittest.TestCase):

    def wait(self, cond, seconds):
        end = time.time() + seconds
        while time.time() < end:
            s = get('state')
            if s and cond(s):
                return s
            time.sleep(0.5)
        self.fail(f'not within {seconds} s; last state: {s}')

    def test_1_everything_reaches_the_page(self):
        s = self.wait(lambda s: s['io'] and s['fix'] and s['attitude'] and s['battery'] and s['mode'] == 'walk'
                      and 'links: elrs' in s['diag'] and s['diag']['links: elrs']['values'].get('lq'), 60)
        self.assertTrue(s['io']['lidar'] and s['io']['gnss'] and s['io']['crsf'] and s['io']['lora'])
        self.assertGreater(s['io']['vbat'], 12)
        self.assertTrue(s['attitude']['absolute'])

    def test_2_driving_moves_the_robot_and_the_fix(self):
        start = self.wait(lambda s: s['fix'], 30)['fix']
        for _ in range(60):  # 6 s forward, the way a held stick does it
            self.assertEqual(post('cmd', {'vx': 1, 'vy': 0, 'wz': 0}), 200)
            time.sleep(0.1)
        pose = get('pose')
        self.assertGreater(pose['motion'][0], 0.02)
        s = self.wait(lambda s: s['battery']['current'] <= -3.9, 5)  # the walking current
        moved = ((s['fix']['lat'] - start['lat']) * 111320, (s['fix']['lon'] - start['lon']) * 79600)
        self.assertGreater((moved[0] ** 2 + moved[1] ** 2) ** 0.5, 5.0)  # x50: ~0.5 m walked is 25 m

    def test_3_buttons_reach_the_simulated_mcu(self):
        self.assertEqual(post('beep', {}), 200)
        self.assertEqual(post('lte', {'on': True}), 200)
        self.wait(lambda s: s['io']['lte_en'], 5)
        self.wait(lambda s: s['io']['lte_status'], 8)  # the module "boots" in 3 s
