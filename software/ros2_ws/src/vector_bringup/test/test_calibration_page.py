"""The operator page's calibration wizard against the leg firmware logic (sim_legs) on vcan0:
refused while walking, then a whole leg swept and saved through the HTTP API."""
import json
import os
import time
import unittest
import urllib.error
import urllib.request

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

HAVE_VCAN = os.path.exists('/sys/class/net/vcan0')
LIB = os.path.join(get_package_share_directory('vector_hw'), '..', '..', 'lib', 'vector_hw')
PORT = 18090


@pytest.mark.launch_test
def generate_test_description():
    page = launch_ros.actions.Node(package='vector_link', executable='operator_page.py', output='screen',
                                   parameters=[{'port': PORT, 'can_interface': 'vcan0'}])
    legs = (ExecuteProcess(cmd=[os.path.join(LIB, 'sim_legs'), 'vcan0'], output='screen') if HAVE_VCAN else
            ExecuteProcess(cmd=['sleep', '60']))
    return launch.LaunchDescription([legs, page, launch_testing.actions.ReadyToTest()])


def request(path, body=None):
    req = urllib.request.Request(f'http://127.0.0.1:{PORT}{path}', data=json.dumps(body).encode() if body is not None else None,
                                 headers={'Content-Type': 'application/json'}, method='POST' if body is not None else 'GET')
    try:
        with urllib.request.urlopen(req, timeout=3) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


@unittest.skipUnless(HAVE_VCAN, 'no vcan0 interface')
class TestCalibrationPage(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('cal_test')
        cls.mode = cls.node.create_publisher(String, '/gait_node/mode',
                                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        end = time.time() + 20  # the page's server starts a moment after the launch
        while time.time() < end:
            try:
                request('/api/state')
                break
            except (urllib.error.URLError, ConnectionError):
                time.sleep(0.3)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def set_mode(self, mode):
        self.mode.publish(String(data=mode))
        end = time.time() + 5
        while time.time() < end and request('/api/state')[1]['mode'] != mode:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertEqual(request('/api/state')[1]['mode'], mode)

    def test_calibrate_a_leg_from_the_page(self):
        self.set_mode('walk')
        status, body = request('/api/calibrate', {'leg': 'R2'})
        self.assertEqual(status, 409, body)
        self.assertIn('sit the robot down', body['error'])
        self.assertEqual(request('/api/calibrate', {'leg': 'X9'})[0], 400)

        self.set_mode('sentinel')
        self.assertEqual(request('/api/calibrate', {'leg': 'R2', 'joint': 'all', 'mode': 'full'})[0], 200)
        self.assertEqual(request('/api/calibrate', {'leg': 'R2'})[0], 409)  # one at a time
        end, cal = time.time() + 60, None
        while time.time() < end:
            cal = request('/api/state')[1]['cal']
            if cal['phase'] != 'running':
                break
            time.sleep(0.5)
        self.assertEqual(cal['phase'], 'done', cal)
        self.assertEqual(set(cal['results']), {'coxa', 'femur', 'tibia'})
        for r in cal['results'].values():
            self.assertTrue(1.3 <= r['slope'] <= 1.6, cal)
        self.assertEqual(request('/api/calibrate/save', {})[0], 200)
        self.assertTrue(request('/api/state')[1]['cal']['saved'])
