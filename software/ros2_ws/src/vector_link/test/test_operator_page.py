"""The operator page's HTTP API against the ROS side: state in, commands out."""
import gzip
import json
import time
import unittest
import urllib.error
import urllib.request

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geographic_msgs.msg import GeoPath
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState, CompressedImage, Imu, NavSatFix
from std_msgs.msg import Empty, UInt8MultiArray
from std_srvs.srv import SetBool, Trigger

PORT = 18080
BASE = f'http://127.0.0.1:{PORT}'


@pytest.mark.launch_test
def generate_test_description():
    page = launch_ros.actions.Node(package='vector_link', executable='operator_page.py', output='screen',
                                   parameters=[{'port': PORT, 'token': 'k'}])
    return launch.LaunchDescription([page, launch_testing.actions.ReadyToTest()])


def request(path, body=None, key='k'):
    url = BASE + path + (f'?key={key}' if key else '')
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(url, data=data, headers={'Content-Type': 'application/json'},
                                 method='POST' if body is not None else 'GET')
    try:
        with urllib.request.urlopen(req, timeout=2) as r:
            return r.status, r.read()
    except urllib.error.HTTPError as e:
        return e.code, e.read()


class TestOperatorPage(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('page_test')
        cls.cmd, cls.hb, cls.missions, cls.calls = [], [], [], []
        cls.node.create_subscription(Twist, '/cmd_vel/teleop', cls.cmd.append, 10)
        cls.node.create_subscription(Empty, '/operator/heartbeat', cls.hb.append, 10)
        cls.node.create_subscription(GeoPath, '/mission', cls.missions.append, 10)

        def trig(req, res):
            cls.calls.append('sentinel')
            res.success = True
            return res

        def estop(req, res):
            cls.calls.append(f'estop {req.data}')
            res.success = True
            return res
        cls.node.create_service(Trigger, '/gait_node/sentinel', trig)
        cls.node.create_service(SetBool, '/gait_node/estop', estop)
        cls.io_calls = []

        def beep(req, res):
            cls.io_calls.append('beep')
            res.success = True
            return res

        def lte(req, res):
            cls.io_calls.append(f'lte {req.data}')
            res.success = True
            return res
        cls.node.create_service(Trigger, '/io/beep', beep)
        cls.node.create_service(SetBool, '/io/lte_power', lte)
        cls.diag = cls.node.create_publisher(DiagnosticArray, '/diagnostics', 10)
        cls.lora = cls.node.create_publisher(UInt8MultiArray, '/lora/rx', 10)
        cls.imu = cls.node.create_publisher(Imu, '/imu', rclpy.qos.qos_profile_sensor_data)
        cls.fix = cls.node.create_publisher(NavSatFix, '/gnss/fix', 10)
        cls.battery = cls.node.create_publisher(BatteryState, '/battery', 10)
        cls.camera = cls.node.create_publisher(CompressedImage, '/yolo/debug/compressed', 10)
        # stands in for gait_node's parameters
        cls.gait = rclpy.create_node('gait_node')
        cls.gait.declare_parameter('gait', 'tripod')
        cls.gait.declare_parameter('body_z', 0.0)
        end = time.time() + 10
        while time.time() < end:
            try:
                urllib.request.urlopen(BASE + '/api/state?key=k', timeout=1)
                break
            except OSError:
                time.sleep(0.2)
        # HTTP answering isn't ROS ready: on a slow box discovery takes a few seconds more, and
        # until the page sees our services and subscriptions it answers 503 or drops commands
        end = time.time() + 15
        while time.time() < end and not (cls.node.count_publishers('/cmd_vel/teleop')
                                         and cls.node.count_clients('/gait_node/sentinel')
                                         and cls.node.count_clients('/gait_node/estop')
                                         and cls.node.count_clients('/io/beep')):
            rclpy.spin_once(cls.node, timeout_sec=0.1)
        end = time.time() + 1.0  # and the other way round
        while time.time() < end:
            rclpy.spin_once(cls.node, timeout_sec=0.05)
            rclpy.spin_once(cls.gait, timeout_sec=0.05)

    @classmethod
    def tearDownClass(cls):
        cls.gait.destroy_node()
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            rclpy.spin_once(self.gait, timeout_sec=0.02)

    def request_spinning(self, path, body):
        """A request that needs gait_node to answer while it waits."""
        import threading
        out = {}
        t = threading.Thread(target=lambda: out.update(r=request(path, body)))
        t.start()
        while t.is_alive():
            self.spin(0.05)
        return out['r']

    def test_page_and_key(self):
        status, body = request('/')
        self.assertEqual(status, 200)
        self.assertIn(b'VECTOR', body)
        self.assertEqual(request('/api/state', key='wrong')[0], 403)

    def test_state_follows_ros(self):
        for _ in range(20):
            self.battery.publish(BatteryState(voltage=15.2, current=-2.0, percentage=0.64))
            self.spin(0.1)
        s = json.loads(request('/api/state')[1])
        self.assertEqual(s['battery']['percent'], 64)
        self.assertEqual(s['video'], ':8889/yolo')

    def test_drive_scaled_and_stopped_by_the_watchdog(self):
        self.cmd.clear()
        self.assertEqual(request('/api/cmd', {'vx': 1, 'vy': 0, 'wz': -2})[0], 200)
        self.spin(0.8)  # no more commands: the watchdog stops it
        self.assertAlmostEqual(self.cmd[0].linear.x, 0.10)
        self.assertAlmostEqual(self.cmd[0].angular.z, -0.5)
        self.assertEqual(self.cmd[-1].linear.x, 0.0)
        self.assertTrue(self.hb)

    def test_mission(self):
        self.assertEqual(request('/api/mission', {'points': [[44.1, 26.1], [44.2, 26.2]]})[0], 200)
        self.assertEqual(request('/api/mission', {'points': [[144.1, 26.1]]})[0], 400)
        self.spin(0.5)
        self.assertEqual(len(self.missions[-1].poses), 2)
        self.assertAlmostEqual(self.missions[-1].poses[1].pose.position.longitude, 26.2)

    def test_buttons(self):
        self.spin(0.5)
        self.assertEqual(request('/api/sentinel', {})[0], 200)
        self.assertEqual(request('/api/estop', {'on': True})[0], 200)
        self.assertEqual(request('/api/wake', {})[0], 503)  # nobody offers wake here
        self.spin(0.5)
        self.assertEqual(self.calls, ['sentinel', 'estop True'])

    def test_height_and_gait(self):
        self.spin(2.5)  # the page's parameter clients find gait_node
        status, body = self.request_spinning('/api/height', {'step': 1})
        self.assertEqual(status, 200, body)
        self.assertAlmostEqual(self.gait.get_parameter('body_z').value, 0.005)
        self.request_spinning('/api/height', {'step': -1})
        self.request_spinning('/api/height', {'step': -1})
        self.assertAlmostEqual(self.gait.get_parameter('body_z').value, -0.005)
        self.assertEqual(self.request_spinning('/api/gait', {'name': 'wave'})[0], 200)
        self.assertEqual(self.gait.get_parameter('gait').value, 'wave')
        self.assertEqual(request('/api/gait', {'name': 'gallop'})[0], 400)

    def test_robot(self):
        status, body = request('/api/robot')
        self.assertEqual(status, 200)
        plain = json.loads(body)
        self.assertEqual(set(plain['mounts']), {'L1', 'L2', 'L3', 'R1', 'R2', 'R3'})
        self.assertTrue(plain['shape']['parts'])  # silhouette.json from the CAD model
        req = urllib.request.Request(BASE + '/api/robot?key=k', headers={'Accept-Encoding': 'gzip'})
        with urllib.request.urlopen(req, timeout=2) as r:
            self.assertEqual(r.headers['Content-Encoding'], 'gzip')
            self.assertEqual(json.loads(gzip.decompress(r.read())), plain)

    def test_robot3d(self):
        self.assertEqual(request('/api/robot3d', key=None)[0], 403)
        req = urllib.request.Request(BASE + '/api/robot3d?key=k', headers={'Accept-Encoding': 'gzip'})
        with urllib.request.urlopen(req, timeout=5) as r:
            self.assertEqual(r.headers['Content-Encoding'], 'gzip')
            body = gzip.decompress(r.read())
        self.assertEqual(request('/api/robot3d')[1], body)  # the same without gzip
        if not body.startswith(b'version https://git-lfs'):  # a checkout without LFS has the pointer
            data = json.loads(body)
            self.assertEqual(set(data['parts']), {'L1', 'L2', 'L3', 'R1', 'R2', 'R3'})
            self.assertTrue(data['body'] and data['colours'])

    def test_icons_and_manifest(self):
        status, body = request('/icon.svg', key=None)  # the browser asks without the key
        self.assertEqual(status, 200)
        self.assertIn(b'<svg', body)
        status, body = request('/apple-touch-icon.png', key=None)
        self.assertEqual((status, body[:4]), (200, b'\x89PNG'))
        self.assertEqual(json.loads(request('/manifest.webmanifest')[1])['start_url'], '/?key=k')
        self.assertEqual(json.loads(request('/manifest.webmanifest', key='wrong')[1])['start_url'], '/')
        self.assertEqual(request('/api/state', key=None)[0], 403)  # everything else still needs it

    def test_snapshot(self):
        self.assertEqual(request('/api/snapshot')[0], 404)
        for _ in range(5):
            self.camera.publish(CompressedImage(format='jpeg', data=b'\xff\xd8fake'))
            self.spin(0.1)
        status, body = request('/api/snapshot')
        self.assertEqual(status, 200)
        self.assertEqual(body, b'\xff\xd8fake')

    def test_io_panel(self):
        self.assertIsNone(json.loads(request('/api/state')[1])['io'])
        vals = {'version': '1234567', 'uptime_s': '3725.5', 'vbat': '14.82', 'v5': '5.03', 'temp': '31.4',
                'sats': '9', 'fix_quality': '1', 'beacons': '12', 'lidar': 'True', 'gnss': 'True', 'fix': 'True',
                'lora': 'True', 'crsf': 'False', 'lte_en': 'False', 'lte_status': 'False', 'lora_rssi': '-97'}
        st = DiagnosticStatus(name='io: mcu', level=DiagnosticStatus.OK, message='14.82 V',
                              values=[KeyValue(key=k, value=v) for k, v in vals.items()])
        imu = DiagnosticStatus(name='imu: compass', level=DiagnosticStatus.WARN, message='no compass')
        for _ in range(10):
            self.diag.publish(DiagnosticArray(status=[st, imu]))
            self.lora.publish(UInt8MultiArray(data=list(b'base: hello')))
            self.spin(0.1)
        s = json.loads(request('/api/state')[1])
        io = s['io']
        self.assertEqual(io['vbat'], 14.82)
        self.assertEqual(io['sats'], 9)
        self.assertEqual(io['beacons'], 12)
        self.assertTrue(io['lidar'] and io['fix'] and not io['crsf'])
        self.assertEqual(io['lora_rssi'], -97.0)
        self.assertIsNone(io['lora_snr'])
        self.assertEqual(s['diag']['imu: compass']['level'], 1)
        self.assertEqual(s['lora_rx']['text'], 'base: hello')

        self.assertEqual(request('/api/beep', {})[0], 200)
        self.assertEqual(request('/api/lte', {'on': True})[0], 200)
        self.spin(0.5)
        self.assertEqual(self.io_calls, ['beep', 'lte True'])

    def test_tiles_data(self):
        m = Imu()
        # level, yaw 0 in ENU = facing east = compass 90
        m.orientation.w = 1.0
        m.orientation_covariance[8] = 0.01
        f = NavSatFix(latitude=44.4268, longitude=26.1025, altitude=81.5)
        f.position_covariance = [3.24, 0.0, 0.0, 0.0, 3.24, 0.0, 0.0, 0.0, 9.0]
        f.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        for _ in range(10):
            self.imu.publish(m)
            self.fix.publish(f)
            self.spin(0.1)
        s = json.loads(request('/api/state')[1])
        self.assertEqual(s['attitude']['heading'], 90)
        self.assertEqual(s['attitude']['roll'], 0.0)
        self.assertTrue(s['attitude']['absolute'])
        self.assertEqual(s['fix']['acc'], 1.8)
        self.assertEqual(s['fix']['alt'], 81.5)
