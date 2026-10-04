#!/usr/bin/env python3
"""The operator page: http://<robot>:8080 on a phone or laptop, over Wi-Fi, LTE or Tailscale.

Video (MediaMTX WebRTC), battery, links, gait mode, a map with the GNSS position where
tapping sets waypoints, a joystick, and Sit / Wake / E-stop. While the page is open it
sends operator/heartbeat, so the link manager knows someone is watching.

JSON API (all POST bodies JSON; with the token parameter set, every request needs ?key=):
  GET  /api/state                   everything the page shows
  GET  /api/pose                    joint angles, walking speed, gait mode and the obstacles
                                    around the robot, for the top-down drawing (polled at 10 Hz)
  GET  /api/robot                   the leg geometry (legs.yaml), once
  POST /api/cmd {vx, vy, wz}        -1..1 each, scaled by max_v / max_w, to cmd_vel/teleop
  POST /api/heartbeat
  POST /api/mission {points: [[lat, lon], ...]}    POST /api/cancel
  POST /api/sentinel  /api/wake  /api/estop {on}
  POST /api/gait {name}             tripod, ripple or wave (only while standing)
  POST /api/height {step}           body height up (+1) or down (-1) by 5 mm, gait_node body_z
  GET  /api/snapshot                the latest annotated camera frame, JPEG
"""
import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
from geographic_msgs.msg import GeoPath, GeoPoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import BatteryState, CompressedImage, JointState, NavSatFix
from std_msgs.msg import Bool, Empty, Float32MultiArray, String
from std_srvs.srv import SetBool, Trigger

try:
    from vision_msgs.msg import Detection2DArray, Detection3DArray
except ImportError:  # the page works without the vision messages, just without detections
    Detection2DArray = Detection3DArray = None

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
HEIGHT_STEP, HEIGHT_MIN, HEIGHT_MAX = 0.005, -0.04, 0.03


class Operator(Node):
    def __init__(self):
        super().__init__('operator_page')
        p = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731
        self.port = p('port', 8080)
        self.token = p('token', '')
        self.max_v, self.max_w = p('max_v', 0.10), p('max_w', 0.5)
        self.video = p('video', ':8889/yolo')  # MediaMTX WebRTC, relative to the page's host
        self.lock = threading.Lock()
        self.s = {'battery': None, 'mode': None, 'active': None, 'mission': None, 'estop': None,
                  'fix': None, 'heading': None, 'diag': {}}
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel/teleop', 10)
        self.hb_pub = self.create_publisher(Empty, 'operator/heartbeat', 10)
        self.mission_pub = self.create_publisher(GeoPath, 'mission', 10)
        self.sentinel = self.create_client(Trigger, 'gait_node/sentinel')
        self.wake = self.create_client(Trigger, 'gait_node/wake')
        self.estop = self.create_client(SetBool, 'gait_node/estop')
        self.cancel = self.create_client(Trigger, 'mission/cancel')
        self.sub(BatteryState, 'battery', self.on_battery)
        self.sub(String, 'gait_node/mode', lambda m: self.put('mode', m.data), LATCHED)
        self.sub(String, 'links/active', lambda m: self.put('active', m.data), LATCHED)
        self.sub(String, 'mission/status', lambda m: self.put('mission', m.data), LATCHED)
        self.sub(Bool, 'power/estop', lambda m: self.put('estop', m.data), LATCHED)
        self.sub(NavSatFix, 'gnss/fix', self.on_fix)
        self.sub(Odometry, 'odometry/global', self.on_odom)
        self.sub(DiagnosticArray, '/diagnostics', self.on_diag)
        self.pose = {'joints': {}, 'motion': [0.0, 0.0, 0.0], 'sectors': None}
        self.sub(JointState, 'joint_states', self.on_joints)
        self.sub(Odometry, 'odom/legs', self.on_legs)
        self.sub(Float32MultiArray, 'obstacles/sectors', self.on_sectors)
        try:
            with open(os.path.join(get_package_share_directory('vector_description'), 'config', 'legs.yaml')) as f:
                g = yaml.safe_load(f)
            self.robot = {k: g[k] for k in ('coxa', 'femur', 'tibia', 'body', 'mounts', 'stand')}
        except (OSError, KeyError, yaml.YAMLError):
            self.robot = None
        self.last_cmd = 0.0
        self.create_timer(0.1, self.cmd_watchdog)
        self.set_params = self.create_client(SetParameters, 'gait_node/set_parameters')
        self.get_params = self.create_client(GetParameters, 'gait_node/get_parameters')
        self.jpeg = None
        self.sub(CompressedImage, 'yolo/debug/compressed', lambda m: setattr(self, 'jpeg', bytes(m.data)))
        self.s['objects'] = []
        self.dists = {}
        if Detection2DArray is not None:
            self.sub(Detection2DArray, 'yolo/detections', self.on_detections)
            self.sub(Detection3DArray, 'yolo/detections_3d', self.on_detections_3d)
        self.s['gait'] = self.s['body_z'] = None
        self.create_timer(2.0, self.refresh_params)

    def sub(self, t, topic, cb, qos=10):
        self.create_subscription(t, topic, cb, qos)

    def put(self, key, value):
        with self.lock:
            self.s[key] = value

    def on_battery(self, m):
        self.put('battery', {'voltage': round(m.voltage, 2), 'current': round(m.current, 2),
                             'percent': round(m.percentage * 100), 'status': m.power_supply_status})

    def on_fix(self, m):
        self.put('fix', {'lat': m.latitude, 'lon': m.longitude, 'status': m.status.status, 't': time.time()})

    def on_odom(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.put('heading', round((90 - math.degrees(yaw)) % 360))  # ENU yaw -> compass

    def on_detections(self, m):
        now = time.time()
        objs = {}
        for d in m.detections:
            if d.results:
                c = d.results[0].hypothesis.class_id
                objs[c] = objs.get(c, 0) + 1
        out = []
        for c, n in sorted(objs.items(), key=lambda kv: -kv[1]):
            dist = self.dists.get(c)
            out.append({'cls': c, 'n': n, 'dist': round(dist[0], 1) if dist and now - dist[1] < 1.0 else None})
        self.put('objects', out)

    def on_detections_3d(self, m):
        now = time.time()
        for d in m.detections:
            if d.results:
                p = d.bbox.center.position
                dist, c = math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z), d.results[0].hypothesis.class_id
                if c not in self.dists or now - self.dists[c][1] > 0.2 or dist < self.dists[c][0]:
                    self.dists[c] = (dist, now)

    def refresh_params(self):
        """The gait and body height as gait_node has them, for the page."""
        if not self.get_params.service_is_ready():
            return
        fut = self.get_params.call_async(GetParameters.Request(names=['gait', 'body_z']))

        def done(f):
            r = f.result()
            if r and len(r.values) == 2:
                self.put('gait', r.values[0].string_value or None)
                self.put('body_z', round(r.values[1].double_value, 3))
        fut.add_done_callback(done)

    def set_param(self, name, value):
        """(ok, reason); waits for gait_node, called from an HTTP thread while the node spins."""
        if not self.set_params.service_is_ready():
            return False, 'gait_node not available'
        v = ParameterValue()
        if isinstance(value, str):
            v.type, v.string_value = ParameterType.PARAMETER_STRING, value
        else:
            v.type, v.double_value = ParameterType.PARAMETER_DOUBLE, float(value)
        done = threading.Event()
        fut = self.set_params.call_async(SetParameters.Request(parameters=[Parameter(name=name, value=v)]))
        fut.add_done_callback(lambda _: done.set())
        if not done.wait(2.0) or not fut.result() or not fut.result().results:
            return False, 'no answer'
        r = fut.result().results[0]
        if r.successful:
            self.put({'gait': 'gait', 'body_z': 'body_z'}[name], value)
        return r.successful, r.reason

    def height(self, step):
        z = self.s.get('body_z') or 0.0
        z = max(HEIGHT_MIN, min(HEIGHT_MAX, z + (HEIGHT_STEP if step > 0 else -HEIGHT_STEP)))
        return self.set_param('body_z', round(z, 3))

    def on_joints(self, m):
        with self.lock:
            self.pose['joints'] = {n: round(p, 3) for n, p in zip(m.name, m.position)}

    def on_legs(self, m):
        t = m.twist.twist
        with self.lock:
            self.pose['motion'] = [round(t.linear.x, 3), round(t.linear.y, 3), round(t.angular.z, 3)]

    def on_sectors(self, m):
        with self.lock:
            self.pose['sectors'] = [None if not math.isfinite(v) else round(v, 2) for v in m.data]
            self.pose['sectors_t'] = time.time()

    def pose_state(self):
        with self.lock:
            p = dict(self.pose)
            p['mode'] = self.s['mode']
        if p.get('sectors_t') and time.time() - p['sectors_t'] > 1.0:
            p['sectors'] = None  # stale: no HUD rather than an old one
        return p

    def on_diag(self, m):
        with self.lock:
            for st in m.status:
                if st.name.startswith(('links: ', 'power: ')):
                    self.s['diag'][st.name] = {'level': int.from_bytes(st.level, 'little') if isinstance(st.level, bytes)
                                               else int(st.level), 'message': st.message}

    def state(self):
        with self.lock:
            s = json.loads(json.dumps(self.s))
        s['video'] = self.video
        s['max_v'], s['max_w'] = self.max_v, self.max_w
        return s

    def drive(self, vx, vy, wz):
        clamp = lambda v: max(-1.0, min(1.0, float(v)))  # noqa: E731
        t = Twist()
        t.linear.x, t.linear.y = clamp(vx) * self.max_v, clamp(vy) * self.max_v
        t.angular.z = clamp(wz) * self.max_w
        self.cmd_pub.publish(t)
        self.hb_pub.publish(Empty())
        self.last_cmd = time.monotonic()

    def cmd_watchdog(self):
        # the page sends at 10 Hz while the stick is held; a dropped phone mustn't leave it walking
        if self.last_cmd and time.monotonic() - self.last_cmd > 0.4:
            self.cmd_pub.publish(Twist())
            self.last_cmd = 0.0

    def mission(self, points):
        path = GeoPath()
        path.header.stamp = self.get_clock().now().to_msg()
        for lat, lon in points:
            g = GeoPoseStamped()
            g.pose.position.latitude, g.pose.position.longitude = float(lat), float(lon)
            path.poses.append(g)
        self.mission_pub.publish(path)

    def call(self, client, req):
        if not client.service_is_ready():
            return False
        client.call_async(req)
        return True


def handler(node, page):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def allowed(self):
            q = parse_qs(urlparse(self.path).query)
            return not node.token or q.get('key', [''])[0] == node.token

        def reply(self, code, body, ctype='application/json'):
            data = body if isinstance(body, bytes) else json.dumps(body).encode()
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if not self.allowed():
                return self.reply(403, {'error': 'key'})
            path = urlparse(self.path).path
            if path in ('/', '/index.html'):
                return self.reply(200, page, 'text/html; charset=utf-8')
            if path == '/api/state':
                return self.reply(200, node.state())
            if path == '/api/pose':
                return self.reply(200, node.pose_state())
            if path == '/api/robot':
                return self.reply(200, node.robot or {})
            if path == '/api/snapshot':
                if node.jpeg is None:
                    return self.reply(404, {'error': 'no camera frame yet'})
                return self.reply(200, node.jpeg, 'image/jpeg')
            self.reply(404, {'error': 'not found'})

        def do_POST(self):
            if not self.allowed():
                return self.reply(403, {'error': 'key'})
            try:
                n = int(self.headers.get('Content-Length') or 0)
                body = json.loads(self.rfile.read(n) or b'{}')
            except (ValueError, json.JSONDecodeError):
                return self.reply(400, {'error': 'bad json'})
            path = urlparse(self.path).path
            ok = True
            if path == '/api/cmd':
                node.drive(body.get('vx', 0), body.get('vy', 0), body.get('wz', 0))
            elif path == '/api/heartbeat':
                node.hb_pub.publish(Empty())
            elif path == '/api/mission':
                pts = body.get('points', [])
                if not all(len(p) == 2 and -90 <= p[0] <= 90 and -180 <= p[1] <= 180 for p in pts):
                    return self.reply(400, {'error': 'points are [lat, lon]'})
                node.mission(pts)
            elif path == '/api/cancel':
                ok = node.call(node.cancel, Trigger.Request())
            elif path == '/api/sentinel':
                ok = node.call(node.sentinel, Trigger.Request())
            elif path == '/api/wake':
                ok = node.call(node.wake, Trigger.Request())
            elif path == '/api/estop':
                ok = node.call(node.estop, SetBool.Request(data=bool(body.get('on', True))))
            elif path in ('/api/gait', '/api/height'):
                if path == '/api/gait':
                    if body.get('name') not in ('tripod', 'ripple', 'wave'):
                        return self.reply(400, {'error': 'tripod, ripple or wave'})
                    ok, reason = node.set_param('gait', body['name'])
                else:
                    ok, reason = node.height(float(body.get('step', 1)))
                return self.reply(200 if ok else 409, {'ok': ok, 'reason': reason})
            else:
                return self.reply(404, {'error': 'not found'})
            self.reply(200 if ok else 503, {'ok': ok})
    return Handler


def main():
    rclpy.init()
    node = Operator()
    page = open(os.path.join(get_package_share_directory('vector_link'), 'web', 'index.html'), 'rb').read()
    server = ThreadingHTTPServer(('0.0.0.0', node.port), handler(node, page))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    node.get_logger().info(f'operator page on http://0.0.0.0:{node.port}' + (' (key required)' if node.token else ''))
    try:
        rclpy.spin(node)
    finally:
        server.shutdown()


if __name__ == '__main__':
    main()
