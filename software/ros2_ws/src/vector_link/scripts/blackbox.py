#!/usr/bin/env python3
"""The black box: the last two minutes of what the robot did, saved when something goes wrong.

Keeps the topics below in memory (vector_link/blackbox.py) and writes them to an MCAP bag
(opens in Foxglove, `ros2 bag play`) post_s seconds after:
  an e-stop (gait_node/mode straight from walk to sentinel), a leg fault (halted),
  a new ERROR from the legs, power or links in /diagnostics, a fall (IMU tilt over tilt_deg),
  or ~/save (std_srvs/Trigger; the operator page's button).
Bags go to `dir` as <date>-<time>_<reason>, the newest max_bags kept; each save is announced
on blackbox/saved (String: the bag's name).
"""
import math
import os
import shutil
import time
from datetime import datetime

import rclpy
import rclpy.executors
import rosbag2_py
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_srvs.srv import Trigger

from vector_link.blackbox import BlackBox, Triggers

TOPICS = ['/joint_states', '/cmd_vel', '/cmd_vel/teleop', '/cmd_vel/elrs', '/cmd_vel/nav', '/gait_node/mode',
          '/diagnostics', '/battery', '/imu', '/gnss/fix', '/links/active', '/mission/status', '/odom/legs',
          '/obstacles/sectors', '/home']


class BlackBoxNode(Node):
    def __init__(self):
        super().__init__('blackbox')
        p = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731
        self.dir = os.path.expanduser(p('dir', '~/.ros/blackbox'))
        self.max_bags = p('max_bags', 30)
        self.topics = p('topics', TOPICS)
        self.box = BlackBox(window_s=p('window_s', 120.0), post_s=p('post_s', 10.0), max_hz=p('max_hz', 50.0))
        self.triggers = Triggers(tilt_deg=p('tilt_deg', 45.0))
        self.types = {}
        self.saved_pub = self.create_publisher(String, 'blackbox/saved', 10)
        self.create_service(Trigger, '~/save', self.on_save)
        self.create_timer(2.0, self.discover)
        self.create_timer(0.5, self.check)
        self.discover()

    def discover(self):
        """Subscribe to the listed topics as they appear, whatever their type."""
        for topic, types in self.get_topic_names_and_types():
            if topic in self.topics and topic not in self.types and types:
                try:
                    cls = get_message(types[0])
                except (AttributeError, ModuleNotFoundError, ValueError):
                    continue
                # latched topics (the gait mode, home) latched here too, so the state from
                # before this node started arrives; anything else best effort, which matches every publisher
                pubs = self.get_publishers_info_by_topic(topic)
                latched = pubs and all(p.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL for p in pubs)
                qos = (QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
                       if latched else QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))
                self.types[topic] = (types[0], cls)
                self.create_subscription(cls, topic, lambda data, topic=topic: self.on_msg(topic, data), qos, raw=True)

    def on_msg(self, topic, data):
        now = time.time()
        self.box.add(now, topic, data)
        reason = None
        if topic == '/gait_node/mode':
            reason = self.triggers.on_mode(deserialize_message(data, String).data)
        elif topic == '/diagnostics':
            for s in deserialize_message(data, DiagnosticArray).status:
                level = int.from_bytes(s.level, 'little') if isinstance(s.level, bytes) else int(s.level)
                reason = self.triggers.on_diag(s.name, level) or reason
        elif topic == '/imu':
            q = deserialize_message(data, Imu).orientation
            roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
            pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
            reason = self.triggers.on_tilt(roll, pitch)
        if reason:
            self.get_logger().warn(f'{reason}: saving the black box in {self.box.post_s:.0f} s')
            self.box.trigger(now, reason)

    def on_save(self, req, res):
        self.box.trigger(time.time(), 'manual')
        res.success, res.message = True, f'saving in {self.box.post_s:.0f} s'
        return res

    def check(self, now=None):
        out = self.box.due(time.time() if now is None else now)
        if out:
            self.write(*out)

    def flush(self):
        """On shutdown: a save still waiting for its post_s is written with what there is."""
        if self.box.pending:
            self.check(now=math.inf)

    def write(self, reasons, msgs):
        tag = '_'.join(r.replace(' ', '-').replace(':', '') for r in reasons)[:60]
        name = datetime.now().strftime('%Y%m%d-%H%M%S') + '_' + tag
        path, n = os.path.join(self.dir, name), 1
        while os.path.exists(path):  # two saves in one second
            n += 1
            path = os.path.join(self.dir, f'{name}-{n}')
        name = os.path.basename(path)
        try:
            os.makedirs(self.dir, exist_ok=True)
            w = rosbag2_py.SequentialWriter()
            w.open(rosbag2_py.StorageOptions(uri=path, storage_id='mcap'),
                   rosbag2_py.ConverterOptions('cdr', 'cdr'))
            for i, topic in enumerate(sorted({m[1] for m in msgs})):
                w.create_topic(rosbag2_py.TopicMetadata(id=i, name=topic, type=self.types[topic][0],
                                                        serialization_format='cdr'))
            for t, topic, data in msgs:
                w.write(topic, data, int(t * 1e9))
            del w  # closes the bag
        except Exception as e:  # noqa: BLE001: a full disk must not take the robot down
            self.get_logger().error(f'black box not saved: {e}')
            shutil.rmtree(path, ignore_errors=True)
            return
        self.get_logger().info(f'black box saved: {path} ({len(msgs)} messages)')
        self.saved_pub.publish(String(data=name))
        bags = sorted(d for d in os.listdir(self.dir) if os.path.isdir(os.path.join(self.dir, d)))
        for old in bags[:-self.max_bags]:
            shutil.rmtree(os.path.join(self.dir, old), ignore_errors=True)


def main():
    rclpy.init()
    node = BlackBoxNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.flush()


if __name__ == '__main__':
    main()
