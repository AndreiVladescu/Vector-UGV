#!/usr/bin/env python3
"""The one node that writes cmd_vel, and the link watchdog.

  cmd_vel/elrs    the radio (elrs.py, only while armed)            highest priority
  cmd_vel/teleop  an operator over Wi-Fi or LTE (Foxglove, teleop_twist_keyboard remapped)
  cmd_vel/nav     the GPS mission (waypoints.py)                     lowest
  operator/heartbeat  std_msgs/Empty from the operator's screen, about 1 Hz; teleop counts too

With no radio link and no operator for loss_s (5 s) it stops the robot and asks gait_node
to sit down; it stays down until someone wakes it. With return_home (on by default) and a
GNSS fix it first sends a mission to the home point and sits down when the robot gets there
(or can't); a link coming back cancels that mission. Home: geographic_msgs/GeoPoint on
home/set (the operator page's Set home), else the first fix after start; published latched
on `home`. A GPS mission only moves while a link is up, unless missions_alone is set. Status: links/active (latched String: elrs, teleop, nav or
idle) and /diagnostics (control, Wi-Fi signal, LTE signal from the modem's AT port).
"""
import threading
import time

import rclpy
import serial
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geographic_msgs.msg import GeoPath, GeoPoint, GeoPoseStamped
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Empty, String, UInt8
from std_srvs.srv import Trigger

from vector_link import net
from vector_link.manager import SOURCES, LinkManager


class LinkNode(Node):
    def __init__(self):
        super().__init__('link_manager')
        p = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731
        self.m = LinkManager(loss_s=p('loss_s', 5.0), heartbeat_s=p('heartbeat_s', 3.0),
                             missions_alone=p('missions_alone', False), return_home=p('return_home', True))
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.home, self.fix_t = None, -1e9
        self.home_pub = self.create_publisher(GeoPoint, 'home', latched)
        self.mission_pub = self.create_publisher(GeoPath, 'mission', 10)
        self.cancel = self.create_client(Trigger, 'mission/cancel')
        self.create_subscription(GeoPoint, 'home/set', lambda msg: self.set_home(msg.latitude, msg.longitude, 'set'), 10)
        self.create_subscription(NavSatFix, 'gnss/fix', self.on_fix, 10)
        self.create_subscription(String, 'mission/status', lambda msg: self.m.on_mission_status(msg.data, time.monotonic()),
                                 latched)
        self.wifi_if = p('wifi_iface', 'wlan0')
        self.lte_if = p('lte_iface', 'auto')      # the A7670E in RNDIS / ECM mode; auto: by driver
        self.at_port = p('lte_at_port', '')       # e.g. /dev/ttyUSB2; '' = don't ask the modem
        self.lte_dbm = None
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.active_pub = self.create_publisher(String, 'links/active',
                                                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.sentinel = self.create_client(Trigger, 'gait_node/sentinel')
        for s in SOURCES:
            self.create_subscription(Twist, f'cmd_vel/{s}', lambda msg, s=s: self.on_cmd(s, msg), 10)
        self.create_subscription(UInt8, 'elrs/link', lambda msg: self.m.on_radio(msg.data, time.monotonic()), 10)
        self.create_subscription(Empty, 'operator/heartbeat', lambda _: self.m.on_heartbeat(time.monotonic()), 10)
        self.active = None
        self.create_timer(0.05, self.tick)
        self.create_timer(1.0, self.report)
        if self.at_port:
            threading.Thread(target=self.modem, daemon=True).start()
        self.publish_active('idle')

    def set_home(self, lat, lon, how):
        self.home = (lat, lon)
        self.home_pub.publish(GeoPoint(latitude=lat, longitude=lon))
        self.get_logger().info(f'home {how}: {lat:.6f}, {lon:.6f}')

    def on_fix(self, msg):
        if msg.status.status >= NavSatStatus.STATUS_FIX:
            self.fix_t = time.monotonic()
            if self.home is None:
                self.set_home(msg.latitude, msg.longitude, 'from the first fix')

    def on_cmd(self, source, msg):
        self.m.on_cmd(source, msg.linear.x, msg.linear.y, msg.angular.z, time.monotonic())

    def publish_active(self, name):
        if name != self.active:
            self.active = name
            self.active_pub.publish(String(data=name))
            self.get_logger().info(f'control: {name}')

    def tick(self):
        now = time.monotonic()
        cmd, action = self.m.step(now, home_ready=self.home is not None and now - self.fix_t < 5.0)
        if cmd is not None:
            t = Twist()
            t.linear.x, t.linear.y, t.angular.z = cmd
            self.cmd_pub.publish(t)
        self.publish_active(self.m.source or 'idle')
        if action == 'sentinel':
            self.get_logger().warn('no radio and no operator: stopping and sitting down')
            if self.sentinel.service_is_ready():
                self.sentinel.call_async(Trigger.Request())
        elif action == 'home':
            self.get_logger().warn('no radio and no operator: walking home')
            path = GeoPath()
            path.header.stamp = self.get_clock().now().to_msg()
            p = GeoPoseStamped()
            p.pose.position.latitude, p.pose.position.longitude = self.home
            path.poses.append(p)
            self.mission_pub.publish(path)
        elif action == 'cancel':
            self.get_logger().info('link back: the walk home is cancelled')
            if self.cancel.service_is_ready():
                self.cancel.call_async(Trigger.Request())

    def modem(self):
        """AT+CSQ every 10 s on the modem's AT port."""
        while rclpy.ok():
            try:
                with serial.Serial(self.at_port, 115200, timeout=1) as s:
                    s.write(b'AT+CSQ\r')
                    self.lte_dbm = net.csq_dbm(s.read(64).decode(errors='replace'))
            except (serial.SerialException, OSError):
                self.lte_dbm = None
            time.sleep(10)

    def report(self):
        now = time.monotonic()
        links = self.m.links(now)
        route = net.default_route()
        control = DiagnosticStatus(name='links: control', hardware_id='link_manager')
        lost = not any(links.values())
        control.level = DiagnosticStatus.ERROR if self.m.sat_down else DiagnosticStatus.WARN if lost else DiagnosticStatus.OK
        control.message = (f'{self.active}; ' + ('links lost, sat down' if self.m.sat_down else
                           'links lost, walking home' if self.m.returning else 'no links' if lost else
                           ', '.join(k for k, v in links.items() if v)))
        control.values = [KeyValue(key='radio', value=str(links['radio'])),
                          KeyValue(key='operator', value=str(links['operator'])),
                          KeyValue(key='default route', value=str(route))]
        wifi = DiagnosticStatus(name='links: wifi', hardware_id=self.wifi_if)
        dbm = net.wifi_dbm(self.wifi_if)
        wifi.level = DiagnosticStatus.OK if dbm is not None else DiagnosticStatus.WARN
        wifi.message = f'{dbm} dBm' if dbm is not None else net.operstate(self.wifi_if)
        iface = net.modem_iface() if self.lte_if == 'auto' else self.lte_if
        lte = DiagnosticStatus(name='links: lte', hardware_id=iface or 'no modem')
        state = net.operstate(iface) if iface else 'absent'
        lte.level = DiagnosticStatus.OK if state == 'up' or state == 'unknown' else DiagnosticStatus.WARN
        lte.message = state + (f', {self.lte_dbm} dBm' if self.lte_dbm is not None else '')
        arr = DiagnosticArray(status=[control, wifi, lte])
        arr.header.stamp = self.get_clock().now().to_msg()
        self.diag_pub.publish(arr)


def main():
    rclpy.init()
    rclpy.spin(LinkNode())


if __name__ == '__main__':
    main()
