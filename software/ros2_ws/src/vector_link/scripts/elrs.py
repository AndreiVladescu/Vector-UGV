#!/usr/bin/env python3
"""ExpressLRS receiver on a UART (CRSF, 420000 baud) -> the robot.

Mode 2 sticks: right stick forward / sideways, left stick turns (sideways) and sets the
top speed (throttle, 30-100 %). Switches:
  AUX1 (ch 5)  arm: only then are the sticks sent, on cmd_vel/elrs (the link manager
               gives the radio priority over everything else)
  AUX2 (ch 6)  high = Sentinel (sit down, legs off), back low = wake
  AUX3 (ch 7)  high = e-stop (gait_node/estop), back low releases it; wake with AUX2 after
Publishes elrs/link (std_msgs/UInt8, link quality %, 0 = lost) at 10 Hz and the link in
/diagnostics. Sends the battery (from /battery) and the gait mode back as telemetry.
"""
import threading
import time

import rclpy
import serial
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String, UInt8
from std_srvs.srv import SetBool, Trigger

from vector_link import crsf


class Elrs(Node):
    def __init__(self):
        super().__init__('elrs')
        p = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731
        self.port = p('port', '/dev/ttyAMA2')
        self.baud = p('baud', 420000)
        self.max_v, self.max_w = p('max_v', 0.10), p('max_w', 0.5)
        self.deadband = p('deadband', 0.05)
        self.ch = None
        self.ch_t = 0.0
        self.stats = {}
        self.stats_t = 0.0
        self.switch = {}
        self.battery = None
        self.mode = ''
        self.lock = threading.Lock()
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel/elrs', 10)
        self.link_pub = self.create_publisher(UInt8, 'elrs/link', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.sentinel = self.create_client(Trigger, 'gait_node/sentinel')
        self.wake = self.create_client(Trigger, 'gait_node/wake')
        self.estop = self.create_client(SetBool, 'gait_node/estop')
        self.create_subscription(BatteryState, 'battery', lambda m: setattr(self, 'battery', m), 10)
        self.create_subscription(String, 'gait_node/mode', lambda m: setattr(self, 'mode', m.data),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.ser = None
        threading.Thread(target=self.reader, daemon=True).start()
        self.create_timer(0.02, self.tick)
        self.create_timer(0.1, self.report)
        self.create_timer(1.0, self.telemetry)

    def reader(self):
        parser = crsf.Parser()
        while rclpy.ok():
            try:
                if self.ser is None:
                    self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
                    self.get_logger().info(f'receiver on {self.port} at {self.baud}')
                data = self.ser.read(64)
            except (serial.SerialException, OSError) as e:
                self.get_logger().warn(f'{self.port}: {e}', throttle_duration_sec=30)
                self.ser = None
                time.sleep(2)
                continue
            now = time.monotonic()
            for ftype, payload in parser.feed(data):
                with self.lock:
                    if ftype == crsf.T_RC_CHANNELS and len(payload) >= 22:
                        self.ch, self.ch_t = crsf.unpack_channels(payload), now
                    elif ftype == crsf.T_LINK_STATS and len(payload) >= 10:
                        self.stats, self.stats_t = crsf.link_stats(payload), now

    def link_ok(self, now):
        # ELRS stops sending channels on failsafe; link stats alone don't count
        return self.ch is not None and now - self.ch_t < 0.5

    def on_switch(self, name, high):
        if self.switch.get(name) == high:
            return
        first = name not in self.switch
        self.switch[name] = high
        if first:
            return  # the position at start-up isn't a command
        if name == 'sentinel':
            client = self.sentinel if high else self.wake
            if client.service_is_ready():
                client.call_async(Trigger.Request())
        elif name == 'estop' and self.estop.service_is_ready():
            self.estop.call_async(SetBool.Request(data=high))
        self.get_logger().info(f'{name} {"on" if high else "off"}')

    def tick(self):
        now = time.monotonic()
        with self.lock:
            ch = list(self.ch) if self.link_ok(now) else None
        if ch is None:
            return
        n = [crsf.normalize(c) for c in ch]
        self.on_switch('sentinel', n[5] > 0.5)
        self.on_switch('estop', n[6] > 0.5)
        if n[4] <= 0.5:  # not armed
            return
        db = lambda v: 0.0 if abs(v) < self.deadband else v  # noqa: E731
        speed = 0.3 + 0.7 * (n[2] + 1) / 2
        t = Twist()
        t.linear.x = db(n[1]) * self.max_v * speed
        t.linear.y = -db(n[0]) * self.max_v * speed
        t.angular.z = -db(n[3]) * self.max_w * speed
        self.cmd_pub.publish(t)

    def report(self):
        now = time.monotonic()
        with self.lock:
            ok, s = self.link_ok(now), dict(self.stats) if now - self.stats_t < 1.0 else {}
            armed = ok and crsf.normalize(self.ch[4]) > 0.5
        self.link_pub.publish(UInt8(data=s.get('lq', 1) if ok else 0))
        d = DiagnosticStatus(name='links: elrs', hardware_id=self.port)
        if self.ser is None:
            d.level, d.message = DiagnosticStatus.WARN, 'no receiver port'
        elif not ok:
            d.level, d.message = DiagnosticStatus.WARN, 'no link'
        else:
            d.level = DiagnosticStatus.OK
            d.message = f'LQ {s.get("lq", "?")} %, {s.get("rssi", "?")} dBm' + (', armed' if armed else '')
            d.values = [KeyValue(key=k, value=str(v)) for k, v in s.items()]
        arr = DiagnosticArray(status=[d])
        arr.header.stamp = self.get_clock().now().to_msg()
        self.diag_pub.publish(arr)

    def telemetry(self):
        if self.ser is None:
            return
        out = crsf.flight_mode(self.mode.upper()[:14] or 'VECTOR')
        b = self.battery
        if b is not None:
            used = (1 - b.percentage) * b.capacity * 1000 if b.capacity == b.capacity and b.capacity > 0 else 0
            out += crsf.battery(b.voltage, max(0.0, -b.current), used, b.percentage * 100)
        try:
            self.ser.write(out)
        except (serial.SerialException, OSError):
            pass


def main():
    rclpy.init()
    rclpy.spin(Elrs())


if __name__ == '__main__':
    main()
