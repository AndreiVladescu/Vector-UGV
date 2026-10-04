#!/usr/bin/env python3
"""The carrier's IO MCU on UART2 (firmware/io-node) -> ROS.

Publishes scan (LaserScan from the LD19), gnss/nmea_sentence (for nmea_topic_driver), io/crsf
(the receiver's CRSF frames, for elrs.py with port:=io), lora/rx and the MCU's housekeeping
in /diagnostics. Takes io/crsf_out (telemetry to the receiver) and lora/tx. Services:
io/lte_power (SetBool), io/beep (Trigger, two short beeps), io/bootloader (Trigger: resets
the MCU into ST's ROM bootloader and lets go of the port for release_s, for io_flash.py).
Parameters lidar_pwm (0.1 %, 0 = the LD19's own 10 Hz) and beacon_s are sent again
whenever the MCU restarts.
"""
import struct
import threading
import time

import rclpy
import serial
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nmea_msgs.msg import Sentence
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import UInt8MultiArray
from std_srvs.srv import SetBool, Trigger

from vector_io import lidar, link


class IoBridge(Node):
    def __init__(self):
        super().__init__('io_bridge')
        p = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731
        self.port, self.baud = p('port', '/dev/ttyAMA2'), p('baud', 1000000)
        self.lidar_frame, self.gnss_frame = p('lidar_frame', 'lidar'), p('gnss_frame', 'gnss')
        self.turns = lidar.Turns(bins=p('lidar_bins', 450), yaw=p('lidar_yaw', 0.0))
        self.lidar_pwm, self.beacon_s = p('lidar_pwm', 0), p('beacon_s', 30)
        self.release_s = p('release_s', 120.0)
        self.add_on_set_parameters_callback(self.on_params)

        self.ser = None
        self.lock = threading.Lock()
        self.release_until = 0.0
        self.status, self.status_t, self.uptime = None, 0.0, None
        self.lora_last = None
        self.dec = link.Decoder()
        self.scan_pub = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self.nmea_pub = self.create_publisher(Sentence, 'gnss/nmea_sentence', 10)
        self.crsf_pub = self.create_publisher(UInt8MultiArray, 'io/crsf', 20)
        self.lora_pub = self.create_publisher(UInt8MultiArray, 'lora/rx', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_subscription(UInt8MultiArray, 'io/crsf_out', lambda m: self.send(link.CRSF_OUT, bytes(m.data)), 10)
        self.create_subscription(UInt8MultiArray, 'lora/tx', lambda m: self.send(link.LORA_SEND, bytes(m.data)), 10)
        self.create_service(SetBool, 'io/lte_power', self.lte_power)
        self.create_service(Trigger, 'io/beep', self.beep)
        self.create_service(Trigger, 'io/bootloader', self.bootloader)
        threading.Thread(target=self.reader, daemon=True).start()
        self.create_timer(1.0, self.report)

    # ---- to the MCU ----

    def send(self, mtype, payload=b''):
        with self.lock:
            if self.ser is None:
                return False
            try:
                self.ser.write(link.encode(mtype, payload))
                return True
            except (serial.SerialException, OSError, ValueError):
                return False

    def configure(self):
        self.send(link.LIDAR_PWM, struct.pack('<H', max(0, min(1000, self.lidar_pwm))))
        self.send(link.BEACON, struct.pack('<H', max(0, min(65535, self.beacon_s))))

    def on_params(self, params):
        for prm in params:
            if prm.name == 'lidar_pwm':
                self.lidar_pwm = prm.value
            elif prm.name == 'beacon_s':
                self.beacon_s = prm.value
        self.configure()
        return SetParametersResult(successful=True)

    def lte_power(self, req, res):
        res.success = self.send(link.LTE_POWER, bytes([1 if req.data else 0]))
        res.message = 'LTE supply ' + ('on' if req.data else 'off') if res.success else 'no IO MCU'
        return res

    def beep(self, req, res):
        res.success = self.send(link.BEEP, struct.pack('<HHHB', 2700, 80, 80, 2))
        return res

    def bootloader(self, req, res):
        res.success = self.send(link.BOOTLOADER, struct.pack('<I', link.BOOT_MAGIC))
        if res.success:
            time.sleep(0.05)
            with self.lock:
                self.ser.close()
                self.ser = None
                self.release_until = time.monotonic() + self.release_s
            res.message = f'ROM bootloader on {self.port}, port free for {self.release_s:.0f} s'
        else:
            res.message = 'no IO MCU'
        return res

    # ---- from the MCU ----

    def reader(self):
        while rclpy.ok():
            with self.lock:
                ser = self.ser
                if ser is None and time.monotonic() >= self.release_until:
                    try:
                        ser = self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
                        self.get_logger().info(f'IO MCU on {self.port}')
                    except (serial.SerialException, OSError) as e:
                        self.get_logger().warn(f'{self.port}: {e}', throttle_duration_sec=30)
            if ser is None:
                time.sleep(0.5)
                continue
            try:
                data = ser.read(512)
            except (serial.SerialException, OSError, TypeError):
                with self.lock:
                    if self.ser is ser:  # not closed on purpose
                        self.ser = None
                        self.get_logger().warn(f'{self.port} lost')
                time.sleep(0.5)
                continue
            for mtype, payload in self.dec.feed(data):
                self.on_msg(mtype, payload)

    def on_msg(self, mtype, p):
        stamp = self.get_clock().now().to_msg()
        if mtype == link.STATUS and len(p) >= 30:
            st = link.status(p)
            if self.uptime is None or st['uptime_s'] < self.uptime:
                self.get_logger().info(f'IO MCU {st["version"]} up, reset: {st["reset"]}')
                self.configure()
            self.uptime, self.status, self.status_t = st['uptime_s'], st, time.monotonic()
        elif mtype == link.CRSF:
            self.crsf_pub.publish(UInt8MultiArray(data=list(p)))
        elif mtype == link.LIDAR:
            turn = self.turns.feed(p)
            if turn:
                self.publish_scan(turn, stamp)
        elif mtype == link.NMEA:
            m = Sentence(sentence=p.decode('ascii', 'replace'))
            m.header.stamp, m.header.frame_id = stamp, self.gnss_frame
            self.nmea_pub.publish(m)
        elif mtype == link.LORA_RX and len(p) >= 3:
            rssi, snr = struct.unpack('<hb', p[:3])
            self.lora_last = (rssi, snr / 4, time.monotonic())
            self.lora_pub.publish(UInt8MultiArray(data=list(p[3:])))
        elif mtype == link.LORA_TX and p:
            if p[0]:
                self.get_logger().warn(f'LoRa packet not sent: {link.LORA_RESULT.get(p[0], p[0])}')

    def publish_scan(self, turn, stamp):
        ranges, intens, speed = turn
        n = len(ranges)
        m = LaserScan()
        m.header.stamp, m.header.frame_id = stamp, self.lidar_frame
        m.angle_min, m.angle_increment = 0.0, 2 * 3.141592653589793 / n
        m.angle_max = m.angle_min + m.angle_increment * (n - 1)
        m.scan_time = 360.0 / speed if speed else 0.1
        m.time_increment = m.scan_time / n
        m.range_min, m.range_max = self.turns.range_min, self.turns.range_max
        m.ranges, m.intensities = ranges, intens
        self.scan_pub.publish(m)

    def report(self):
        d = DiagnosticStatus(name='io: mcu', hardware_id=self.port)
        st = self.status if time.monotonic() - self.status_t < 1.0 else None
        if st is None:
            d.level = DiagnosticStatus.ERROR
            d.message = 'released for flashing' if time.monotonic() < self.release_until else 'no status'
        else:
            missing = [k for k in ('crsf', 'lidar', 'gnss', 'lora') if not st[k]]
            d.level = DiagnosticStatus.OK if not missing else DiagnosticStatus.WARN
            d.message = f'{st["vbat"]:.2f} V' + (', no ' + ', '.join(missing) if missing else '')
            d.values = [KeyValue(key=k, value=str(v)) for k, v in st.items()]
            if self.lora_last:
                d.values += [KeyValue(key='lora_rssi', value=str(self.lora_last[0])),
                             KeyValue(key='lora_snr', value=str(self.lora_last[1]))]
            d.values.append(KeyValue(key='bad_frames_here', value=str(self.dec.bad)))
        arr = DiagnosticArray(status=[d])
        arr.header.stamp = self.get_clock().now().to_msg()
        self.diag_pub.publish(arr)


def main():
    rclpy.init()
    rclpy.spin(IoBridge())


if __name__ == '__main__':
    main()
