#!/usr/bin/env python3
"""A simulated IO MCU on a pseudo-terminal, for trying the operator page and io_bridge without
the carrier. It speaks the real link protocol (vector_io/link.py) on `link` (a symlink to the
pty, default /tmp/vector-io-sim) and follows the robot's own motion (odom/legs):

  GNSS   GGA and RMC at 1 Hz from a start point, moved by the leg odometry (gnss_scale to
         exaggerate it); satellites wander between 7 and 12
  lidar  LDS01RR (XV-11) packets of a rectangular yard around the start point; the motor
         spins at the rpm the bridge asks for (0 = stopped, no packets)
  ELRS   RC channels at 50 Hz (sticks centred, not armed) and link statistics with a wandering LQ
  LoRa   a packet "base: ping N" every 20 s, LORA_TX answers, the beacon count
  status battery voltage draining with the walking current, 5 V rail, board temperature

It also publishes battery (BatteryState) from the same voltage, standing in for the power board.
Break things live, e.g.  ros2 param set /sim_io gnss false  (radio, lidar, gnss, lora too),
ros2 param set /sim_io vbat 13.4,  ros2 param set /sim_io gnss_scale 20.
Commands from the bridge (beep, LTE power, LoRa send, lidar speed, bootloader) are logged.
"""
import math
import os
import pty
import random
import struct
import time
import tty

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import BatteryState

from vector_io import lidar, link

CRSF_SYNC, CRSF_RC, CRSF_STATS = 0xC8, 0x16, 0x14


def crsf_frame(ftype, payload):
    body = bytes([ftype]) + payload
    crc = 0
    for b in body:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return bytes([CRSF_SYNC, len(body) + 1]) + body + bytes([crc])


def nmea(body):
    s = 0
    for c in body:
        s ^= ord(c)
    return f'${body}*{s:02X}'.encode()


def ddmm(deg, lat):
    a = abs(deg)
    d = int(a)
    m = (a - d) * 60
    return (f'{d:02d}{m:08.5f}', 'N' if deg >= 0 else 'S') if lat else (f'{d:03d}{m:08.5f}', 'E' if deg >= 0 else 'W')


class SimIo(Node):
    def __init__(self):
        super().__init__('sim_io')
        p = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731
        self.path = p('link', '/tmp/vector-io-sim')
        self.lat0, self.lon0 = p('lat', 44.4268), p('lon', 26.1025)
        for name, default in (('radio', True), ('lidar', True), ('gnss', True), ('lora', True),
                              ('vbat', 15.6), ('gnss_scale', 1.0), ('yard_x', 6.0), ('yard_y', 4.0)):
            self.declare_parameter(name, default)
        # start facing what the simulated compass says (imu_node sim: magnetic east, minus the declination)
        self.yaw = -math.radians(p('declination_deg', 6.0))
        self.x = self.y = 0.0
        self.v = (0.0, 0.0, 0.0)
        self.t0 = time.monotonic()
        self.lte_en, self.lte_on_at = False, None
        self.beacon_s, self.beacons, self.next_beacon = 30, 0, time.monotonic() + 30
        self.lidar_rpm, self.lidar_index, self.lidar_due = 0, 0, 0.0
        self.lq, self.sats, self.ping_n, self.next_ping = 99.0, 9, 0, time.monotonic() + 10
        self.silent_until = 0.0
        self.last_t = time.monotonic()

        self.master, self.slave = pty.openpty()
        tty.setraw(self.master)
        tty.setraw(self.slave)
        os.set_blocking(self.master, False)
        if os.path.islink(self.path):
            os.unlink(self.path)
        os.symlink(os.ttyname(self.slave), self.path)
        self.get_logger().info(f'IO MCU simulated on {os.ttyname(self.slave)}, linked at {self.path}')
        self.dec = link.Decoder()

        self.battery_pub = self.create_publisher(BatteryState, 'battery', 10)
        self.create_subscription(Odometry, 'odom/legs', self.on_odom, 10)
        self.create_timer(0.01, self.read)
        self.create_timer(0.02, self.fast)      # motion, CRSF, lidar
        self.create_timer(0.1, self.status)
        self.create_timer(1.0, self.slow)       # GNSS, battery, LoRa

    def prm(self, name):
        return self.get_parameter(name).value

    def send(self, mtype, payload=b''):
        if time.monotonic() < self.silent_until:
            return
        try:
            os.write(self.master, link.encode(mtype, payload))
        except (BlockingIOError, OSError):
            pass  # nobody reading: the bridge isn't up, the buffer is full

    def on_odom(self, m):
        t = m.twist.twist
        self.v = (t.linear.x, t.linear.y, t.angular.z)

    # ---- commands from io_bridge ----

    def read(self):
        try:
            data = os.read(self.master, 4096)
        except (BlockingIOError, OSError):
            return
        log = self.get_logger()
        for mtype, p in self.dec.feed(data):
            if mtype == link.BEEP and len(p) >= 7:
                hz, on, off, n = struct.unpack('<HHHB', p[:7])
                log.info(f'BEEP x{n} at {hz} Hz')
            elif mtype == link.LTE_POWER and p:
                self.lte_en = bool(p[0])
                self.lte_on_at = time.monotonic() + 3.0 if self.lte_en else None
                log.info(f'LTE supply {"on" if self.lte_en else "off"}')
            elif mtype == link.LORA_SEND:
                ok = self.prm('lora')
                self.send(link.LORA_TX, bytes([0 if ok else 3]))
                log.info(f'LoRa out: {p!r}' + ('' if ok else ' (no radio)'))
            elif mtype == link.LIDAR_RPM and len(p) >= 2:
                self.lidar_rpm = struct.unpack('<H', p[:2])[0]
                log.info(f'lidar motor {self.lidar_rpm} rpm')
            elif mtype == link.BEACON and len(p) >= 2:
                self.beacon_s = struct.unpack('<H', p[:2])[0]
            elif mtype == link.BOOTLOADER:
                log.warn('bootloader requested: silent for 5 s, then a fresh start')
                self.silent_until = time.monotonic() + 5.0
                self.t0 = self.silent_until

    # ---- what the MCU would send ----

    def fast(self):
        now = time.monotonic()
        dt, self.last_t = now - self.last_t, now
        vx, vy, wz = self.v
        self.x += (vx * math.cos(self.yaw) - vy * math.sin(self.yaw)) * dt
        self.y += (vx * math.sin(self.yaw) + vy * math.cos(self.yaw)) * dt
        self.yaw += wz * dt

        if self.prm('radio'):
            ch = [992] * 16
            ch[2], ch[4] = 172, 172  # throttle low, not armed
            v = 0
            for i, c in enumerate(ch):
                v |= c << (11 * i)
            self.send(link.CRSF, crsf_frame(CRSF_RC, v.to_bytes(22, 'little')))
            if random.random() < 0.1:
                self.lq = max(40.0, min(100.0, self.lq + random.uniform(-3, 3) + (95 - self.lq) * 0.1))
                rssi = int(60 + (100 - self.lq) * 0.6)
                stats = struct.pack('<BBBbBBBBBb', rssi, rssi + 3, int(self.lq), 9, 0, 4, 2, rssi + 5, 100, 7)
                self.send(link.CRSF, crsf_frame(CRSF_STATS, stats))

        # 90 packets a turn at the asked speed, by the clock: the timer runs a little late
        self.lidar_due = min(self.lidar_due + self.lidar_rpm * 90 / 60 * dt, 90)
        while self.lidar_due >= 1:
            self.lidar_due -= 1
            if self.prm('lidar'):
                self.send(link.LIDAR, self.lidar_packet())

    def lidar_packet(self):
        index = self.lidar_index
        self.lidar_index = (index + 1) % 90
        hx, hy = self.prm('yard_x') / 2, self.prm('yard_y') / 2
        ranges = []
        for j in range(4):
            ang = self.yaw + math.radians(4 * index + j)  # counter-clockwise, as Turns expects by default
            dx, dy = math.cos(ang), math.sin(ang)
            ts = [t for t in ((hx - self.x) / dx if dx > 1e-9 else None, (-hx - self.x) / dx if dx < -1e-9 else None,
                              (hy - self.y) / dy if dy > 1e-9 else None, (-hy - self.y) / dy if dy < -1e-9 else None)
                  if t is not None and t > 0]
            r = min(ts) if ts else 0.0
            ranges.append(r + random.gauss(0, 0.01) if 0.15 < r < 6 else 0.0)
        return lidar.packet(self.lidar_rpm + random.uniform(-2, 2), index, ranges)

    def status(self):
        now = time.monotonic()
        v = self.prm('vbat')
        st = link.pack_status(
            version=0x5e62b48, uptime_s=max(0.0, now - self.t0), reset=1, vbat=v, v5=5.02 + random.uniform(-0.01, 0.01),
            temp=31.0 + 4 * math.hypot(self.v[0], self.v[1]) / 0.1, sats=self.sats if self.prm('gnss') else 0,
            fix_quality=1 if self.prm('gnss') else 0, beacons=self.beacons, lte_en=self.lte_en,
            lte_status=bool(self.lte_on_at and now > self.lte_on_at), crsf=self.prm('radio'), lidar=self.prm('lidar'),
            gnss=self.prm('gnss'), fix=self.prm('gnss'), lora=self.prm('lora'), host=True)
        self.send(link.STATUS, st)

    def slow(self):
        now = time.monotonic()
        # the battery: 4S, about 2 A standing and 4 A walking; ~0.2 mV a second
        walking = math.hypot(self.v[0], self.v[1]) > 0.003 or abs(self.v[2]) > 0.02
        amps = 4.0 if walking else 2.0
        v = max(12.0, self.prm('vbat') - amps * 0.00005)
        self.set_parameters([Parameter('vbat', value=v)])
        b = BatteryState(voltage=v, current=-amps, percentage=max(0.0, min(1.0, (v - 13.2) / 3.6)),
                         power_supply_status=BatteryState.POWER_SUPPLY_STATUS_DISCHARGING, present=True,
                         capacity=7.5, design_capacity=7.5)  # 4S3P of 2.5 Ah cells
        b.header.stamp = self.get_clock().now().to_msg()
        self.battery_pub.publish(b)

        if self.prm('gnss'):
            self.sats = max(7, min(12, self.sats + random.choice((-1, 0, 0, 0, 1))))
            k = self.prm('gnss_scale')
            lat = self.lat0 + (self.y * k + random.gauss(0, 0.3)) / 111320
            lon = self.lon0 + (self.x * k + random.gauss(0, 0.3)) / (111320 * math.cos(math.radians(self.lat0)))
            (la, ns), (lo, ew) = ddmm(lat, True), ddmm(lon, False)
            utc = time.strftime('%H%M%S.00', time.gmtime())
            self.send(link.NMEA, nmea(f'GNGGA,{utc},{la},{ns},{lo},{ew},1,{self.sats:02d},0.9,81.5,M,36.0,M,,'))
            knots = math.hypot(self.v[0], self.v[1]) * self.prm('gnss_scale') * 1.943844
            course = (90 - math.degrees(self.yaw)) % 360
            date = time.strftime('%d%m%y', time.gmtime())
            self.send(link.NMEA, nmea(f'GNRMC,{utc},A,{la},{ns},{lo},{ew},{knots:.3f},{course:.1f},{date},,,A'))
        else:
            self.send(link.NMEA, nmea('GNGGA,,,,,,0,00,99.99,,,,,,'))

        if self.prm('lora'):
            if self.beacon_s and now >= self.next_beacon:
                self.beacons += 1
                self.next_beacon = now + self.beacon_s
            if now >= self.next_ping:
                self.next_ping = now + 20
                self.ping_n += 1
                rssi, snr = int(random.uniform(-112, -88)), int(random.uniform(-6, 9) * 4)
                self.send(link.LORA_RX, struct.pack('<hb', rssi, snr) + f'base: ping {self.ping_n}'.encode())

    def destroy_node(self):
        if os.path.islink(self.path):
            os.unlink(self.path)
        super().destroy_node()


def main():
    rclpy.init()
    node = SimIo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
