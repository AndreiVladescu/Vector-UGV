#!/usr/bin/env python3
"""LSM6DSV16X on the carrier and MMC5983MA on the nose board, both on I2C1 -> imu
(sensor_msgs/Imu, orientation in ENU: yaw 0 = east, what navsat_transform expects) and mag
(sensor_msgs/MagneticField). Without the compass the yaw only comes from the gyro, and its
covariance says so.

The gyro bias is averaged over the first bias_s seconds (keep the robot still). imu_axes and
mag_axes map the chips' axes onto base_link, e.g. "-y,x,z" for a chip turned 90 degrees.
imu/calibrate_mag (Trigger): turn the robot through every orientation for cal_s seconds; the
hard- and soft-iron correction lands in mag_offset / mag_scale (logged, for the yaml).
bus:=sim runs on simulated chips, turning at sim_turn rad/s about z, or with the legs'
odometry (odom/legs) when that is published, so the demo's heading follows the driving.
"""
import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Quaternion
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, MagneticField
from std_srvs.srv import Trigger

from vector_imu import fusion, lsm6dsv, mmc5983, sim


def axes_map(spec):
    """"-y,x,z" -> function mapping a chip vector onto the body"""
    idx = {'x': 0, 'y': 1, 'z': 2}
    parts = [p.strip() for p in spec.split(',')]
    if len(parts) != 3 or sorted(p[-1] for p in parts) != ['x', 'y', 'z']:
        raise ValueError(f'bad axes "{spec}"')
    m = [(idx[p[-1]], -1.0 if p.startswith('-') else 1.0) for p in parts]
    return lambda v: tuple(s * v[i] for i, s in m)


class ImuNode(Node):
    def __init__(self):
        super().__init__('imu')
        p = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731
        self.bus_name = p('bus', '/dev/i2c-1')
        self.frame = p('frame_id', 'base_link')
        self.rate = p('rate', 100.0)
        self.imu_axes, self.mag_axes = axes_map(p('imu_axes', 'x,y,z')), axes_map(p('mag_axes', 'x,y,z'))
        self.declare_parameter('mag_offset', [0.0, 0.0, 0.0])
        self.declare_parameter('mag_scale', [1.0, 1.0, 1.0])
        self.fusion = fusion.Fusion(gain=p('gain', 1.0), declination=math.radians(p('declination_deg', 6.0)))
        self.bias_s, self.cal_s = p('bias_s', 2.0), p('cal_s', 30.0)
        self.sim_turn = p('sim_turn', 0.0)

        if self.bus_name == 'sim':
            self.bus = sim.turning(self.sim_turn)
            from nav_msgs.msg import Odometry
            self.create_subscription(Odometry, 'odom/legs', lambda m: setattr(self.bus, 'rate', m.twist.twist.angular.z), 10)
        else:
            from vector_imu.i2c import Bus
            self.bus = Bus(self.bus_name)
        self.imu = lsm6dsv.Lsm6dsv(self.bus)
        self.mag = mmc5983.Mmc5983(self.bus)
        self.imu_ok = self.mag_ok = False
        self.imu_errors = self.mag_errors = 0
        self.retry_t = 0.0
        self.bias, self.bias_sum, self.bias_n, self.t0 = (0.0, 0.0, 0.0), [0.0, 0.0, 0.0], 0, time.monotonic()
        self.field = None
        self.field_t = 0.0
        self.temp = None
        self.cal = None
        self.tick_n = 0
        self.last = None

        self.imu_pub = self.create_publisher(Imu, 'imu', qos_profile_sensor_data)
        self.mag_pub = self.create_publisher(MagneticField, 'mag', qos_profile_sensor_data)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_service(Trigger, 'imu/calibrate_mag', self.calibrate)
        self.create_timer(1.0 / self.rate, self.tick)
        self.create_timer(1.0, self.report)

    def start_parts(self, now):
        self.retry_t = now + 2.0
        for name, part in (('imu', self.imu), ('mag', self.mag)):
            if getattr(self, name + '_ok'):
                continue
            try:
                part.start()
                setattr(self, name + '_ok', True)
                if name == 'imu' and self.bias_n >= 0:
                    self.t0, self.bias_sum, self.bias_n = now, [0.0, 0.0, 0.0], 0  # the bias window starts now
                self.get_logger().info(f'{type(part).__name__} up')
            except OSError as e:
                self.get_logger().warn(f'{type(part).__name__}: {e}', throttle_duration_sec=60)

    def tick(self):
        now = time.monotonic()
        if (not self.imu_ok or not self.mag_ok) and now >= self.retry_t:
            self.start_parts(now)
        if hasattr(self.bus, 'step'):
            self.bus.step(1.0 / self.rate)
        self.tick_n += 1
        if self.mag_ok and self.tick_n % 2 == 0:  # the compass runs at 50 Hz
            try:
                self.read_mag(now)
            except OSError:
                self.mag_errors += 1
                self.mag_ok = False
        if not self.imu_ok:
            return
        try:
            gyro, accel, self.temp = self.imu.read()
        except OSError:
            self.imu_errors += 1
            self.imu_ok = False
            return
        gyro, accel = self.imu_axes(gyro), self.imu_axes(accel)
        if self.bias_n >= 0 and now - self.t0 < self.bias_s:
            self.bias_sum = [s + g for s, g in zip(self.bias_sum, gyro)]
            self.bias_n += 1
        elif self.bias_n > 0:
            self.bias = tuple(s / self.bias_n for s in self.bias_sum)
            self.bias_n = -1
            self.get_logger().info('gyro bias ' + ', '.join(f'{b * 1000:.1f}' for b in self.bias) + ' mrad/s')
        gyro = tuple(g - b for g, b in zip(gyro, self.bias))
        dt = 1.0 / self.rate if self.last is None else min(0.1, now - self.last)
        self.last = now
        field = self.field if self.field is not None and now - self.field_t < 0.2 else None
        q = self.fusion.update(gyro, accel, field, dt)
        self.publish(q, gyro, accel, field is not None)

    def read_mag(self, now):
        raw = self.mag_axes(self.mag.read())
        if self.cal is not None:
            lo, hi, t_end = self.cal
            self.cal = ([min(a, b) for a, b in zip(lo, raw)], [max(a, b) for a, b in zip(hi, raw)], t_end)
            if now >= t_end:
                self.finish_calibration()
        off = self.get_parameter('mag_offset').value
        scale = self.get_parameter('mag_scale').value
        self.field = tuple((r - o) * s for r, o, s in zip(raw, off, scale))
        self.field_t = now
        m = MagneticField()
        m.header.stamp, m.header.frame_id = self.get_clock().now().to_msg(), self.frame
        m.magnetic_field.x, m.magnetic_field.y, m.magnetic_field.z = self.field
        m.magnetic_field_covariance = [1e-12, 0.0, 0.0, 0.0, 1e-12, 0.0, 0.0, 0.0, 1e-12]
        self.mag_pub.publish(m)

    def publish(self, q, gyro, accel, with_compass):
        m = Imu()
        m.header.stamp, m.header.frame_id = self.get_clock().now().to_msg(), self.frame
        m.orientation = Quaternion(w=q[0], x=q[1], y=q[2], z=q[3])
        yaw_var = 0.01 if with_compass else 1e3  # about 6 degrees, or none at all
        m.orientation_covariance = [3e-4, 0.0, 0.0, 0.0, 3e-4, 0.0, 0.0, 0.0, yaw_var]
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = gyro
        m.angular_velocity_covariance = [1e-5, 0.0, 0.0, 0.0, 1e-5, 0.0, 0.0, 0.0, 1e-5]
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = accel
        m.linear_acceleration_covariance = [4e-3, 0.0, 0.0, 0.0, 4e-3, 0.0, 0.0, 0.0, 4e-3]
        self.imu_pub.publish(m)

    def calibrate(self, req, res):
        if not self.mag_ok:
            res.success, res.message = False, 'no compass'
            return res
        self.cal = ([math.inf] * 3, [-math.inf] * 3, time.monotonic() + self.cal_s)
        res.success, res.message = True, f'turn the robot through every orientation for {self.cal_s:.0f} s'
        return res

    def finish_calibration(self):
        lo, hi, _ = self.cal
        self.cal = None
        half = [(h - lo_) / 2 for lo_, h in zip(lo, hi)]
        if min(half) <= 5e-6:
            self.get_logger().warn('compass calibration: not turned enough, nothing changed')
            return
        off = [(h + lo_) / 2 for lo_, h in zip(lo, hi)]
        avg = sum(half) / 3
        scale = [avg / h for h in half]
        self.set_parameters([Parameter('mag_offset', value=off), Parameter('mag_scale', value=scale)])
        self.get_logger().info(f'compass calibration: mag_offset: [{", ".join(f"{o:.3e}" for o in off)}], '
                               f'mag_scale: [{", ".join(f"{s:.3f}" for s in scale)}]')

    def report(self):
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        d = DiagnosticStatus(name='imu: lsm6dsv16x', hardware_id=self.bus_name)
        if self.imu_ok:
            r, pch, y = fusion.euler(self.fusion.q) if self.fusion.q else (0.0, 0.0, 0.0)
            d.level = DiagnosticStatus.OK
            d.message = f'roll {math.degrees(r):.1f}, pitch {math.degrees(pch):.1f}, yaw {math.degrees(y):.1f}'
            d.values = [KeyValue(key='temperature', value=f'{self.temp:.1f}'),
                        KeyValue(key='bias_done', value=str(self.bias_n < 0))]
        else:
            d.level, d.message = DiagnosticStatus.ERROR, 'not answering'
        d.values.append(KeyValue(key='errors', value=str(self.imu_errors)))
        c = DiagnosticStatus(name='imu: compass', hardware_id=self.bus_name)
        if self.mag_ok and self.field is not None:
            c.level = DiagnosticStatus.OK
            c.message = f'{math.sqrt(sum(v * v for v in self.field)) * 1e6:.1f} uT' + (', calibrating' if self.cal else '')
        else:
            c.level, c.message = DiagnosticStatus.WARN, 'no compass: yaw from the gyro only'
        c.values = [KeyValue(key='errors', value=str(self.mag_errors))]
        arr.status = [d, c]
        self.diag_pub.publish(arr)


def main():
    rclpy.init()
    rclpy.spin(ImuNode())


if __name__ == '__main__':
    main()
