#!/usr/bin/env python3
"""Gazebo's one-ray scans on legs/<leg>/tof_scan -> sensor_msgs/Range on legs/<leg>/tof, as
leg_monitor publishes them on the robot: frame <leg>_tof, inf when nothing is in range."""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Range

LEGS = ('L1', 'L2', 'L3', 'R1', 'R2', 'R3')


class SimTof(Node):
    def __init__(self):
        super().__init__('sim_tof')
        self.fov = self.declare_parameter('field_of_view', 0.47).value  # VL53L1X, 27 deg
        self.pubs = {leg: self.create_publisher(Range, f'legs/{leg}/tof', qos_profile_sensor_data) for leg in LEGS}
        for leg in LEGS:
            self.create_subscription(LaserScan, f'legs/{leg}/tof_scan', lambda m, leg=leg: self.on_scan(leg, m),
                                     qos_profile_sensor_data)

    def on_scan(self, leg, scan):
        hits = [r for r in scan.ranges if math.isfinite(r) and scan.range_min <= r <= scan.range_max]
        msg = Range(radiation_type=Range.INFRARED, field_of_view=self.fov,
                    min_range=scan.range_min, max_range=scan.range_max, range=min(hits) if hits else math.inf)
        msg.header.stamp = scan.header.stamp
        msg.header.frame_id = f'{leg}_tof'
        self.pubs[leg].publish(msg)


def main():
    rclpy.init()
    rclpy.spin(SimTof())


if __name__ == '__main__':
    main()
