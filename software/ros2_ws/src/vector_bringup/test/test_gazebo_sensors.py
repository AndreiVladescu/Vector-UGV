"""The simulated LDS01RR and leg ToF rays: a wall put 1 m ahead shows up on /scan and in the
obstacle map's front sector, and standing on flat ground every leg ray sees the ground."""
import math
import os
import subprocess
import sys

import pytest
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import Float32MultiArray

sys.path.insert(0, os.path.dirname(__file__))
from gz_helpers import gazebo_launch, GazeboTest  # noqa: E402

LEGS = ('L1', 'L2', 'L3', 'R1', 'R2', 'R3')
WALL = ("<sdf version='1.9'><model name='wall'><static>true</static><pose>1.0 0 0.25 0 0 0</pose><link name='l'>"
        "<collision name='c'><geometry><box><size>0.1 1.0 0.5</size></box></geometry></collision>"
        "<visual name='v'><geometry><box><size>0.1 1.0 0.5</size></box></geometry></visual></link></model></sdf>")


@pytest.mark.launch_test
def generate_test_description():
    return gazebo_launch(world='flat', nav='true')


class TestSensors(GazeboTest):

    def test_lidar_tof_and_obstacles(self):
        scan, tof, sectors = [], {}, []
        self.node.create_subscription(LaserScan, '/scan', scan.append, qos_profile_sensor_data)
        for leg in LEGS:
            self.node.create_subscription(Range, f'/legs/{leg}/tof', lambda m, leg=leg: tof.update({leg: m}),
                                          qos_profile_sensor_data)
        self.node.create_subscription(Float32MultiArray, '/obstacles/sectors', lambda m: sectors.append(list(m.data)), 10)
        self.wait_for_sim()

        # every leg ray, standing, meets the flat ground ~37 cm out (10 deg down from ~6 cm)
        self.spin_for(2.0)
        self.assertEqual(set(tof), set(LEGS), 'leg ToF missing')
        print('\n--- gazebo sensors ---')
        print('leg ToF on flat ground: ' + ', '.join(f'{k} {v.range:.2f} m' for k, v in sorted(tof.items())))
        for leg, r in tof.items():
            self.assertEqual(r.header.frame_id, f'{leg}_tof')
            self.assertTrue(0.25 < r.range < 0.6, f'{leg}: {r.range}')

        # a wall 1 m ahead of the body centre
        env = dict(os.environ, GZ_PARTITION=f'vector_test_{os.getpid()}')
        r = subprocess.run(['gz', 'service', '-s', '/world/flat/create', '--reqtype', 'gz.msgs.EntityFactory',
                            '--reptype', 'gz.msgs.Boolean', '--timeout', '5000', '--req', f'sdf: "{WALL}"'],
                           env=env, capture_output=True, text=True, timeout=20)
        self.assertIn('true', r.stdout, r.stdout + r.stderr)
        scan.clear()
        self.spin_for(3.0)
        self.assertTrue(scan, 'no /scan')
        s = scan[-1]
        self.assertEqual(s.header.frame_id, 'lidar')
        self.assertEqual(len(s.ranges), 360)
        ahead = [r for i, r in enumerate(s.ranges)
                 if abs(s.angle_min + i * s.angle_increment) < math.radians(5) and math.isfinite(r)]
        print(f'lidar straight ahead: {min(ahead):.2f} m' if ahead else 'lidar: nothing ahead')
        self.assertTrue(ahead and 0.85 < min(ahead) < 1.05, ahead)
        self.spin_for(2.0)
        self.assertTrue(sectors, 'no obstacles/sectors')
        print(f'obstacle map, ahead: {sectors[-1][0]:.2f} m')
        self.assertTrue(0.85 < sectors[-1][0] < 1.05, sectors[-1])
