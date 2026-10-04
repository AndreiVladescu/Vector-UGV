"""io_bridge.py and elrs.py (port:=io) with a fake IO MCU on a pseudo-terminal: status in
/diagnostics, lidar turns on /scan, NMEA, CRSF to cmd_vel and telemetry back, LoRa both
ways, the LTE supply and the bootloader hand-over."""
import os
import pty
import struct
import time
import tty
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nmea_msgs.msg import Sentence
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import UInt8MultiArray
from std_srvs.srv import SetBool, Trigger

from vector_io import lidar, link
from vector_link import crsf

MASTER, SLAVE = pty.openpty()
tty.setraw(MASTER)
tty.setraw(SLAVE)
PORT = os.ttyname(SLAVE)


@pytest.mark.launch_test
def generate_test_description():
    bridge = launch_ros.actions.Node(package='vector_io', executable='io_bridge.py', output='screen',
                                     parameters=[{'port': PORT, 'lidar_pwm': 400, 'release_s': 3.0}])
    radio = launch_ros.actions.Node(package='vector_link', executable='elrs.py', output='screen',
                                    parameters=[{'port': 'io'}])
    return launch.LaunchDescription([bridge, radio, launch_testing.actions.ReadyToTest()])


class TestIoBridge(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('io_test')
        cls.got = {'scan': [], 'nmea': [], 'cmd': [], 'lora': [], 'diag': []}
        n = cls.node
        n.create_subscription(LaserScan, '/scan', lambda m: cls.got['scan'].append(m), qos_profile_sensor_data)
        n.create_subscription(Sentence, '/gnss/nmea_sentence', lambda m: cls.got['nmea'].append(m), 10)
        n.create_subscription(Twist, '/cmd_vel/elrs', lambda m: cls.got['cmd'].append(m), 50)
        n.create_subscription(UInt8MultiArray, '/lora/rx', lambda m: cls.got['lora'].append(bytes(m.data)), 10)

        def diag(m):
            cls.got['diag'] += [s for s in m.status if s.name == 'io: mcu']
        n.create_subscription(DiagnosticArray, '/diagnostics', diag, 10)
        cls.lora_tx = n.create_publisher(UInt8MultiArray, '/lora/tx', 10)
        cls.dec = link.Decoder()
        cls.from_bridge = []
        cls.uptime = 1.0
        os.set_blocking(MASTER, False)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def mcu(self, mtype, payload=b''):
        data = link.encode(mtype, payload)
        end = time.time() + 2
        while data and time.time() < end:  # the pty buffer fills while the bridge isn't reading
            try:
                data = data[os.write(MASTER, data):]
            except BlockingIOError:
                time.sleep(0.005)

    def status(self):
        self.__class__.uptime += 0.1
        self.mcu(link.STATUS, link.pack_status(uptime_s=self.uptime, vbat=14.8, v5=5.0, temp=30.0, sats=8,
                                               fix_quality=1, crsf=True, lidar=True, gnss=True, fix=True,
                                               lora=True, host=True))

    def spin(self, seconds, until=None, every=None):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            try:
                self.from_bridge.extend(self.dec.feed(os.read(MASTER, 4096)))
            except (BlockingIOError, OSError):
                pass
            if every:
                every()
            if until and until():
                return True
        return False

    def sent(self, mtype):
        return [p for t, p in self.from_bridge if t == mtype]

    def test_1_status_and_config(self):
        # the bridge sends its settings when it first hears from the MCU
        self.assertTrue(self.spin(15, lambda: self.sent(link.BEACON), self.status))
        self.assertEqual(struct.unpack('<H', self.sent(link.LIDAR_PWM)[-1])[0], 400)
        self.assertEqual(struct.unpack('<H', self.sent(link.BEACON)[-1])[0], 30)
        self.assertTrue(self.spin(5, lambda: self.got['diag'] and self.got['diag'][-1].level == b'\x00', self.status))
        d = self.got['diag'][-1]
        self.assertIn('14.80 V', d.message)
        values = {kv.key: kv.value for kv in d.values}
        self.assertEqual(values['sats'], '8')
        # the parameter goes out at once
        cli = self.node.create_client(SetParameters, '/io_bridge/set_parameters')
        self.assertTrue(cli.wait_for_service(timeout_sec=5))
        fut = cli.call_async(SetParameters.Request(parameters=[Parameter('lidar_pwm', value=0).to_parameter_msg()]))
        self.assertTrue(self.spin(5, lambda: fut.done() and struct.unpack('<H', self.sent(link.LIDAR_PWM)[-1])[0] == 0))

    def test_2_scan(self):
        wall = lambda a: 2.0 if 30 <= a <= 60 else 0.0  # noqa: E731
        for _ in range(3):
            for k in range(45):
                a0 = k * 8.0
                self.mcu(link.LIDAR, lidar.packet(3600, a0, [wall((a0 + 8 / 11 * i) % 360) for i in range(12)],
                                                  end=(a0 + 8) % 360))
            self.spin(0.05)
        self.assertTrue(self.spin(5, lambda: self.got['scan']))
        s = self.got['scan'][-1]
        self.assertEqual(s.header.frame_id, 'lidar')
        self.assertEqual(len(s.ranges), 450)
        self.assertAlmostEqual(s.scan_time, 0.1)
        k = round(315 / 360 * 450)
        self.assertAlmostEqual(s.ranges[k], 2.0)
        self.assertEqual(s.ranges[round(45 / 360 * 450)], float('inf'))

    def test_3_nmea(self):
        body = 'GNGGA,101010.00,4426.16140,N,02606.16800,E,1,09,1.0,80.5,M,36.0,M,,'
        sum_ = 0
        for c in body:
            sum_ ^= ord(c)
        self.mcu(link.NMEA, f'${body}*{sum_:02X}'.encode())
        self.assertTrue(self.spin(5, lambda: self.got['nmea']))
        m = self.got['nmea'][-1]
        self.assertTrue(m.sentence.startswith('$GNGGA,101010.00'))
        self.assertEqual(m.header.frame_id, 'gnss')

    def test_4_crsf(self):
        ch = [crsf.CH_MID] * 16
        ch[1] = crsf.CH_MAX  # full forward
        ch[2] = crsf.CH_MAX
        ch[4] = crsf.CH_MIN
        start = crsf.frame(crsf.T_RC_CHANNELS, crsf.pack_channels(ch))
        ch[4] = crsf.CH_MAX  # armed
        armed = crsf.frame(crsf.T_RC_CHANNELS, crsf.pack_channels(ch))
        self.spin(0.5, every=lambda: self.mcu(link.CRSF, start))
        self.assertTrue(self.spin(5, lambda: any(c.linear.x > 0.05 for c in self.got['cmd']),
                                  lambda: self.mcu(link.CRSF, armed)))
        # telemetry back to the receiver: one CRSF frame per message, flight mode at least
        self.assertTrue(self.spin(3, lambda: any(p[2] == crsf.T_FLIGHT_MODE for p in self.sent(link.CRSF_OUT))))
        for p in self.sent(link.CRSF_OUT):
            self.assertEqual(p[0], crsf.SYNC)
            self.assertEqual(len(p), p[1] + 2)

    def test_5_lora(self):
        self.lora_tx.publish(UInt8MultiArray(data=list(b'hello base')))
        self.assertTrue(self.spin(5, lambda: b'hello base' in self.sent(link.LORA_SEND)))
        self.mcu(link.LORA_RX, struct.pack('<hb', -97, -22) + b'ping')
        self.assertTrue(self.spin(5, lambda: b'ping' in self.got['lora']))

    def test_6_lte_and_beep(self):
        for name, srv, req in (('io/lte_power', SetBool, SetBool.Request(data=True)), ('io/beep', Trigger, Trigger.Request())):
            cli = self.node.create_client(srv, name)
            self.assertTrue(cli.wait_for_service(timeout_sec=5))
            fut = cli.call_async(req)
            self.assertTrue(self.spin(5, fut.done))
            self.assertTrue(fut.result().success)
        self.assertTrue(self.spin(2, lambda: self.sent(link.LTE_POWER) and self.sent(link.BEEP)))
        self.assertEqual(self.sent(link.LTE_POWER)[-1], b'\x01')

    def test_7_bootloader(self):
        cli = self.node.create_client(Trigger, 'io/bootloader')
        self.assertTrue(cli.wait_for_service(timeout_sec=5))
        fut = cli.call_async(Trigger.Request())
        self.assertTrue(self.spin(5, fut.done))
        self.assertTrue(fut.result().success)
        self.assertTrue(self.spin(2, lambda: self.sent(link.BOOTLOADER)))
        self.assertEqual(struct.unpack('<I', self.sent(link.BOOTLOADER)[-1])[0], link.BOOT_MAGIC)
        self.assertTrue(self.spin(4, lambda: self.got['diag'] and 'flashing' in self.got['diag'][-1].message))
        # after release_s the bridge takes the port back, and the restarted MCU gets the settings again
        self.spin(3.5)
        self.__class__.uptime = 0.0
        n = len(self.sent(link.BEACON))
        self.assertTrue(self.spin(10, lambda: len(self.sent(link.BEACON)) > n, self.status))
