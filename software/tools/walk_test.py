#!/usr/bin/env python3
"""Drive the robot through a fixed routine and watch the legs while it runs.

  walk_test.py            # one cycle: forward, back, strafe, turn, sentinel, wake
  walk_test.py 0          # repeat until Ctrl-C (for the soak run)

Each cycle prints the time spent in sentinel/wake, the worst joint_states gap and any leg that
left the active state or reported a fault. Exits non-zero if anything went wrong.
"""
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

ROUTINE = [  # (name, vx, vy, wz, seconds)
    ('forward', 0.06, 0.0, 0.0, 15),
    ('back', -0.04, 0.0, 0.0, 8),
    ('strafe', 0.0, 0.04, 0.0, 8),
    ('turn', 0.0, 0.0, 0.4, 10),
    ('arc', 0.05, 0.0, -0.3, 10),
    ('stand', 0.0, 0.0, 0.0, 3),
]


class WalkTest(Node):
    def __init__(self):
        super().__init__('walk_test')
        self.cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sentinel = self.create_client(Trigger, 'gait_node/sentinel')
        self.wake = self.create_client(Trigger, 'gait_node/wake')
        self.mode = None
        self.create_subscription(String, 'gait_node/mode', self.on_mode,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(DiagnosticArray, '/diagnostics', self.on_diag, 10)
        self.create_subscription(JointState, 'joint_states', self.on_joints, 10)
        self.reset()

    def reset(self):
        self.problems = set()
        self.joint_count = 0
        self.joint_gap = 0.0
        self.last_joint = None
        self.diag_count = 0

    def on_mode(self, msg):
        self.mode = msg.data

    def on_joints(self, _):
        t = time.monotonic()
        if self.last_joint is not None:
            self.joint_gap = max(self.joint_gap, t - self.last_joint)
        self.last_joint = t
        self.joint_count += 1

    def on_diag(self, msg):
        self.diag_count += 1
        powered = self.mode in ('walk', 'stopping', 'lowering', 'raising')
        for s in msg.status:
            if not s.name.startswith('legs: '):
                continue
            if s.level != DiagnosticStatus.OK or (powered and s.message != 'active'):
                self.problems.add(f'{s.name[6:]} {s.message}')

    def spin_for(self, seconds, twist=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if twist:
                self.cmd.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_mode(self, mode, timeout):
        start = time.monotonic()
        while self.mode != mode:
            if time.monotonic() - start > timeout:
                return None
            rclpy.spin_once(self, timeout_sec=0.05)
        return time.monotonic() - start

    def call(self, client):
        client.wait_for_service(timeout_sec=5)
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5)
        res = future.result()
        return res is not None and res.success

    def cycle(self):
        self.reset()
        start = time.monotonic()
        for _, vx, vy, wz, secs in ROUTINE:
            t = Twist()
            t.linear.x, t.linear.y, t.angular.z = vx, vy, wz
            self.spin_for(secs, t)
        self.cmd.publish(Twist())
        walked = time.monotonic() - start

        sit = self.call(self.sentinel) and self.wait_mode('sentinel', 30)
        self.spin_for(3)
        up = self.call(self.wake) and self.wait_mode('walk', 30)
        rate = self.joint_count / (time.monotonic() - start)
        ok = bool(sit and up and not self.problems and self.joint_gap < 0.1 and self.diag_count)
        print(f'walk {walked:.0f} s, sentinel {sit or 0:.1f} s, wake {up or 0:.1f} s, '
              f'joint_states {rate:.0f} Hz max gap {self.joint_gap * 1000:.0f} ms, '
              f'{"ok" if ok else "FAIL"}'
              + ('' if not self.problems else ', legs: ' + ', '.join(sorted(self.problems))), flush=True)
        return ok


def main():
    cycles = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    rclpy.init()
    node = WalkTest()
    if node.wait_mode('walk', 30) is None:
        sys.exit(f'robot not walking (mode {node.mode})')
    failed = done = 0
    try:
        while cycles == 0 or done < cycles:
            failed += not node.cycle()
            done += 1
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd.publish(Twist())
    print(f'{done} cycles, {failed} failed')
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
