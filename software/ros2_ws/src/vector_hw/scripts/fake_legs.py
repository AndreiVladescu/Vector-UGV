#!/usr/bin/env python3
"""Six simulated leg nodes on a CAN interface, speaking protocol/vector.dbc.

Stands in for the leg firmware until the boards exist:
  sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
  ros2 run vector_hw fake_legs.py --channel vcan0

Each leg follows LEG_CMD at the servo speed limit, reports LEG_STATE at the bus rate and
LEG_STATUS at a quarter of it. If SYNC stops for longer than the watchdog time the legs
flag a fault and hold (the real firmware crouches and cuts servo power).
"""
import argparse
import math
import os
import random
import time

import can
import cantools

LEGS = ['L1', 'L2', 'L3', 'R1', 'R2', 'R3']
JOINTS = ['coxa', 'femur', 'tibia']
FAULT_WATCHDOG = 0x01


def default_dbc():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('vector_hw'), 'vector.dbc')
    except Exception:
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(here, '..', '..', '..', '..', '..', 'protocol', 'vector.dbc')


class Leg:
    def __init__(self, name, start):
        self.name = name
        self.pos = list(start)
        self.target = list(start)
        self.enabled = False
        self.faults = 0

    def step(self, dt, max_speed):
        moved = 0.0
        for j in range(3):
            err = self.target[j] - self.pos[j]
            step = max(-max_speed * dt, min(max_speed * dt, err))
            self.pos[j] += step
            moved += abs(step)
        return moved / dt if dt > 0 else 0.0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--channel', default='vcan0')
    ap.add_argument('--dbc', default=default_dbc())
    ap.add_argument('--rate', type=float, default=200.0, help='state frames per second per leg')
    ap.add_argument('--speed', type=float, default=math.degrees(7.0), help='max joint speed, deg/s')
    ap.add_argument('--watchdog', type=float, default=0.2, help='s without SYNC before faulting')
    ap.add_argument('--start', type=float, nargs=3, default=[0.0, 14.46, -103.25],
                    metavar=('COXA', 'FEMUR', 'TIBIA'), help='initial joint angles, deg')
    ap.add_argument('--quiet', action='store_true')
    args = ap.parse_args()

    db = cantools.database.load_file(args.dbc)
    bus = can.Bus(interface='socketcan', channel=args.channel)
    legs = [Leg(n, args.start) for n in LEGS]
    cmd_msgs = {db.get_message_by_name(f'LEG_{n}_CMD').frame_id: i for i, n in enumerate(LEGS)}
    state_msgs = [db.get_message_by_name(f'LEG_{n}_STATE') for n in LEGS]
    status_msgs = [db.get_message_by_name(f'LEG_{n}_STATUS') for n in LEGS]
    sync_id = db.get_message_by_name('SYNC').frame_id

    period = 1.0 / args.rate
    next_tick = time.monotonic()
    last_sync = None
    tick = 0
    stats = {'sync': 0, 'cmd': 0}
    last_report = time.monotonic()
    print(f'fake legs on {args.channel}, {args.rate:.0f} Hz, dbc {args.dbc}', flush=True)

    try:
        while True:
            # Handle everything that arrives until the next tick.
            while True:
                wait = next_tick - time.monotonic()
                if wait <= 0:
                    break
                msg = bus.recv(timeout=wait)
                if msg is None or msg.is_error_frame or msg.is_extended_id:
                    continue
                if msg.arbitration_id == sync_id:
                    last_sync = time.monotonic()
                    stats['sync'] += 1
                elif msg.arbitration_id in cmd_msgs:
                    leg = legs[cmd_msgs[msg.arbitration_id]]
                    c = db.decode_message(msg.arbitration_id, msg.data)
                    stats['cmd'] += 1
                    leg.enabled = bool(c['servo_enable'])
                    if leg.enabled and not leg.faults:
                        leg.target = [c['coxa_target'], c['femur_target'], c['tibia_target']]

            now = time.monotonic()
            next_tick += period
            if next_tick < now:  # fell behind, don't try to catch up
                next_tick = now + period

            starved = last_sync is None or now - last_sync > args.watchdog
            for i, leg in enumerate(legs):
                if starved and leg.enabled:
                    leg.faults |= FAULT_WATCHDOG
                    leg.target = list(leg.pos)  # hold
                elif not starved:
                    leg.faults &= ~FAULT_WATCHDOG
                speed = leg.step(period, args.speed)
                current = (250 + 3 * speed) if leg.enabled else 20
                data = state_msgs[i].encode({
                    'coxa_pos': leg.pos[0], 'femur_pos': leg.pos[1], 'tibia_pos': leg.pos[2],
                    'leg_current': min(65535, int(current))})
                bus.send(can.Message(arbitration_id=state_msgs[i].frame_id, data=data, is_extended_id=False))
                if tick % 4 == i % 4:
                    data = status_msgs[i].encode({
                        'tof_distance': 400 + random.randint(-5, 5), 'vbat': 15400, 'servo_rail': 6000,
                        'temperature': 32, 'faults': leg.faults})
                    bus.send(can.Message(arbitration_id=status_msgs[i].frame_id, data=data, is_extended_id=False))
            tick += 1

            if not args.quiet and now - last_report > 2.0:
                dt = now - last_report
                faults = ' '.join(l.name for l in legs if l.faults) or 'none'
                print(f'sync {stats["sync"] / dt:5.0f}/s  cmd {stats["cmd"] / dt:5.0f}/s  faults: {faults}', flush=True)
                stats = {'sync': 0, 'cmd': 0}
                last_report = now
    except KeyboardInterrupt:
        pass
    finally:
        bus.shutdown()


if __name__ == '__main__':
    main()
