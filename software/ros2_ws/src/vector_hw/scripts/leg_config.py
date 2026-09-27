#!/usr/bin/env python3
"""Configure and calibrate the leg nodes over CAN.

  leg_config.py status                              state and faults of every leg
  leg_config.py read R1                             stored settings of one leg
  leg_config.py push                                docs/servos.md + legs.yaml -> all legs, saved
  leg_config.py calibrate R1                        sweep all three joints over 500-2500 us
  leg_config.py calibrate R1 --joint femur --mode limits

Calibration prints rows in the docs/servos.md format. Legs must be off (not walking),
and the leg should hang free: every joint sweeps its whole range.
"""
import argparse
import datetime
import math
import os
import re
import sys
import time

import can
import yaml

LEGS = {'L1': 1, 'L2': 2, 'L3': 3, 'R1': 4, 'R2': 5, 'R3': 6}
JOINTS = ['coxa', 'femur', 'tibia']
STATES = ['off', 'wake', 'active', 'crouch', 'calibrate', 'fault']
FAULTS = ['watchdog', 'estop', 'wake', 'uncalibrated', 'buck', 'overtemp', 'cal', 'config']

LEG_STATUS, LEG_CONFIG, LEG_REPLY = 0x030, 0x050, 0x060
OP_READ, OP_WRITE, OP_SAVE, OP_CALIBRATE = 0, 1, 2, 3
ST_OK, ST_CAL_RESULT, ST_CAL_FAILED, ST_CAL_DONE = 0, 4, 5, 6
STATUS_NAMES = ['ok', 'bad key', 'bad value', 'busy', 'cal result', 'cal failed', 'cal done']

# key: (name, scale to the wire)
KEYS = {1: ('mid_mv', 10), 2: ('slope', 1e4), 3: ('center_us', 10), 4: ('direction', 1),
        5: ('us_per_deg', 1000), 6: ('min_deg', 100), 7: ('max_deg', 100), 8: ('fit_err_mv', 10),
        9: ('calibrated', 1)}
KEY = {name: k for k, (name, _) in KEYS.items()}


def find_up(rel):
    d = os.getcwd()
    while True:
        p = os.path.join(d, rel)
        if os.path.exists(p):
            return p
        if os.path.dirname(d) == d:
            return rel
        d = os.path.dirname(d)


class Bus:
    def __init__(self, channel):
        self.bus = can.Bus(interface='socketcan', channel=channel)
        self.seq = 0

    def send(self, node, joint, key, value, op):
        self.seq = (self.seq + 1) & 0xff
        data = bytes([joint, key]) + int(value).to_bytes(4, 'little', signed=True) + bytes([self.seq, op])
        self.bus.send(can.Message(arbitration_id=LEG_CONFIG | node, data=data, is_extended_id=False))
        return self.seq

    def replies(self, node, timeout):
        end = time.time() + timeout
        while time.time() < end:
            m = self.bus.recv(timeout=end - time.time())
            if m and m.arbitration_id == LEG_REPLY | node and len(m.data) == 8:
                d = m.data
                yield d[0], d[1], int.from_bytes(d[2:6], 'little', signed=True), d[6], d[7]

    def request(self, node, joint, key, value=0, op=OP_READ, timeout=0.3):
        for _ in range(3):
            seq = self.send(node, joint, key, value, op)
            for j, k, v, s, status in self.replies(node, timeout):
                if s == seq:
                    return status, v
        raise TimeoutError(f'no reply from node {node}')


def cmd_status(bus, args):
    seen = {}
    end = time.time() + 1.0
    while time.time() < end and len(seen) < len(LEGS):
        m = bus.bus.recv(timeout=0.2)
        if m and m.arbitration_id & 0x7f0 == LEG_STATUS and len(m.data) == 8:
            seen[m.arbitration_id & 0xf] = m.data
    for name, node in LEGS.items():
        d = seen.get(node)
        if d is None:
            print(f'{name}: no answer')
            continue
        state = d[7] & 0xf
        faults = [f for i, f in enumerate(FAULTS) if d[6] & (1 << i)]
        vbat = (d[2] | (d[3] & 0xf) << 8) * 10
        print(f'{name}: {STATES[state] if state < len(STATES) else state:9s} '
              f'faults {",".join(faults) or "-":24s} vbat {vbat / 1000:.2f} V  temp {int.from_bytes(d[5:6], "little", signed=True)} C')


def cmd_read(bus, args):
    node = LEGS[args.leg]
    for j, joint in enumerate(JOINTS):
        vals = []
        for k, (name, scale) in KEYS.items():
            status, v = bus.request(node, j, k)
            vals.append(f'{name} {v / scale:g}' if status == ST_OK else f'{name} ?')
        print(f'{args.leg}_{joint}: ' + ', '.join(vals))


def servo_table(path):
    rows = {}
    for line in open(path):
        cells = [c.strip() for c in line.strip().strip('|').split('|')]
        if len(cells) > 4 and re.fullmatch(r'[LR][123]_(coxa|femur|tibia)', cells[0]):
            try:
                rows[cells[0]] = (float(cells[3]), float(cells[4]))
            except ValueError:
                pass
    return rows


def cmd_push(bus, args):
    table = servo_table(args.servos)
    limits = yaml.safe_load(open(args.legs))['limits']
    for name, node in LEGS.items():
        ok = True
        for j, joint in enumerate(JOINTS):
            lo, hi = (math.degrees(a) for a in limits[joint])
            # until the real joint zero is measured: put the middle of the range at 1500 us
            center = 1500 - (lo + hi) / 2 * 11.11
            writes = [(KEY['min_deg'], lo * 100), (KEY['max_deg'], hi * 100), (KEY['center_us'], center * 10)]
            cal = table.get(f'{name}_{joint}')
            if cal:
                writes = [(KEY['slope'], cal[1] * 1e4), (KEY['mid_mv'], cal[0] * 10)] + writes
            else:
                print(f'{name}_{joint}: not in {args.servos}, left as is')
            for key, value in writes:
                status, _ = bus.request(node, j, key, round(value), OP_WRITE)
                if status != ST_OK:
                    print(f'{name}_{joint}: {KEYS[key][0]} rejected ({STATUS_NAMES[status]})')
                    ok = False
        status, _ = bus.request(node, 0, 0, 0, OP_SAVE)
        print(f'{name}: {"saved" if ok and status == ST_OK else "check the messages above"}')


def cmd_calibrate(bus, args):
    node = LEGS[args.leg]
    joint = 0xff if args.joint == 'all' else JOINTS.index(args.joint)
    mode = 1 if args.mode == 'limits' else 0
    status, _ = bus.request(node, joint, 0, mode, OP_CALIBRATE)
    if status != ST_OK:
        sys.exit(f'{args.leg}: calibration refused ({STATUS_NAMES[status]}), is the leg off?')
    print(f'{args.leg}: calibrating {args.joint}, {args.mode} range, takes about 10 s per joint')

    results, failed = {}, []
    for j, key, value, _, st in bus.replies(node, timeout=12 * (3 if joint == 0xff else 1) + 5):
        if st == ST_CAL_RESULT:
            results.setdefault(j, {})[KEYS[key][0]] = value / KEYS[key][1]
        elif st == ST_CAL_FAILED:
            failed.append(j)
            print(f'{args.leg}_{JOINTS[j]}: FAILED, fit error {value / 10:.0f} mV (wiper not following?)')
        elif st == ST_CAL_DONE:
            break
    else:
        sys.exit('timed out waiting for the calibration to finish')

    today = datetime.date.today().isoformat()
    print('| Joint | Label | Date | mid (mV @ 1500 µs) | slope (mV/µs) | @ 500 µs | @ 2500 µs | fit error (mV) | noise p-p (mV) | Notes |')
    for j, r in sorted(results.items()):
        mid, slope = r['mid_mv'], r['slope']
        print(f'| {args.leg}_{JOINTS[j]} |  | {today} | {mid:.0f} | {slope:.3f} | {mid - 1000 * slope:.0f} | '
              f'{mid + 1000 * slope:.0f} | {r["fit_err_mv"]:.0f} |  | on the leg, {args.mode} range |')
    if args.save and not failed:
        print('saved' if bus.request(node, 0, 0, 0, OP_SAVE)[0] == ST_OK else 'save failed')
    elif not args.save:
        print('not saved; rerun with --save, or add the rows to docs/servos.md and use push')
    sys.exit(1 if failed else 0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--channel', default='can0')
    sub = ap.add_subparsers(dest='cmd', required=True)
    sub.add_parser('status')
    p = sub.add_parser('read')
    p.add_argument('leg', choices=LEGS)
    p = sub.add_parser('push')
    p.add_argument('--servos', default=find_up('docs/servos.md'))
    p.add_argument('--legs', default=find_up('software/ros2_ws/src/vector_description/config/legs.yaml'))
    p = sub.add_parser('calibrate')
    p.add_argument('leg', choices=LEGS)
    p.add_argument('--joint', choices=JOINTS + ['all'], default='all')
    p.add_argument('--mode', choices=['full', 'limits'], default='full',
                   help='full: 500-2500 us; limits: the joint range from its min/max angle plus 5 deg')
    p.add_argument('--save', action='store_true', help='store the result in the leg flash')
    args = ap.parse_args()

    bus = Bus(args.channel)
    try:
        {'status': cmd_status, 'read': cmd_read, 'push': cmd_push, 'calibrate': cmd_calibrate}[args.cmd](bus, args)
    finally:
        bus.bus.shutdown()


if __name__ == '__main__':
    main()
