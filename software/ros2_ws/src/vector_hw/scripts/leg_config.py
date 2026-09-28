#!/usr/bin/env python3
"""Configure and calibrate the leg nodes over CAN.

  leg_config.py status                              state and faults of every leg
  leg_config.py read R1                             stored settings of one leg
  leg_config.py push                                docs/servos.md + legs.yaml -> all legs, saved
  leg_config.py calibrate R1                        sweep all three joints over 500-2500 us
  leg_config.py calibrate R1 --joint femur --mode limits
  leg_config.py selftest all                         supplies and sensors of fresh boards, servos optional
  leg_config.py selftest R1 --vbat 15.92 --save      also correct the VBAT reading to a meter's value
  leg_config.py flash R1 leg-node.bin                new firmware through the CAN bootloader
  leg_config.py flash all leg-node.bin

Calibration prints rows in the docs/servos.md format. Legs must be off (not walking),
and the leg should hang free: every joint sweeps its whole range.
"""
import argparse
import binascii
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
STATES = ['off', 'wake', 'active', 'crouch', 'calibrate', 'fault', 'test']
FAULTS = ['watchdog', 'estop', 'wake', 'uncalibrated', 'buck', 'overload', 'cal', 'config']

LEG_STATUS, LEG_CONFIG, LEG_REPLY = 0x030, 0x050, 0x060
OP_READ, OP_WRITE, OP_SAVE, OP_CALIBRATE, OP_DEFAULTS, OP_SELFTEST = range(6)
ST_OK, ST_CAL_RESULT, ST_CAL_FAILED, ST_CAL_DONE = 0, 4, 5, 6
ST_TEST_PASS, ST_TEST_FAIL, ST_TEST_DONE = 7, 8, 9
STATUS_NAMES = ['ok', 'bad key', 'bad value', 'busy', 'cal result', 'cal failed', 'cal done',
                'test pass', 'test fail', 'test done']
LEG_WIDE = 0xff

CAN_BOOT, CAN_BOOT_REPLY = 0x070, 0x080
B_ENTER, B_INFO, B_ERASE, B_DATA, B_DONE, B_RUN = range(1, 7)
B_OK, B_BUSY = 0, 7
BOOT_STATUS = ['ok', 'bad op', 'bad sequence', 'flash error', 'bad crc', 'too big', 'no image', 'busy']

# key: (name, scale to the wire)
KEYS = {1: ('mid_mv', 10), 2: ('slope', 1e4), 3: ('center_us', 10), 4: ('direction', 1),
        5: ('us_per_deg', 1000), 6: ('min_deg', 100), 7: ('max_deg', 100), 8: ('fit_err_mv', 10),
        9: ('calibrated', 1)}
KEY = {name: k for k, (name, _) in KEYS.items()}
LEG_KEYS = {16: 'version', 17: 'reset_cause', 18: 'can_errors', 19: 'uptime_s', 20: 'vbat_gain', 21: 'rail_gain',
            22: 'i_zero_ma', 23: 'id_straps'}
LEG_KEY = {name: k for k, name in LEG_KEYS.items()}
RESETS = ['power', 'pin', 'watchdog', 'software', 'other']
# self-test items: name, unit, what a failure usually means
TESTS = {1: ('current zero', 'mA', 'INA181 or shunt'), 2: ('6V0 with buck off', 'mV', 'buck stuck on'),
         3: ('VBAT', 'mV', 'no supply or VBAT divider'), 4: ('temperature', 'C', 'NTC or its pull-up'),
         5: ('power good', '', 'buck, inductor or PG pull-up'), 6: ('6V0 with buck on', 'mV', 'buck or feedback'),
         7: ('current, buck on', 'mA', 'short on the servo rail'), 8: ('ToF', 'mm', 'no sensor or I2C'),
         9: ('coxa wiper', 'mV', 'no servo'), 10: ('femur wiper', 'mV', 'no servo'), 11: ('tibia wiper', 'mV', 'no servo')}


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

    def boot(self, node, data, timeout=0.2, tries=5, match=None):
        op = data[0]
        for _ in range(tries):
            self.bus.send(can.Message(arbitration_id=CAN_BOOT | node, data=data, is_extended_id=False))
            end = time.time() + timeout
            while time.time() < end:
                m = self.bus.recv(timeout=end - time.time())
                if (m and m.arbitration_id == CAN_BOOT_REPLY | node and len(m.data) >= 6 and m.data[0] == op | 0x80
                        and (match is None or match(m.data))):
                    return m.data[1], int.from_bytes(m.data[2:6], 'little')
        raise TimeoutError(f'no bootloader reply from node {node}')


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


def leg_info(bus, node):
    vals = {}
    for k, name in LEG_KEYS.items():
        status, v = bus.request(node, LEG_WIDE, k)
        vals[name] = v if status == ST_OK else None
    return vals


def describe(info):
    v = info['version']
    version = '?' if v is None else f'{v & 0xfffffff:07x}' + ('+local changes' if v & 0x10000000 else '')
    rc = info['reset_cause'] or 0
    errs = info['can_errors'] or 0
    return (f'firmware {version}, last reset {",".join(r for i, r in enumerate(RESETS) if rc & (1 << i)) or "?"}, '
            f'up {info["uptime_s"]} s, CAN bus-off {errs >> 16} dropped {errs & 0xffff}, '
            f'current zero {info["i_zero_ma"]} mA, gains vbat {(info["vbat_gain"] or 0) / 1e4:.4f} '
            f'rail {(info["rail_gain"] or 0) / 1e4:.4f}, {straps(info["id_straps"])}')


def straps(v):
    if v is None or v == 0xff:
        return 'leg ID set at build time'
    return f'ID jumpers {"right" if v & 4 else "left"} position {v & 3}' + (' (not set!)' if v & 3 == 0 else '')


def cmd_read(bus, args):
    node = LEGS[args.leg]
    print(f'{args.leg}: ' + describe(leg_info(bus, node)))
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


def selftest_leg(bus, name, args):
    node = LEGS[name]
    status, _ = bus.request(node, LEG_WIDE, 0, 0, OP_SELFTEST)
    if status != ST_OK:
        print(f'{name}: self-test refused ({STATUS_NAMES[status]}), is the leg off?')
        return False
    readings, failed = {}, None
    for _, item, value, _, st in bus.replies(node, timeout=2):
        if st in (ST_TEST_PASS, ST_TEST_FAIL):
            readings[item] = (value, st == ST_TEST_PASS)
        elif st == ST_TEST_DONE:
            failed = value
            break
    if failed is None:
        print(f'{name}: self-test timed out')
        return False
    print(f'{name}:')
    for item, (label, unit, hint) in TESTS.items():
        if item in readings:
            value, ok = readings[item]
            print(f'  {label:18s} {value:7d} {unit:3s} {"ok" if ok else "FAIL  (" + hint + ")"}')
    # the wiper checks only say whether a servo is plugged in
    real = sum(1 for item, (_, ok) in readings.items() if not ok and item < 9)
    print(f'  {"pass" if real == 0 else f"{real} failed"}')

    # the 6V0 rail is only up for 200 ms here, too short for a meter; its gain can be
    # written directly with the rail_gain key if it ever matters
    if args.vbat is not None and readings.get(3, (0,))[0] > 0:
        _, gain = bus.request(node, LEG_WIDE, LEG_KEY['vbat_gain'])
        new = round(gain * args.vbat * 1000 / readings[3][0])
        status, _ = bus.request(node, LEG_WIDE, LEG_KEY['vbat_gain'], new, OP_WRITE)
        print(f'  vbat_gain {gain / 1e4:.4f} -> {new / 1e4:.4f}' + ('' if status == ST_OK else f' rejected ({STATUS_NAMES[status]})'))
        if args.save:
            print('  saved' if bus.request(node, 0, 0, 0, OP_SAVE)[0] == ST_OK else '  save failed')
    return real == 0


def cmd_selftest(bus, args):
    ok = [selftest_leg(bus, name, args) for name in (LEGS if args.leg == 'all' else [args.leg])]
    sys.exit(0 if all(ok) else 1)


def flash_leg(bus, name, image):
    node = LEGS[name]
    # the application resets into the bootloader, which then answers; right after power-up
    # the bootloader also listens for 200 ms, which rescues a leg with broken firmware
    st, info = bus.boot(node, bytes([B_ENTER]) + b'boot', timeout=0.1, tries=30)
    if st == B_BUSY:
        raise RuntimeError(f'{name}: leg is powered, turn the legs off first')
    size_max = (info >> 16) * 1024
    print(f'{name}: bootloader v{info & 0xff}, image {"valid" if info & 0x100 else "missing"}')
    data = image + b'\xff' * (-len(image) % 8)
    if len(data) > size_max:
        raise RuntimeError(f'{name}: {len(data)} bytes, only {size_max} fit')

    pages = (len(data) + 2047) // 2048
    st, _ = bus.boot(node, bytes([B_ERASE]) + len(data).to_bytes(4, 'little'), timeout=0.05 * pages + 1, tries=1)
    if st != B_OK:
        raise RuntimeError(f'{name}: erase failed ({BOOT_STATUS[st]})')

    t = time.time()
    for i, off in enumerate(range(0, len(data), 6)):
        seq = i & 0xff
        st, _ = bus.boot(node, bytes([B_DATA, seq]) + data[off:off + 6],
                         match=lambda d, s=seq: d[1] != B_OK or d[2] == (s + 1) & 0xff)
        if st != B_OK:
            raise RuntimeError(f'{name}: write failed at {off} ({BOOT_STATUS[st]})')
        if i % 500 == 0:
            print(f'\r{name}: {off * 100 // len(data):3d} %', end='', flush=True)
    print(f'\r{name}: {len(data)} bytes in {time.time() - t:.1f} s')

    st, _ = bus.boot(node, bytes([B_DONE]) + binascii.crc32(data).to_bytes(4, 'little'), timeout=1, tries=1)
    if st != B_OK:
        raise RuntimeError(f'{name}: image rejected ({BOOT_STATUS[st]})')
    bus.boot(node, bytes([B_RUN]))

    end = time.time() + 2
    while time.time() < end:
        m = bus.bus.recv(timeout=0.2)
        if m and m.arbitration_id == LEG_STATUS | node:
            print(f'{name}: running')
            return
    raise RuntimeError(f'{name}: no status after the restart')


def cmd_flash(bus, args):
    image = open(args.image, 'rb').read()
    failed = []
    for name in (LEGS if args.leg == 'all' else [args.leg]):
        try:
            flash_leg(bus, name, image)
        except (RuntimeError, TimeoutError) as e:
            print(e)
            failed.append(name)
    sys.exit(f'failed: {", ".join(failed)}' if failed else 0)


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
    p = sub.add_parser('selftest')
    p.add_argument('leg', choices=list(LEGS) + ['all'])
    p.add_argument('--vbat', type=float, help='battery voltage from a meter, V: corrects the VBAT divider')
    p.add_argument('--save', action='store_true', help='store the correction in the leg flash')
    p = sub.add_parser('flash')
    p.add_argument('leg', choices=list(LEGS) + ['all'])
    p.add_argument('image', help='leg-node.bin from the STM32 build')
    args = ap.parse_args()

    bus = Bus(args.channel)
    try:
        {'status': cmd_status, 'read': cmd_read, 'push': cmd_push, 'calibrate': cmd_calibrate,
         'selftest': cmd_selftest, 'flash': cmd_flash}[args.cmd](bus, args)
    finally:
        bus.bus.shutdown()


if __name__ == '__main__':
    main()
