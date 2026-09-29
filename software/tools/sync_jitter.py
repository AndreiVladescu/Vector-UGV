#!/usr/bin/env python3
"""Timing of the SYNC frames the CM5 sends to the legs: how steady the 200 Hz control loop is.

  sync_jitter.py vcan0 30        # listen 30 s, print the period statistics

The legs crouch after 100 ms without SYNC, so the worst gap is what matters most.
"""
import socket
import struct
import sys
import time


def main():
    ifname = sys.argv[1] if len(sys.argv) > 1 else 'can0'
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 30
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, struct.pack('=II', 0x000, 0x7FF))
    s.bind((ifname,))
    s.settimeout(1.0)
    stamps = []
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        try:
            s.recv(16)
        except socket.timeout:
            continue
        stamps.append(time.monotonic())
    gaps = sorted((b - a) * 1000 for a, b in zip(stamps, stamps[1:]))
    if not gaps:
        sys.exit('no SYNC frames on ' + ifname)
    n = len(gaps)
    mean = sum(gaps) / n
    pct = lambda p: gaps[min(n - 1, int(p / 100 * n))]
    print(f'{n + 1} SYNC frames in {seconds:.0f} s, period mean {mean:.2f} ms')
    print(f'  p50 {pct(50):.2f}  p99 {pct(99):.2f}  p99.9 {pct(99.9):.2f}  max {gaps[-1]:.2f} ms')
    print(f'  gaps over 10 ms: {sum(g > 10 for g in gaps)}, over 50 ms: {sum(g > 50 for g in gaps)}')


if __name__ == '__main__':
    main()
