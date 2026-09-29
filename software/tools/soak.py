#!/usr/bin/env python3
"""Long run on the robot computer: one line a minute of memory, temperature, CPU, SYNC timing
and YOLO rate, to a CSV and the terminal. Run on the host while the stack (and walk_test.py 0)
runs. Needs root for the per-container memory:

  sudo soak.py can0 120 soak.csv     # 120 minutes
"""
import csv
import os
import re
import subprocess
import sys
import time

from sync_jitter import pct, sync_gaps


def sh(cmd):
    return subprocess.run(cmd, shell=True, capture_output=True, text=True).stdout


def cpu_times():
    v = [int(x) for x in open('/proc/stat').readline().split()[1:]]
    return sum(v), v[3] + v[4]


def meminfo():
    m = dict(re.findall(r'(\w+):\s+(\d+)', open('/proc/meminfo').read()))
    return int(m['MemTotal']) - int(m['MemAvailable']), int(m['SwapTotal']) - int(m['SwapFree'])


def containers():
    """PSS per container in MB, from /proc (Pi OS has the memory cgroup off, so docker stats shows 0)."""
    names = dict(l.split() for l in sh("docker ps --no-trunc --format '{{.ID}} {{.Names}}'").splitlines())
    out = dict.fromkeys(names.values(), 0)
    for pid in filter(str.isdigit, os.listdir('/proc')):
        try:
            cg = open(f'/proc/{pid}/cgroup').read()
            m = re.search(r'docker-([0-9a-f]{64})', cg)
            if m and m.group(1) in names:
                pss = re.search(r'Pss:\s+(\d+)', open(f'/proc/{pid}/smaps_rollup').read())
                out[names[m.group(1)]] += int(pss.group(1))
        except (OSError, AttributeError):
            pass
    return {re.sub(r'^docker-|-\d+$', '', k): v // 1024 for k, v in out.items()}


def yolo_fps():
    lines = [l for l in sh('docker logs --since 60s docker-vision-1 2>&1').splitlines() if ' fps' in l]
    rates = [float(re.search(r'([\d.]+) fps', l).group(1)) for l in lines]
    return round(sum(rates) / len(rates), 1) if rates else 0


def main():
    ifname = sys.argv[1] if len(sys.argv) > 1 else 'can0'
    minutes = int(sys.argv[2]) if len(sys.argv) > 2 else 60
    path = sys.argv[3] if len(sys.argv) > 3 else 'soak.csv'
    fields = ['min', 'mem_mb', 'swap_mb', 'cpu_pct', 'temp_c', 'throttled', 'sync_p99', 'sync_max',
              'sync_over10', 'yolo_fps', 'containers']
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fields)
        w.writeheader()
        for i in range(1, minutes + 1):
            total0, idle0 = cpu_times()
            gaps = sync_gaps(ifname, 55) or [0]
            total1, idle1 = cpu_times()
            mem, swap = meminfo()
            row = {
                'min': i,
                'mem_mb': mem // 1024,
                'swap_mb': swap // 1024,
                'cpu_pct': round(100 * (1 - (idle1 - idle0) / (total1 - total0))),
                'temp_c': float(open('/sys/class/thermal/thermal_zone0/temp').read()) / 1000,
                'throttled': sh('vcgencmd get_throttled').strip().split('=')[-1],
                'sync_p99': round(pct(gaps, 99), 2),
                'sync_max': round(gaps[-1], 2),
                'sync_over10': sum(g > 10 for g in gaps),
                'yolo_fps': yolo_fps(),
                'containers': ' '.join(f'{k}={v}' for k, v in containers().items()),
            }
            w.writerow(row)
            f.flush()
            print(' '.join(f'{k} {v}' for k, v in row.items()), flush=True)


if __name__ == '__main__':
    main()
