#!/usr/bin/env python3
"""Where the RAM goes: proportional set size (shared pages split fairly) per process,
grouped by name, plus what the kernel says is available.

  sudo mem_report.py            # sudo to read every process
"""
import collections
import os


def pss_kb(pid):
    try:
        with open(f'/proc/{pid}/smaps_rollup') as f:
            for line in f:
                if line.startswith('Pss:'):
                    return int(line.split()[1])
    except OSError:
        pass
    return 0


def name(pid):
    try:
        with open(f'/proc/{pid}/cmdline', 'rb') as f:
            args = [a.decode(errors='replace') for a in f.read().split(b'\0') if a]
    except OSError:
        return None
    if not args:
        return None
    exe = os.path.basename(args[0])
    # python and ros2 entry points: name them by the script / node
    if exe.startswith('python') and len(args) > 1:
        exe = os.path.basename(args[1])
    for a in args:
        if a.startswith('__node:='):
            exe = a.split(':=')[1]
    return exe


def main():
    total = collections.Counter()
    count = collections.Counter()
    for pid in filter(str.isdigit, os.listdir('/proc')):
        n = name(pid)
        if n:
            total[n] += pss_kb(pid)
            count[n] += 1
    meminfo = dict(line.split(':') for line in open('/proc/meminfo'))
    kb = lambda k: int(meminfo[k].split()[0])
    used = kb('MemTotal') - kb('MemAvailable')
    print(f'used {used / 1024:.0f} MB of {kb("MemTotal") / 1024:.0f} MB, available {kb("MemAvailable") / 1024:.0f} MB, '
          f'swap used {(kb("SwapTotal") - kb("SwapFree")) / 1024:.0f} MB')
    print(f'{"PSS MB":>7}  processes')
    for n, v in total.most_common(25):
        print(f'{v / 1024:7.1f}  {n}' + (f' (x{count[n]})' if count[n] > 1 else ''))


if __name__ == '__main__':
    main()
