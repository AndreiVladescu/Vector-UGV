"""Network state from /proc and /sys (host networking in the container shows the host's)."""
import os


def operstate(iface):
    try:
        return open(f'/sys/class/net/{iface}/operstate').read().strip()
    except OSError:
        return 'absent'


def wifi_dbm(iface='wlan0', path='/proc/net/wireless'):
    """Signal level in dBm, None when not associated."""
    try:
        for line in open(path):
            if line.strip().startswith(iface + ':'):
                return int(float(line.split()[3].rstrip('.')))
    except (OSError, ValueError, IndexError):
        pass
    return None


def default_route(path='/proc/net/route'):
    """Interface of the default route with the lowest metric."""
    best = None
    try:
        for line in list(open(path))[1:]:
            f = line.split()
            if f[1] == '00000000' and f[7] == '00000000':
                if best is None or int(f[6]) < best[1]:
                    best = (f[0], int(f[6]))
    except (OSError, IndexError, ValueError):
        pass
    return best[0] if best else None


def csq_dbm(reply):
    """AT+CSQ reply ('+CSQ: 18,99') -> dBm, None when unknown (99)."""
    for line in reply.splitlines():
        if line.startswith('+CSQ:'):
            rssi = int(line.split(':')[1].split(',')[0])
            return None if rssi == 99 else -113 + 2 * rssi
    return None


def usb_serial_exists(port):
    return bool(port) and os.path.exists(port)


def modem_iface(root='/sys/class/net', drivers=('rndis_host', 'cdc_ether', 'cdc_ncm', 'qmi_wwan')):
    """The LTE modem's network interface, found by its USB driver (the name depends on the
    modem's mode and the OS: usb0, enx..., wwan0)."""
    try:
        for iface in sorted(os.listdir(root)):
            drv = os.path.realpath(os.path.join(root, iface, 'device', 'driver'))
            if os.path.basename(drv) in drivers:
                return iface
    except OSError:
        pass
    return None
