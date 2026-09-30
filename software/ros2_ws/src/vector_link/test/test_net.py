import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from vector_link.net import csq_dbm, default_route, wifi_dbm  # noqa: E402

WIRELESS = """Inter-| sta-|   Quality        |   Discarded packets               | Missed | WE
 face | tus | link level noise |  nwid  crypt   frag  retry   misc | beacon | 22
 wlan0: 0000   54.  -56.  -256        0      0      0      0     14        0
"""
ROUTE = """Iface\tDestination\tGateway \tFlags\tRefCnt\tUse\tMetric\tMask\t\tMTU\tWindow\tIRTT
usb0\t00000000\t01AAA8C0\t0003\t0\t0\t700\t00000000\t0\t0\t0
wlan0\t00000000\t0100A8C0\t0003\t0\t0\t600\t00000000\t0\t0\t0
wlan0\t0000A8C0\t00000000\t0001\t0\t0\t600\t00FFFFFF\t0\t0\t0
"""


def test_wifi(tmp_path):
    p = tmp_path / 'w'
    p.write_text(WIRELESS)
    assert wifi_dbm('wlan0', str(p)) == -56 and wifi_dbm('wlan1', str(p)) is None


def test_default_route_prefers_low_metric(tmp_path):
    p = tmp_path / 'r'
    p.write_text(ROUTE)
    assert default_route(str(p)) == 'wlan0'


def test_csq():
    assert csq_dbm('AT+CSQ\r\r\n+CSQ: 18,99\r\n\r\nOK\r\n') == -77
    assert csq_dbm('+CSQ: 99,99') is None


def test_modem_found_by_driver(tmp_path):
    from vector_link.net import modem_iface
    for iface, drv in (('eth0', 'macb'), ('enx0a1b2c', 'rndis_host'), ('wlan0', 'brcmfmac')):
        dev = tmp_path / 'devices' / iface
        (dev / 'drivers' / drv).mkdir(parents=True)
        (tmp_path / 'net' / iface).mkdir(parents=True)
        os.symlink(dev, tmp_path / 'net' / iface / 'device')
        os.symlink(dev / 'drivers' / drv, dev / 'driver')
    assert modem_iface(str(tmp_path / 'net')) == 'enx0a1b2c'
    assert modem_iface(str(tmp_path / 'nothing')) is None
