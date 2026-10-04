import struct

import pytest

from vector_io import link


def test_crc_check_value():
    assert link.crc16(b'123456789') == 0x29B1


def test_same_bytes_as_the_firmware():
    # firmware/io-node/test/test_io.c checks the same vector
    assert link.encode(0x01, bytes([0x11, 0x00, 0x22])) == bytes.fromhex('0301110422078200')


@pytest.mark.parametrize('payload', [b'', b'\0', b'\0\0\0', bytes(range(1, 251)), bytes(250), bytes(range(256))[:250]])
def test_roundtrip(payload):
    wire = link.encode(0x42, payload)
    assert wire.count(0) == 1 and wire[-1] == 0
    d = link.Decoder()
    half = len(wire) // 2
    assert d.feed(wire[:half]) == []
    assert d.feed(wire[half:]) == [(0x42, payload)]


def test_bad_frames_are_counted_and_skipped():
    d = link.Decoder()
    good = link.encode(7, b'abc')
    bad = bytearray(good)
    bad[2] ^= 0x01
    assert d.feed(b'\0\0' + bytes(bad) + good) == [(7, b'abc')]
    assert d.bad == 1


def test_too_long():
    with pytest.raises(ValueError):
        link.encode(1, bytes(251))


def test_status():
    p = link.pack_status(version=0x1234567 | 0x10000000, uptime_s=12.5, reset=4 | 8, vbat=14.8, v5=5.02,
                         temp=31.4, sats=9, fix_quality=1, beacons=3, crsf=True, lora=True, host=True)
    assert len(p) == 30
    s = link.status(p)
    assert s['version'] == '1234567+'
    assert s['reset'] == 'watchdog,software'
    assert s['vbat'] == 14.8 and s['v5'] == 5.02 and s['temp'] == 31.4
    assert s['crsf'] and s['lora'] and s['host'] and not s['lidar']
    assert link.status(link.pack_status(temp=None))['temp'] is None


def test_beacon():
    p = struct.pack('<BBBBiihH', ord('V'), 7, 1, 9, 444360233, 261028000, 80, 1220)
    b = link.beacon(p)
    assert b['seq'] == 7 and b['sats'] == 9 and abs(b['lat'] - 44.4360233) < 1e-9 and b['vbat'] == 12.2
    assert link.beacon(b'hello') is None
