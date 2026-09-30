import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from vector_link.crsf import (CH_MAX, CH_MID, CH_MIN, T_LINK_STATS, T_RC_CHANNELS, Parser,  # noqa: E402
                              battery, crc8, frame, link_stats, normalize, pack_channels, unpack_channels)


def test_crc_is_dvb_s2():
    assert crc8(b'123456789') == 0xBC  # the published CRC-8/DVB-S2 check value


def test_channel_frame_layout():
    ch = [CH_MID] * 16
    f = frame(T_RC_CHANNELS, pack_channels(ch))
    assert f[0] == 0xC8 and f[1] == 24 and f[2] == 0x16 and len(f) == 26
    assert crc8(f[2:-1]) == f[-1]


def test_channels_round_trip():
    ch = [CH_MIN, CH_MAX, 1500, 0, 2047] + list(range(100, 1100, 100)) + [7]
    assert unpack_channels(pack_channels(ch)) == ch


def test_normalize():
    assert normalize(CH_MID) == 0 and normalize(CH_MAX) == 1 and normalize(CH_MIN) == -1 and normalize(0) == -1


def test_parser_resyncs_and_drops_bad_crc():
    good = frame(T_RC_CHANNELS, pack_channels([CH_MID] * 16))
    bad = bytearray(good)
    bad[5] ^= 0xFF
    p = Parser()
    out = p.feed(b'\x00\x13' + bytes(bad) + good[:10])
    out += p.feed(good[10:])
    assert [t for t, _ in out] == [T_RC_CHANNELS] and p.bad >= 1


def test_link_stats():
    s = link_stats(bytes([60, 70, 95, 8, 0, 5, 3, 55, 100, 9]))
    assert s['lq'] == 95 and s['rssi1'] == -60 and s['snr'] == 8
    assert T_LINK_STATS == 0x14


def test_battery_frame():
    f = battery(15.87, 3.2, 1234, 67)
    assert f[2] == 0x08 and f[3:5] == (159).to_bytes(2, 'big') and f[5:7] == (32).to_bytes(2, 'big')
    assert int.from_bytes(f[7:10], 'big') == 1234 and f[10] == 67
