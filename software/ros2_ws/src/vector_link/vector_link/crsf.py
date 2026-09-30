"""CRSF, the serial protocol between an ExpressLRS receiver and a flight controller (here:
the robot). 420000 baud, 8N1. Frame: sync 0xC8, length (type + payload + crc), type,
payload, CRC-8 (poly 0xD5) over type and payload."""
import struct

SYNC = 0xC8
T_BATTERY = 0x08
T_LINK_STATS = 0x14
T_RC_CHANNELS = 0x16
T_FLIGHT_MODE = 0x21

CH_MIN, CH_MID, CH_MAX = 172, 992, 1811  # -100 %, 0, +100 % (988 / 1500 / 2012 us)


def crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def frame(ftype, payload):
    body = bytes([ftype]) + bytes(payload)
    return bytes([SYNC, len(body) + 1]) + body + bytes([crc8(body)])


def unpack_channels(p):
    """22 bytes -> 16 channels of 11 bits, LSB first."""
    v = int.from_bytes(p[:22], 'little')
    return [(v >> (11 * i)) & 0x7FF for i in range(16)]


def pack_channels(ch):
    v = 0
    for i, c in enumerate(ch[:16]):
        v |= (int(c) & 0x7FF) << (11 * i)
    return v.to_bytes(22, 'little')


def normalize(raw):
    """Channel value -> -1..1."""
    return max(-1.0, min(1.0, (raw - CH_MID) / (CH_MAX - CH_MID)))


def link_stats(p):
    """LINK_STATISTICS payload -> dict (RSSI in dBm, LQ in %)."""
    rssi1, rssi2, lq, snr, ant, mode, power, d_rssi, d_lq, d_snr = struct.unpack('<BBBbBBBBBb', p[:10])
    return {'rssi': -(rssi1 if ant == 0 else rssi2), 'rssi1': -rssi1, 'rssi2': -rssi2, 'lq': lq,
            'snr': snr, 'rf_mode': mode, 'down_rssi': -d_rssi, 'down_lq': d_lq}


def battery(volts, amps, used_mah, percent):
    """Battery telemetry for the handset: 0.1 V, 0.1 A, mAh, %."""
    v = max(0, min(0xFFFF, round(volts * 10)))
    a = max(0, min(0xFFFF, round(amps * 10)))
    mah = max(0, min(0xFFFFFF, round(used_mah)))
    return frame(T_BATTERY, struct.pack('>HH', v, a) + mah.to_bytes(3, 'big') + bytes([max(0, min(100, round(percent)))]))


def flight_mode(text):
    return frame(T_FLIGHT_MODE, text.encode()[:14] + b'\0')


class Parser:
    """Feed it bytes, get (type, payload) for every frame with a good CRC."""

    def __init__(self):
        self.buf = bytearray()
        self.bad = 0

    def feed(self, data):
        self.buf += data
        out = []
        while len(self.buf) >= 2:
            if self.buf[0] != SYNC:
                del self.buf[0]
                continue
            n = self.buf[1]
            if n < 2 or n > 62:
                del self.buf[0]
                continue
            if len(self.buf) < n + 2:
                break
            body, crc = bytes(self.buf[2:n + 1]), self.buf[n + 1]
            if crc8(body) == crc:
                out.append((body[0], body[1:]))
                del self.buf[:n + 2]
            else:
                self.bad += 1
                del self.buf[0]
        return out
