"""The UART link to the carrier's IO MCU (firmware/io-node, see its README): COBS frames of
type, payload and CRC-16/CCITT-FALSE, little endian, 1 Mbaud."""
import struct

STATUS, CRSF, LIDAR, NMEA, LORA_RX, LORA_TX = 0x01, 0x02, 0x03, 0x04, 0x05, 0x06
CRSF_OUT, LIDAR_PWM, BEEP, LTE_POWER, LORA_SEND, BEACON, BOOTLOADER = 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87
BOOT_MAGIC = 0x746f6f62
MAX_PAYLOAD = 250

FLAGS = ('lte_en', 'lte_status', 'crsf', 'lidar', 'gnss', 'fix', 'lora', 'host')
_STATUS = struct.Struct('<IIBBHHhBBHHHHHH')
LORA_RESULT = {0: 'sent', 1: 'radio busy', 2: 'duty cycle', 3: 'no radio'}
RESET = ((1, 'power'), (2, 'pin'), (4, 'watchdog'), (8, 'software'), (16, 'other'))


def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data):
    out, block = bytearray(), bytearray()
    for b in data:
        if b:
            block.append(b)
            if len(block) == 254:
                out += bytes([255]) + block
                block = bytearray()
        else:
            out += bytes([len(block) + 1]) + block
            block = bytearray()
    out += bytes([len(block) + 1]) + block
    return bytes(out)


def cobs_decode(data):
    out, i = bytearray(), 0
    while i < len(data):
        code = data[i]
        if code == 0 or i + code > len(data):
            raise ValueError('bad COBS')
        out += data[i + 1:i + code]
        i += code
        if code != 255 and i < len(data):
            out.append(0)
    return bytes(out)


def encode(mtype, payload=b''):
    if len(payload) > MAX_PAYLOAD:
        raise ValueError('payload too long')
    raw = bytes([mtype]) + bytes(payload)
    return cobs_encode(raw + struct.pack('<H', crc16(raw))) + b'\0'


class Decoder:
    """Feed it bytes, get (type, payload) for every good message."""

    def __init__(self):
        self.buf = bytearray()
        self.bad = 0

    def feed(self, data):
        out = []
        for b in data:
            if b:
                self.buf.append(b)
                continue
            raw, self.buf = bytes(self.buf), bytearray()
            if not raw:
                continue
            try:
                msg = cobs_decode(raw)
            except ValueError:
                self.bad += 1
                continue
            if len(msg) < 3 or crc16(msg[:-2]) != struct.unpack('<H', msg[-2:])[0]:
                self.bad += 1
                continue
            out.append((msg[0], msg[1:-2]))
        return out


def status(p):
    """STATUS payload -> dict; temperature None for an open or shorted NTC."""
    (version, uptime, reset, flags, vbat, v5, temp, sats, fix, beacons, dropped,
     bad_crsf, bad_lidar, bad_nmea, bad_link) = _STATUS.unpack(p[:_STATUS.size])
    d = {'version': f'{version & 0x0FFFFFFF:07x}' + ('+' if version & 0x10000000 else ''),
         'uptime_s': uptime / 1000, 'reset': ','.join(n for bit, n in RESET if reset & bit) or 'none',
         'vbat': vbat / 1000, 'v5': v5 / 1000, 'temp': None if temp == -32768 else temp / 10,
         'sats': sats, 'fix_quality': fix, 'beacons': beacons, 'dropped': dropped,
         'bad_crsf': bad_crsf, 'bad_lidar': bad_lidar, 'bad_nmea': bad_nmea, 'bad_link': bad_link}
    d.update({name: bool(flags & (1 << i)) for i, name in enumerate(FLAGS)})
    return d


def pack_status(**kw):
    """The other way round, for tests and fakes."""
    flags = sum(1 << i for i, name in enumerate(FLAGS) if kw.get(name))
    temp = kw.get('temp')
    return _STATUS.pack(kw.get('version', 0), int(kw.get('uptime_s', 0) * 1000), kw.get('reset', 0), flags,
                        int(kw.get('vbat', 0) * 1000), int(kw.get('v5', 0) * 1000),
                        -32768 if temp is None else int(round(temp * 10)), kw.get('sats', 0),
                        kw.get('fix_quality', 0), kw.get('beacons', 0), kw.get('dropped', 0), 0, 0, 0, 0)


def beep(hz, on_ms, off_ms, count):
    return encode(BEEP, struct.pack('<HHHB', hz, on_ms, off_ms, count))


def beacon(p):
    """A LoRa beacon packet -> dict, None if it isn't one."""
    if len(p) != 16 or p[0] != ord('V'):
        return None
    _, seq, fix, sats, lat, lon, alt, vbat = struct.unpack('<BBBBiihH', p)
    return {'seq': seq, 'fix': fix, 'sats': sats, 'lat': lat / 1e7, 'lon': lon / 1e7, 'alt': alt, 'vbat': vbat / 100}
