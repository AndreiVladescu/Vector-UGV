"""MMC5983MA compass on the nose board (I2C 0x30): continuous 50 Hz with automatic set/reset,
18-bit output, 16384 counts per gauss around 131072."""
import time

ADDR = 0x30
PRODUCT_ID, PID = 0x2F, 0x30
XOUT0, CTRL0, CTRL1, CTRL2 = 0x00, 0x09, 0x0A, 0x0B
NULL, PER_GAUSS = 131072, 16384
TESLA_PER_GAUSS = 1e-4


class Mmc5983:
    def __init__(self, bus, addr=ADDR):
        self.bus, self.addr = bus, addr

    def probe(self):
        return self.bus.read(self.addr, PRODUCT_ID, 1)[0] == PID

    def start(self):
        if not self.probe():
            raise OSError(f'no MMC5983MA at 0x{self.addr:02x}')
        self.bus.write(self.addr, CTRL1, [0x80])  # software reset
        time.sleep(0.015)
        self.bus.write(self.addr, CTRL0, [0x08])  # one SET pulse
        time.sleep(0.001)
        self.bus.write(self.addr, CTRL0, [0x20])  # automatic set/reset
        self.bus.write(self.addr, CTRL1, [0x00])  # 100 Hz bandwidth
        self.bus.write(self.addr, CTRL2, [0x80 | 0x08 | 0x04])  # periodic set each sample, continuous, 50 Hz

    def read(self):
        """-> field xyz in tesla"""
        b = self.bus.read(self.addr, XOUT0, 7)
        raw = (b[0] << 10 | b[1] << 2 | (b[6] >> 6) & 3,
               b[2] << 10 | b[3] << 2 | (b[6] >> 4) & 3,
               b[4] << 10 | b[5] << 2 | (b[6] >> 2) & 3)
        return tuple((r - NULL) / PER_GAUSS * TESLA_PER_GAUSS for r in raw)
