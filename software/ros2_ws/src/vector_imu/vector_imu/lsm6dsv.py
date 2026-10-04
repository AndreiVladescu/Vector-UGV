"""LSM6DSV16X on I2C (SA0 low: 0x6A): accelerometer +-4 g and gyro +-1000 dps at 240 Hz,
read by polling."""
import math
import struct
import time

ADDR = 0x6A
WHO_AM_I, WHO = 0x0F, 0x70
CTRL1, CTRL2, CTRL3, CTRL6, CTRL8 = 0x10, 0x11, 0x12, 0x15, 0x17
STATUS, OUT_TEMP, OUTX_G = 0x1E, 0x20, 0x22
ODR_240HZ = 0x07
G = 9.80665
ACCEL_SCALE = 0.122e-3 * G          # +-4 g: 0.122 mg/LSB -> m/s^2
GYRO_SCALE = math.radians(35e-3)    # +-1000 dps: 35 mdps/LSB -> rad/s


class Lsm6dsv:
    def __init__(self, bus, addr=ADDR):
        self.bus, self.addr = bus, addr

    def probe(self):
        return self.bus.read(self.addr, WHO_AM_I, 1)[0] == WHO

    def start(self):
        if not self.probe():
            raise OSError(f'no LSM6DSV16X at 0x{self.addr:02x}')
        self.bus.write(self.addr, CTRL3, [0x01])  # software reset
        time.sleep(0.01)
        self.bus.write(self.addr, CTRL3, [0x44])  # BDU, address auto-increment
        self.bus.write(self.addr, CTRL6, [0x03])  # +-1000 dps
        self.bus.write(self.addr, CTRL8, [0x01])  # +-4 g
        self.bus.write(self.addr, CTRL1, [ODR_240HZ])  # high-performance mode
        self.bus.write(self.addr, CTRL2, [ODR_240HZ])

    def read(self):
        """-> (gyro rad/s xyz, accel m/s^2 xyz, temperature degC)"""
        raw = self.bus.read(self.addr, OUT_TEMP, 14)
        t, gx, gy, gz, ax, ay, az = struct.unpack('<7h', raw)
        return ((gx * GYRO_SCALE, gy * GYRO_SCALE, gz * GYRO_SCALE),
                (ax * ACCEL_SCALE, ay * ACCEL_SCALE, az * ACCEL_SCALE), 25 + t / 256)
