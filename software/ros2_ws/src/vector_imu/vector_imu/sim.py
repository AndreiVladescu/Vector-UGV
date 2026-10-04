"""Register-level stand-ins for the LSM6DSV16X and MMC5983MA on a fake I2C bus, for tests and
for running imu_node.py without hardware (sim:=true)."""
import math
import struct

from vector_imu import lsm6dsv, mmc5983


class SimBus:
    def __init__(self, gyro=(0.0, 0.0, 0.0), accel=(0.0, 0.0, 9.80665), field=(0.0, 24e-6, -41.6e-6),
                 imu=True, compass=True):
        self.gyro, self.accel, self.field = gyro, accel, field
        self.imu, self.compass = imu, compass
        self.writes = []

    def read(self, addr, reg, n):
        if addr == lsm6dsv.ADDR and self.imu:
            regs = bytearray(0x80)
            regs[lsm6dsv.WHO_AM_I] = lsm6dsv.WHO
            g = [int(round(v / lsm6dsv.GYRO_SCALE)) for v in self.gyro]
            a = [int(round(v / lsm6dsv.ACCEL_SCALE)) for v in self.accel]
            regs[lsm6dsv.OUT_TEMP:lsm6dsv.OUT_TEMP + 14] = struct.pack('<7h', 5 * 256, *g, *a)
            return bytes(regs[reg:reg + n])
        if addr == mmc5983.ADDR and self.compass:
            regs = bytearray(0x30)
            regs[mmc5983.PRODUCT_ID] = mmc5983.PID
            raw = [max(0, min(262143, int(round(v / mmc5983.TESLA_PER_GAUSS * mmc5983.PER_GAUSS)) + mmc5983.NULL))
                   for v in self.field]
            for i, r in enumerate(raw):
                regs[2 * i], regs[2 * i + 1] = r >> 10, (r >> 2) & 0xFF
            regs[6] = (raw[0] & 3) << 6 | (raw[1] & 3) << 4 | (raw[2] & 3) << 2
            return bytes(regs[reg:reg + n])
        raise OSError(121, 'Remote I/O error')  # what i2c-dev says to a missing chip

    def write(self, addr, reg, data):
        if (addr == lsm6dsv.ADDR and self.imu) or (addr == mmc5983.ADDR and self.compass):
            self.writes.append((addr, reg, bytes(data)))
            return
        raise OSError(121, 'Remote I/O error')


def turning(rate=0.0):
    """A SimBus whose readings follow a body turning at rate rad/s about z, level."""
    bus = SimBus(gyro=(0.0, 0.0, rate))
    bus.yaw = 0.0
    north = bus.field

    def step(dt):
        bus.yaw += rate * dt
        c, s = math.cos(bus.yaw), math.sin(bus.yaw)
        bus.field = (c * north[0] + s * north[1], -s * north[0] + c * north[1], north[2])
    bus.step = step
    return bus
