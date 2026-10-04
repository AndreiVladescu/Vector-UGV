import math

import pytest

from vector_imu import lsm6dsv, mmc5983
from vector_imu.sim import SimBus


def test_lsm6dsv():
    bus = SimBus(gyro=(0.1, -0.2, 1.0), accel=(0.5, -9.0, 3.0))
    imu = lsm6dsv.Lsm6dsv(bus)
    imu.start()
    regs = {reg: data[0] for addr, reg, data in bus.writes}
    assert regs[lsm6dsv.CTRL3] == 0x44 and regs[lsm6dsv.CTRL6] == 0x03 and regs[lsm6dsv.CTRL8] == 0x01
    assert regs[lsm6dsv.CTRL1] == regs[lsm6dsv.CTRL2] == 0x07
    g, a, t = imu.read()
    assert g == pytest.approx((0.1, -0.2, 1.0), abs=lsm6dsv.GYRO_SCALE)
    assert a == pytest.approx((0.5, -9.0, 3.0), abs=lsm6dsv.ACCEL_SCALE)
    assert t == 30.0


def test_lsm6dsv_scales():
    # datasheet sensitivities: 1 g = 8197 LSB at +-4 g, 1000 dps = 28571 LSB
    assert 9.80665 / lsm6dsv.ACCEL_SCALE == pytest.approx(8196.7, abs=0.1)
    assert math.radians(1000) / lsm6dsv.GYRO_SCALE == pytest.approx(28571.4, abs=0.1)


def test_mmc5983():
    bus = SimBus(field=(20e-6, -35e-6, 41e-6))
    mag = mmc5983.Mmc5983(bus)
    mag.start()
    x, y, z = mag.read()
    lsb = 1e-4 / 16384
    assert (x, y, z) == pytest.approx((20e-6, -35e-6, 41e-6), abs=lsb)
    assert (mmc5983.ADDR, mmc5983.CTRL2, bytes([0x8C])) in bus.writes


def test_missing_parts():
    bus = SimBus(imu=False, compass=False)
    with pytest.raises(OSError):
        lsm6dsv.Lsm6dsv(bus).start()
    with pytest.raises(OSError):
        mmc5983.Mmc5983(bus).start()
