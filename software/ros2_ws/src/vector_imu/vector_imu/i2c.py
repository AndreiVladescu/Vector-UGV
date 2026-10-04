"""Linux i2c-dev: register reads with a repeated start (I2C_RDWR) and plain writes."""
import ctypes
import fcntl
import os

I2C_RDWR = 0x0707
I2C_M_RD = 0x0001


class _Msg(ctypes.Structure):
    _fields_ = [('addr', ctypes.c_uint16), ('flags', ctypes.c_uint16), ('len', ctypes.c_uint16),
                ('buf', ctypes.POINTER(ctypes.c_uint8))]


class _RdWr(ctypes.Structure):
    _fields_ = [('msgs', ctypes.POINTER(_Msg)), ('nmsgs', ctypes.c_uint32)]


class Bus:
    def __init__(self, dev='/dev/i2c-1'):
        self.fd = os.open(dev, os.O_RDWR)

    def close(self):
        os.close(self.fd)

    def _xfer(self, *msgs):
        arr = (_Msg * len(msgs))(*msgs)
        fcntl.ioctl(self.fd, I2C_RDWR, _RdWr(arr, len(msgs)))

    def read(self, addr, reg, n):
        wbuf = (ctypes.c_uint8 * 1)(reg)
        rbuf = (ctypes.c_uint8 * n)()
        self._xfer(_Msg(addr, 0, 1, wbuf), _Msg(addr, I2C_M_RD, n, rbuf))
        return bytes(rbuf)

    def write(self, addr, reg, data):
        data = bytes([reg]) + bytes(data)
        buf = (ctypes.c_uint8 * len(data)).from_buffer_copy(data)
        self._xfer(_Msg(addr, 0, len(data), buf))
