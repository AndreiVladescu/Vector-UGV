"""ST's ROM bootloader on a UART (AN3155), as much as io_flash.py needs: erase, write, verify,
go. stm32flash 0.7 doesn't know the STM32C092. The port is a pyserial-like object at 8E1."""
import time

ACK, NACK = 0x79, 0x1F
FLASH_BASE = 0x08000000
PAGE = 2048
C092_ID = 0x44D  # STM32C091xx/C092xx
CHUNK = 256


class BootError(Exception):
    pass


def _xor(data):
    x = 0
    for b in data:
        x ^= b
    return x


class Bootloader:
    def __init__(self, port):
        self.s = port

    def _ack(self, timeout=1.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            b = self.s.read(1)
            if not b:
                continue
            if b[0] == ACK:
                return
            raise BootError('NACK' if b[0] == NACK else f'unexpected byte {b[0]:#04x}')
        raise BootError('no answer')

    def _cmd(self, c):
        self.s.write(bytes([c, c ^ 0xFF]))
        self._ack()

    def _addr(self, addr):
        a = addr.to_bytes(4, 'big')
        self.s.write(a + bytes([_xor(a)]))
        self._ack()

    def _exact(self, n):
        d = self.s.read(n)
        if len(d) != n:
            raise BootError('short reply')
        return d

    def sync(self, tries=3):
        """0x79 after autobaud; 0x1F if it was already synced (0x7F then reads as a bad command)."""
        for _ in range(tries):
            self.s.reset_input_buffer()
            self.s.write(b'\x7f')
            b = self.s.read(1)
            if b and b[0] in (ACK, NACK):
                return
        raise BootError('no bootloader')

    def chip_id(self):
        self._cmd(0x02)
        n = self._exact(1)[0]
        pid = self._exact(n + 1)
        self._ack()
        return int.from_bytes(pid, 'big')

    def erase(self, first, count):
        """Extended erase of pages first .. first + count - 1."""
        self._cmd(0x44)
        d = (count - 1).to_bytes(2, 'big') + b''.join(p.to_bytes(2, 'big') for p in range(first, first + count))
        self.s.write(d + bytes([_xor(d)]))
        self._ack(timeout=2.0 + 0.05 * count)

    def write(self, addr, data):
        self._cmd(0x31)
        self._addr(addr)
        d = bytes([len(data) - 1]) + data
        self.s.write(d + bytes([_xor(d)]))
        self._ack()

    def read(self, addr, n):
        self._cmd(0x11)
        self._addr(addr)
        self.s.write(bytes([n - 1, (n - 1) ^ 0xFF]))
        self._ack()
        return self._exact(n)

    def go(self, addr):
        self._cmd(0x21)
        self._addr(addr)


def flash(bl, image, base=FLASH_BASE, log=print):
    """Erase the pages the image covers, write and read back in 256-byte blocks, start it."""
    image = bytes(image) + b'\xff' * (-len(image) % 4)
    bl.sync()
    cid = bl.chip_id()
    if cid != C092_ID:
        raise BootError(f'chip ID {cid:#05x}, expected {C092_ID:#05x} (STM32C092)')
    pages = -(-len(image) // PAGE)
    bl.erase((base - FLASH_BASE) // PAGE, pages)
    log(f'erased {pages} pages')
    t0 = time.monotonic()
    for off in range(0, len(image), CHUNK):
        bl.write(base + off, image[off:off + CHUNK])
    for off in range(0, len(image), CHUNK):
        chunk = image[off:off + CHUNK]
        if bl.read(base + off, len(chunk)) != chunk:
            raise BootError(f'verify failed at {base + off:#010x}')
    log(f'wrote and verified {len(image)} bytes in {time.monotonic() - t0:.1f} s')
    bl.go(base)
