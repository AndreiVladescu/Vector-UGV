import pytest

from vector_io import stm32boot as sb


class FakeRom:
    """The AN3155 commands io_flash.py uses, byte by byte, on a 64 KB flash."""

    def __init__(self, chip=sb.C092_ID):
        self.flash = bytearray(b'\xaa' * 0x10000)
        self.chip, self.synced, self.started = chip, False, None
        self.out, self.state = bytearray(), self._idle()
        next(self.state)

    def reset_input_buffer(self):
        self.out.clear()

    def read(self, n):
        d, self.out[:n] = bytes(self.out[:n]), b''
        return d

    def write(self, data):
        for b in data:
            self.state.send(b)

    def _take(self, n):
        buf = bytearray()
        while len(buf) < n:
            buf.append((yield))
        return bytes(buf)

    def _addr(self):
        a = yield from self._take(5)
        if sb._xor(a[:4]) != a[4]:
            return None
        self.out.append(sb.ACK)
        return int.from_bytes(a[:4], 'big') - sb.FLASH_BASE

    def _idle(self):
        while True:
            c = yield
            if not self.synced:
                if c == 0x7F:
                    self.synced = True
                    self.out.append(sb.ACK)
                continue
            x = yield
            if x != c ^ 0xFF:
                self.out.append(sb.NACK)
                continue
            self.out.append(sb.ACK)
            if c == 0x02:
                self.out += bytes([1]) + self.chip.to_bytes(2, 'big') + bytes([sb.ACK])
            elif c == 0x44:
                h = yield from self._take(2)
                n = int.from_bytes(h, 'big') + 1
                rest = yield from self._take(2 * n + 1)
                assert sb._xor(h + rest[:-1]) == rest[-1]
                for i in range(n):
                    p = int.from_bytes(rest[2 * i:2 * i + 2], 'big')
                    self.flash[p * sb.PAGE:(p + 1) * sb.PAGE] = b'\xff' * sb.PAGE
                self.out.append(sb.ACK)
            elif c == 0x31:
                a = yield from self._addr()
                n = (yield) + 1
                d = yield from self._take(n + 1)
                assert sb._xor(bytes([n - 1]) + d[:-1]) == d[-1] and n % 4 == 0
                self.flash[a:a + n] = bytes(x & y for x, y in zip(self.flash[a:a + n], d[:-1]))
                self.out.append(sb.ACK)
            elif c == 0x11:
                a = yield from self._addr()
                n, nc = yield from self._take(2)
                assert nc == n ^ 0xFF
                self.out.append(sb.ACK)
                self.out += self.flash[a:a + n + 1]
            elif c == 0x21:
                self.started = (yield from self._addr()) + sb.FLASH_BASE


def test_flash_writes_verifies_and_starts():
    rom = FakeRom()
    image = bytes(range(256)) * 20 + b'\x01\x02\x03'  # not a multiple of 4 or of a page
    sb.flash(sb.Bootloader(rom), image, log=lambda s: None)
    assert rom.flash[:len(image)] == image
    assert rom.flash[len(image):3 * sb.PAGE] == b'\xff' * (3 * sb.PAGE - len(image))
    assert rom.flash[3 * sb.PAGE:3 * sb.PAGE + 4] == b'\xaa' * 4  # pages past the image untouched
    assert rom.started == sb.FLASH_BASE


def test_already_synced_bootloader():
    rom = FakeRom()
    rom.synced = True  # a second run: 0x7F now reads as a bad command and gets a NACK
    sb.flash(sb.Bootloader(rom), b'\x11' * 8, log=lambda s: None)
    assert rom.flash[:8] == b'\x11' * 8


def test_wrong_chip():
    with pytest.raises(sb.BootError, match='chip ID'):
        sb.flash(sb.Bootloader(FakeRom(chip=0x466)), b'\0' * 4, log=lambda s: None)


def test_verify_catches_a_bad_write():
    rom = FakeRom()
    rom.flash[0:sb.PAGE] = b'\0' * sb.PAGE  # bits that the erase misses stay at 0
    bl = sb.Bootloader(rom)
    bl.erase = lambda first, count: None
    with pytest.raises(sb.BootError, match='verify failed at 0x08000000'):
        sb.flash(bl, b'\xff' * 16, log=lambda s: None)


@pytest.mark.parametrize('cut_after', [0, 1, 5, 19])
def test_cut_short_leaves_the_first_word_erased(cut_after):
    # until the last write the ROM's empty check still sees blank flash and starts the bootloader
    rom = FakeRom()
    bl = sb.Bootloader(rom)
    real, n = bl.write, [0]

    def write(addr, data):
        if n[0] == cut_after:
            raise KeyboardInterrupt
        n[0] += 1
        real(addr, data)
    bl.write = write
    with pytest.raises(KeyboardInterrupt):
        sb.flash(bl, bytes(range(256)) * 20, log=lambda s: None)
    assert rom.flash[:4] == b'\xff' * 4 and rom.started is None
