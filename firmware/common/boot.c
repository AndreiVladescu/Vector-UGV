#include "boot.h"

#include <string.h>

static uint32_t get32(const uint8_t *p) { return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24; }

uint32_t boot_crc32(uint32_t crc, const uint8_t *p, uint32_t len)
{
    crc = ~crc;
    while (len--) {
        crc ^= *p++;
        for (int i = 0; i < 8; i++)
            crc = (crc >> 1) ^ (0xedb88320u & -(crc & 1));
    }
    return ~crc;
}

void boot_reply(const struct boot_io *io, uint8_t node, uint8_t op, uint8_t status, uint32_t arg)
{
    struct can_frame_t f = {.id = CAN_BOOT_REPLY | node, .len = 8,
                            .data = {op | 0x80, status, arg, arg >> 8, arg >> 16, arg >> 24, 0, 0}};
    io->send(io->ctx, &f);
}

bool boot_is_enter(const struct can_frame_t *f, uint8_t node)
{
    return f->id == (CAN_BOOT | node) && f->len >= 5 && f->data[0] == BOOT_ENTER && get32(&f->data[1]) == BOOT_ENTER_MAGIC;
}

bool boot_image_valid(const struct boot_io *io)
{
    uint32_t sp = get32(io->image), reset = get32(io->image + 4);
    return sp >= io->sram_lo && sp <= io->sram_hi && (reset & 1) &&
           reset > io->image_addr && reset < io->image_addr + io->size;
}

void boot_init(struct boot *b, const struct boot_io *io, uint8_t node)
{
    memset(b, 0, sizeof(*b));
    b->io = io;
    b->node = node;
}

/* The first doubleword (stack pointer, reset vector) is held back until the CRC checks out,
   so a half-written image never looks runnable. */
static bool store(struct boot *b)
{
    uint32_t at = (b->written - 1) & ~7u;
    if (at == 0) {
        memcpy(b->head, b->acc, 8);
        return true;
    }
    return b->io->program(b->io->ctx, at, b->acc);
}

static uint8_t data(struct boot *b, const struct can_frame_t *f)
{
    if (f->len < 3)
        return BOOT_BAD_OP;
    if (f->data[1] == (uint8_t)(b->seq - 1))
        return BOOT_OK; /* our ack got lost, the host resent */
    if (f->data[1] != b->seq)
        return BOOT_BAD_SEQ;
    int n = f->len - 2;
    if (b->written + n > b->erased)
        return BOOT_TOO_BIG;
    for (int i = 0; i < n; i++) {
        b->acc[b->written++ & 7] = f->data[2 + i];
        if ((b->written & 7) == 0 && !store(b))
            return BOOT_FLASH_ERR;
    }
    b->seq++;
    return BOOT_OK;
}

static uint8_t done(struct boot *b, const struct can_frame_t *f)
{
    if (f->len < 5 || b->written == 0)
        return BOOT_BAD_OP;
    if (b->erased == 0) {
        /* already committed, the host is repeating DONE because our reply got lost */
        uint32_t crc = boot_crc32(0, b->io->image, b->written);
        return crc != get32(&f->data[1]) ? BOOT_BAD_CRC : boot_image_valid(b->io) ? BOOT_OK : BOOT_NO_IMAGE;
    }
    if (b->written & 7) {
        memset(&b->acc[b->written & 7], 0xff, 8 - (b->written & 7));
        b->written = (b->written + 7) & ~7u; /* pad, then the CRC covers the padding too */
        if (!store(b))
            return BOOT_FLASH_ERR;
    }
    uint32_t n = b->written < 8 ? b->written : 8;
    uint32_t crc = boot_crc32(0, b->head, n);
    crc = boot_crc32(crc, b->io->image + n, b->written - n);
    if (crc != get32(&f->data[1]))
        return BOOT_BAD_CRC;
    if (!b->io->program(b->io->ctx, 0, b->head))
        return BOOT_FLASH_ERR;
    b->erased = 0;
    return boot_image_valid(b->io) ? BOOT_OK : BOOT_NO_IMAGE;
}

bool boot_frame(struct boot *b, const struct can_frame_t *f)
{
    const struct boot_io *io = b->io;
    if (f->id != (CAN_BOOT | b->node) || f->len < 1)
        return false;
    uint8_t op = f->data[0], st = BOOT_OK;
    uint32_t arg = 0;
    switch (op) {
    case BOOT_ENTER:
    case BOOT_INFO:
        arg = BOOT_VERSION | (boot_image_valid(io) ? 0x100 : 0) | (io->size / 1024) << 16;
        break;
    case BOOT_ERASE: {
        uint32_t len = f->len >= 5 ? get32(&f->data[1]) : 0;
        len = (len + io->page - 1) / io->page * io->page;
        if (len == 0 || len > io->size)
            st = BOOT_TOO_BIG;
        else if (!io->erase(io->ctx, 0, len))
            st = BOOT_FLASH_ERR;
        b->erased = st == BOOT_OK ? len : 0;
        b->written = 0;
        b->seq = 0;
        arg = b->erased;
        break;
    }
    case BOOT_DATA:
        st = data(b, f);
        arg = b->seq | b->written << 8;
        break;
    case BOOT_DONE:
        st = done(b, f);
        arg = b->written;
        break;
    case BOOT_RUN:
        st = boot_image_valid(io) ? BOOT_OK : BOOT_NO_IMAGE;
        break;
    default:
        st = BOOT_BAD_OP;
    }
    boot_reply(io, b->node, op, st, arg);
    return op == BOOT_RUN && st == BOOT_OK;
}
