#include "flash_sim.h"

#include <string.h>

static bool erase(void *ctx, uint32_t offset, uint32_t len)
{
    struct flash_sim *s = ctx;
    if (offset % 2048 || len % 2048 || offset + len > FLASH_SIM_SIZE)
        return false;
    memset(&s->mem[offset], 0xff, len);
    s->erases += len / 2048;
    return true;
}

static bool program(void *ctx, uint32_t offset, const uint8_t data[8])
{
    struct flash_sim *s = ctx;
    if (offset % 8 || offset + 8 > FLASH_SIM_SIZE)
        return false;
    for (int i = 0; i < 8; i++)
        if (s->mem[offset + i] != 0xff)
            return false;
    memcpy(&s->mem[offset], data, 8);
    s->programs++;
    return true;
}

static void send(void *ctx, const struct can_frame_t *f)
{
    struct flash_sim *s = ctx;
    if (s->tx)
        s->tx(s->user, f);
}

void flash_sim_init(struct flash_sim *s)
{
    memset(s, 0, sizeof(*s));
    memset(s->mem, 0xff, sizeof(s->mem));
    s->io = (struct boot_io){
        .ctx = s, .image = s->mem, .size = FLASH_SIM_SIZE, .page = 2048,
        .sram_lo = 0x20000000u, .sram_hi = 0x20007800u, .image_addr = FLASH_SIM_ADDR,
        .erase = erase, .program = program, .send = send,
    };
}
