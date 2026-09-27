#include "tof_sim.h"

#include <string.h>

#define OSC_PLL 0x01A8

void tof_sim_init(struct tof_sim *s)
{
    memset(s, 0, sizeof(*s));
    s->present = true;
    s->reg[0x00E5] = 0x01;
    s->reg[0x010F] = 0xEA;
    s->reg[0x0110] = 0xCC;
    s->reg[0x00DE] = OSC_PLL >> 8;
    s->reg[0x00DF] = OSC_PLL & 0xff;
    s->distance_mm = 400;
    s->raw_status = 9;
}

static uint32_t period_ms(struct tof_sim *s)
{
    uint32_t v = (uint32_t)s->reg[0x6C] << 24 | s->reg[0x6D] << 16 | s->reg[0x6E] << 8 | s->reg[0x6F];
    return v ? (uint32_t)(v / (OSC_PLL * 1.075f) + 0.5f) : 100;
}

static void update_gpio(struct tof_sim *s)
{
    uint8_t active = !(s->reg[0x0030] & 0x10);
    s->reg[0x0031] = (s->reg[0x0031] & ~1) | (s->ready ? active : !active);
}

void tof_sim_step(struct tof_sim *s, uint32_t ms)
{
    while (ms--) {
        s->t_ms++;
        if (s->ranging && !s->ready && s->t_ms >= s->next_ms) {
            s->ready = true;
            s->next_ms += period_ms(s);
            s->reg[0x0089] = s->raw_status;
            s->reg[0x0096] = s->distance_mm >> 8;
            s->reg[0x0097] = s->distance_mm & 0xff;
        }
    }
    update_gpio(s);
}

bool tof_sim_write(void *ctx, uint16_t reg, const uint8_t *data, int n)
{
    struct tof_sim *s = ctx;
    if (!s->present || reg + n > (int)sizeof(s->reg))
        return false;
    memcpy(&s->reg[reg], data, n);
    for (int i = 0; i < n; i++) {
        if (reg + i == 0x0086 && (data[i] & 1))
            s->ready = false;
        if (reg + i == 0x0087) {
            s->ranging = data[i] == 0x40;
            s->next_ms = s->t_ms + period_ms(s);
        }
    }
    update_gpio(s);
    return true;
}

bool tof_sim_read(void *ctx, uint16_t reg, uint8_t *data, int n)
{
    struct tof_sim *s = ctx;
    if (!s->present || reg + n > (int)sizeof(s->reg))
        return false;
    memcpy(data, &s->reg[reg], n);
    return true;
}

void tof_sim_delay(void *ctx, uint32_t ms) { tof_sim_step(ctx, ms); }
