#include "bq25798_sim.h"

#include <string.h>

#include "bq25798.h"

static void put16(struct bq25798_sim *s, uint8_t reg, uint16_t v)
{
    s->regs[reg] = v >> 8;
    s->regs[reg + 1] = v & 0xff;
}

uint16_t bq25798_sim_reg16(const struct bq25798_sim *s, uint8_t reg)
{
    return (uint16_t)(s->regs[reg] << 8 | s->regs[reg + 1]);
}

/* what POR and a watchdog expiry leave behind with PROG set for 4S */
static void defaults(struct bq25798_sim *s)
{
    put16(s, CHG_VREG, 1680);
    put16(s, CHG_ICHG, 100);
    put16(s, CHG_IINDPM, 300);
    s->regs[CHG_CTRL0] = 0xA2;
    s->regs[CHG_CTRL1] = 0x85;
    s->regs[CHG_NTC1] = 0x54;
    s->regs[CHG_ADC_CTRL] = 0x30;
}

void bq25798_sim_init(struct bq25798_sim *s)
{
    memset(s, 0, sizeof(*s));
    defaults(s);
    s->vbat_mv = 15200;
}

bool bq25798_sim_write(struct bq25798_sim *s, const uint8_t *d, int n)
{
    if (s->fail_next > 0) {
        s->fail_next--;
        return false;
    }
    for (int i = 1; i < n && d[0] + i - 1 < (int)sizeof(s->regs); i++) {
        uint8_t reg = (uint8_t)(d[0] + i - 1);
        if (reg == CHG_CTRL1 && (d[i] & 0x08)) {
            s->wd_ms = 0;
            s->regs[CHG_STATUS0] &= ~0x20;
        }
        s->regs[reg] = reg == CHG_CTRL1 ? d[i] & ~0x08 : d[i];
    }
    return true;
}

bool bq25798_sim_read(struct bq25798_sim *s, uint8_t reg, uint8_t *out, int n)
{
    if (s->fail_next > 0) {
        s->fail_next--;
        return false;
    }
    for (int i = 0; i < n; i++)
        out[i] = reg + i < (int)sizeof(s->regs) ? s->regs[reg + i] : 0;
    return true;
}

void bq25798_sim_step(struct bq25798_sim *s, uint32_t ms)
{
    uint8_t wd = s->regs[CHG_CTRL1] & 7;
    static const uint32_t wd_limit[8] = {0, 500, 1000, 2000, 20000, 40000, 80000, 160000};
    s->wd_ms += ms;
    if (wd && s->wd_ms >= wd_limit[wd]) {
        defaults(s);
        s->regs[CHG_STATUS0] |= 0x20;
        s->wd_ms = 0;
        s->wd_expired++;
    }
    bool in = s->vbus;
    uint8_t st0 = s->regs[CHG_STATUS0] & 0x20;
    if (in) st0 |= 0x0F; /* VBUS, AC1, AC2 present (tied together), power good */
    s->regs[CHG_STATUS0] = st0;
    uint8_t stat = !in || !(s->regs[CHG_CTRL0] & 0x20) ? CHG_NOT_CHARGING : s->done ? CHG_DONE : CHG_FAST;
    s->regs[CHG_STATUS1] = (uint8_t)(stat << 5);
    bool adc = s->regs[CHG_ADC_CTRL] & 0x80;
    put16(s, 0x35, adc && in ? s->vbus_mv : 0);
    put16(s, 0x37, adc && in ? s->vbus_mv : 0);
    put16(s, 0x39, adc && in ? s->vbus_mv : 0);
    put16(s, 0x3B, adc ? s->vbat_mv : 0);
}
