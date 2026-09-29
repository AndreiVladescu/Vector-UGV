#include "power_sim.h"

#include <string.h>

static void chips_step(struct power_sim *s, uint32_t ms)
{
    s->t += ms;
    bq76942_sim_step(&s->bms, ms);
    bq25798_sim_step(&s->chg, ms);
    /* a charger on PACK+ holds LD up, as on the board */
    s->bms.ld_high = s->chg.vac1 || s->chg.vac2;
}

static uint32_t now_ms(void *ctx) { return ((struct power_sim *)ctx)->t; }
static void delay_ms(void *ctx, uint32_t ms) { chips_step(ctx, ms); }

static bool i2c_write(void *ctx, uint8_t addr, const uint8_t *d, int n)
{
    struct power_sim *s = ctx;
    if (addr == BQ76942_ADDR)
        return bq76942_sim_write(&s->bms, d, n);
    if (addr == BQ25798_ADDR)
        return bq25798_sim_write(&s->chg, d, n);
    return false;
}

static bool i2c_read(void *ctx, uint8_t addr, uint8_t reg, uint8_t *d, int n)
{
    struct power_sim *s = ctx;
    if (addr == BQ76942_ADDR)
        return bq76942_sim_read(&s->bms, reg, d, n);
    if (addr == BQ25798_ADDR)
        return bq25798_sim_read(&s->chg, reg, d, n);
    return false;
}

static void out(void *ctx, enum pwr_out pin, bool on) { ((struct power_sim *)ctx)->outs[pin] = on; }

static bool in(void *ctx, enum pwr_in pin)
{
    struct power_sim *s = ctx;
    switch (pin) {
    case IN_BUTTON: return s->button;
    case IN_HALTED: return s->halted;
    case IN_ESTOP: return s->estop_hw || s->outs[OUT_RUN_LOW];
    case IN_5V_PG: return !s->outs[OUT_5V_OFF] && (bq76942_sim_fets(&s->bms) & BQ_FET_DSG);
    default: return false;
    }
}

static void send(void *ctx, const struct can_frame_t *f)
{
    struct power_sim *s = ctx;
    if (s->tx)
        s->tx(s->user, f);
}

static bool save(void *ctx, int slot, const void *data, int len)
{
    struct power_sim *s = ctx;
    if (s->flash_fail || slot < 0 || slot > 1 || len != sizeof(struct power_config))
        return false;
    memcpy(&s->flash[slot], data, len);
    s->saves++;
    return true;
}

static uint32_t diag(void *ctx, uint8_t key)
{
    (void)ctx;
    return key == KEY_VERSION ? 0x1234567u : 0;
}

void power_sim_init(struct power_sim *s, bool crc)
{
    memset(s, 0, sizeof(*s));
    bq76942_sim_init(&s->bms, crc);
    bq25798_sim_init(&s->chg);
    s->hal = (struct power_hal){s, now_ms, delay_ms, i2c_write, i2c_read, out, in, send, save, diag};
    s->t = 1;
}

void power_sim_boot(struct power_sim *s)
{
    power_init(&s->p, &s->hal, 0x0207, s->bms.crc, &s->flash[0], &s->flash[1]);
}

bool power_sim_mcu_on(const struct power_sim *s) { return s->bms.awake; }

void power_sim_run(struct power_sim *s, uint32_t ms)
{
    while (ms--) {
        chips_step(s, 1);
        if (power_sim_mcu_on(s))
            power_tick(&s->p);
    }
}
