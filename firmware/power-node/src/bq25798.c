#include "bq25798.h"

static bool wr(struct bq25798 *c, uint8_t reg, const uint8_t *d, int n)
{
    uint8_t buf[5] = {reg};
    for (int i = 0; i < n; i++)
        buf[1 + i] = d[i];
    if (c->hal->i2c_write(c->hal->ctx, BQ25798_ADDR, buf, n + 1))
        return true;
    c->errors++;
    return false;
}

static bool rd(struct bq25798 *c, uint8_t reg, uint8_t *d, int n)
{
    if (c->hal->i2c_read(c->hal->ctx, BQ25798_ADDR, reg, d, n))
        return true;
    c->errors++;
    return false;
}

static bool wr8(struct bq25798 *c, uint8_t reg, uint8_t v) { return wr(c, reg, &v, 1); }

static bool wr16(struct bq25798 *c, uint8_t reg, uint16_t v)
{
    uint8_t d[2] = {v >> 8, v & 0xff};
    return wr(c, reg, d, 2);
}

static uint16_t be16(const uint8_t *p) { return (uint16_t)(p[0] << 8 | p[1]); }

bool bq25798_apply(struct bq25798 *c, const struct charge_settings *s)
{
    c->set = *s;
    c->applied = wr16(c, CHG_VREG, s->charge_mv / 10) &&
                 wr16(c, CHG_ICHG, s->charge_ma / 10) &&
                 wr16(c, CHG_IINDPM, s->input_ma / 10) &&
                 /* EN_AUTO_IBATDIS, EN_CHG, EN_TERM */
                 wr8(c, CHG_CTRL0, 0x82 | (s->enable ? 0x20 : 0)) &&
                 /* backup 80 %, VAC_OVP 26 V (the only step above a 20 V input), watchdog 40 s */
                 wr8(c, CHG_CTRL1, 0x85) &&
                 /* no NTC on the charger: the BMS watches the cell temperatures */
                 wr8(c, CHG_NTC1, 0x55) &&
                 /* ADC on, continuous, 15 bit */
                 wr8(c, CHG_ADC_CTRL, 0x80);
    return c->applied;
}

bool bq25798_poll(struct bq25798 *c, struct charger_reading *r)
{
    uint8_t st[7], adc[14], vreg[2], ctrl1;
    if (!rd(c, CHG_STATUS0, st, 7) || !rd(c, CHG_VREG, vreg, 2))
        return false;
    /* WD_STAT, or settings back at their defaults after a charger reset */
    if (!c->applied || (st[0] & 0x20) || be16(vreg) != c->set.charge_mv / 10) {
        c->resets++;
        if (!bq25798_apply(c, &c->set))
            return false;
    }
    if (!rd(c, CHG_CTRL1, &ctrl1, 1) || !wr8(c, CHG_CTRL1, ctrl1 | 0x08)) /* WD_RST */
        return false;
    if (!rd(c, CHG_IBUS_ADC, adc, 14))
        return false;
    r->status0 = st[0];
    r->chg_stat = st[1] >> 5;
    r->fault0 = st[5];
    r->fault1 = st[6];
    r->vac1 = st[0] & 0x02;
    r->vac2 = st[0] & 0x04;
    r->power_good = st[0] & 0x08;
    r->ibus_ma = (int16_t)be16(adc);
    r->ibat_ma = (int16_t)be16(adc + 2);
    r->vbus_mv = be16(adc + 4);
    r->vac1_mv = be16(adc + 6);
    r->vac2_mv = be16(adc + 8);
    r->vbat_mv = be16(adc + 10);
    return true;
}
