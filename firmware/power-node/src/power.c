#include "power.h"

#include <string.h>

#define BOOT_RETRY_MS 200
#define BOOT_TRIES 10
#define FAULT_RETRY_MS 5000
#define BMS_POLL_MS 100
#define CHG_POLL_MS 1000
#define TX_MS 100
#define SLOW_TX_MS 1000
#define SYNC_STALE_MS 500
#define HALTED_MS 1000
#define NO_INPUT_MS 5000
#define PG_BAD_MS 500
#define OFF_RETRY_MS 3000
#define BMS_FAIL_LIMIT 5

static uint32_t now(struct power *p) { return p->hal->now_ms(p->hal->ctx); }
static void out(struct power *p, enum pwr_out pin, bool on) { p->hal->out(p->hal->ctx, pin, on); }
static bool in(struct power *p, enum pwr_in pin) { return p->hal->in(p->hal->ctx, pin); }

/* true once cond has held for ms; *since = 0 while it doesn't */
static bool held(struct power *p, bool cond, uint32_t *since, uint32_t ms)
{
    uint32_t t = now(p);
    if (!cond) {
        *since = 0;
        return false;
    }
    if (!*since)
        *since = t ? t : 1;
    return t - *since >= ms;
}

static void reply(struct power *p, uint8_t key, int32_t value, uint8_t seq, uint8_t status)
{
    struct leg_cfg_msg m = {.joint = 0xff, .key = key, .value = value, .seq = seq, .op = status};
    struct can_frame_t f;
    can_pack_leg_cfg(&f, CAN_LEG_REPLY, POWER_NODE, &m);
    p->hal->send(p->hal->ctx, &f);
}

static uint16_t cell_min(const struct bq_reading *r)
{
    uint16_t v = r->cell_mv[0];
    for (int i = 1; i < BQ_CELLS; i++)
        if (r->cell_mv[i] < v)
            v = r->cell_mv[i];
    return v;
}

static uint16_t cell_max(const struct bq_reading *r)
{
    uint16_t v = r->cell_mv[0];
    for (int i = 1; i < BQ_CELLS; i++)
        if (r->cell_mv[i] > v)
            v = r->cell_mv[i];
    return v;
}

static bool charger_in(struct power *p) { return p->ch_ok && p->ch.vbus; }

static struct charge_settings charge_settings(struct power *p)
{
    return (struct charge_settings){p->cfg.charge_mv, p->cfg.charge_ma, p->cfg.input_ma, true};
}

/* write over the older slot; the other keeps the last good copy if power drops now */
static bool save(struct power *p)
{
    p->cfg.soc = p->gauge.soc;
    p->cfg.seq++;
    config_seal(&p->cfg);
    bool ok = p->hal->save(p->hal->ctx, p->cfg.seq & 1, &p->cfg, sizeof(p->cfg));
    if (!ok) {
        p->cfg.seq--;
        config_seal(&p->cfg);
    }
    return ok;
}

static void enter(struct power *p, enum power_state s)
{
    p->state = s;
    p->entered = now(p);
    switch (s) {
    case POWER_ON:
        out(p, OUT_SHUTDOWN_REQ, false);
        out(p, OUT_CM5_OFF, false);
        bq_fets(&p->bms, true);
        p->low_t = p->halted_t = p->pg_bad_t = 0;
        break;
    case POWER_CHARGE:
        /* CHG and DSG both on, so the charge current doesn't go through the DSG FET's body
           diode. The side boards see VBAT too: their MCUs idle with the servo bucks off. */
        out(p, OUT_SHUTDOWN_REQ, false);
        out(p, OUT_ESTOP, false);
        out(p, OUT_CM5_OFF, true);
        bq_fets(&p->bms, true);
        p->no_input_t = 0;
        break;
    case POWER_HALTING:
        out(p, OUT_SHUTDOWN_REQ, true);
        p->halted_t = 0;
        break;
    case POWER_OFF:
        out(p, OUT_SHUTDOWN_REQ, false);
        out(p, OUT_ESTOP, false);
        out(p, OUT_CM5_OFF, true);
        save(p);
        bq_fets(&p->bms, false);
        p->off_t = 0;
        break;
    case POWER_FAULT:
        out(p, OUT_CM5_OFF, true);
        break;
    case POWER_BOOT:
        p->boot_tries = 0;
        break;
    }
}

static void boot_step(struct power *p)
{
    bool changed;
    if (!bq_configure(&p->bms, &changed) || !bq_read(&p->bms, &p->bat)) {
        if (++p->boot_tries >= BOOT_TRIES) {
            p->faults |= PWR_FAULT_BMS_COMM;
            enter(p, POWER_FAULT);
        }
        return;
    }
    p->bat_ok = true;
    p->bms_fails = 0;
    p->faults &= ~PWR_FAULT_BMS_COMM;
    struct charge_settings cs = charge_settings(p);
    p->ch_ok = bq25798_apply(&p->chg, &cs) && bq25798_poll(&p->chg, &p->ch);

    uint32_t sum = 0;
    for (int i = 0; i < BQ_CELLS; i++)
        sum += p->bat.cell_mv[i];
    gauge_start(&p->gauge, p->cfg.capacity_mah, p->cfg.soc, (uint16_t)(sum / BQ_CELLS));

    /* an MCU reset while running finds the BMS configured and the pack on: carry on */
    bool running = !changed && (p->bat.fets & BQ_FET_DSG);
    if (in(p, IN_BUTTON))
        p->long_done = true; /* the press that woke us isn't a shutdown request */
    if (running || in(p, IN_BUTTON))
        enter(p, POWER_ON);
    else if (charger_in(p))
        enter(p, POWER_CHARGE);
    else
        enter(p, POWER_OFF);
}

static void update_faults(struct power *p)
{
    uint8_t f = p->faults & (PWR_FAULT_CONFIG | PWR_FAULT_5V);
    uint8_t fl = 0;
    if (p->bms_fails >= BMS_FAIL_LIMIT)
        f |= PWR_FAULT_BMS_COMM;
    if (p->bat_ok) {
        const struct bq_reading *r = &p->bat;
        uint16_t lo = cell_min(r), hi = cell_max(r);
        if (r->safety_a || r->safety_b || r->safety_c)
            f |= PWR_FAULT_BMS;
        if (lo < p->cfg.low_mv)
            f |= PWR_FAULT_LOW_CELL;
        if (lo > 3000 && hi - lo > 100)
            f |= PWR_FAULT_IMBALANCE;
        if (r->cell_temp_c > 50 || r->fet_temp_c > 80)
            f |= PWR_FAULT_HOT;
        if (p->gauge.soc < 15 || lo < p->cfg.low_mv + 100)
            fl |= PWR_LOW;
    }
    if (!p->ch_ok || (p->ch.fault0 & 0x7f) || p->ch.fault1) /* bit 7 is regulation, not a fault */
        f |= PWR_FAULT_CHARGER;
    if (charger_in(p))
        fl |= PWR_CHARGER;
    if (p->ch_ok && p->ch.chg_stat != CHG_NOT_CHARGING && p->ch.chg_stat != CHG_DONE)
        fl |= PWR_CHARGING;
    if (in(p, IN_ESTOP))
        fl |= PWR_ESTOP;
    p->faults = f;
    p->flags = fl;
}

static void poll_bms(struct power *p)
{
    if (bq_read(&p->bms, &p->bat)) {
        p->bms_fails = 0;
        p->bat_ok = true;
    } else if (p->bms_fails < 255 && ++p->bms_fails >= BMS_FAIL_LIMIT) {
        p->bat_ok = false;
    }
}

static void poll_charger(struct power *p)
{
    p->ch_ok = bq25798_poll(&p->chg, &p->ch);
    if (!p->ch_ok)
        memset(&p->ch, 0, sizeof(p->ch));
    float q;
    if (bq_passed_charge(&p->bms, &q))
        gauge_update(&p->gauge, q);
    if (p->ch_ok && p->ch.chg_stat == CHG_DONE)
        gauge_set(&p->gauge, 100);
}

static void send_state(struct power *p)
{
    struct can_frame_t f;
    struct power_state_msg m = {
        .pack_mv = p->bat.stack_mv, .current_ma = p->bat.current_ma,
        .soc_half = (uint8_t)(p->gauge.soc * 2 + 0.5f), .temp_c = (int8_t)p->bat.cell_temp_c,
        .state = p->state, .flags = p->flags, .faults = p->faults,
    };
    can_pack_power_state(&f, &m);
    p->hal->send(p->hal->ctx, &f);
}

static void send_slow(struct power *p)
{
    struct can_frame_t f;
    struct power_cells_msg c;
    for (int i = 0; i < BQ_CELLS; i++)
        c.cell_mv[i] = p->bat.cell_mv[i];
    can_pack_power_cells(&f, &c);
    p->hal->send(p->hal->ctx, &f);

    struct power_detail_msg d = {
        .safety_a = p->bat.safety_a, .safety_b = p->bat.safety_b, .safety_c = p->bat.safety_c,
        .fets = p->bat.fets & 0x0f,
        .usb = p->ch.vbus,
        .chg_stat = p->ch.chg_stat, .charger_fault = p->ch.fault0 & 0x7f,
        .input_mv = p->ch.vbus_mv,
        .fet_temp_c = (int8_t)p->bat.fet_temp_c,
    };
    can_pack_power_detail(&f, &d);
    p->hal->send(p->hal->ctx, &f);
}

static void on_press(struct power *p)
{
    if (p->state == POWER_CHARGE || p->state == POWER_OFF)
        enter(p, POWER_ON);
}

static void on_long_press(struct power *p)
{
    switch (p->state) {
    case POWER_ON: enter(p, POWER_HALTING); break;
    case POWER_HALTING: /* the CM5 isn't halting: cut it */
    case POWER_FAULT: enter(p, POWER_OFF); break;
    default: break;
    }
}

static void button(struct power *p)
{
    uint32_t t = now(p);
    bool b = in(p, IN_BUTTON);
    if (b && !p->pressed) {
        p->pressed = true;
        p->press_t = t;
        p->long_done = false;
        on_press(p);
    } else if (!b && p->pressed) {
        p->pressed = false;
    }
    if (b && !p->long_done && t - p->press_t >= LONG_PRESS_MS) {
        p->long_done = true;
        on_long_press(p);
    }
}

static void after_halt(struct power *p)
{
    enter(p, charger_in(p) ? POWER_CHARGE : POWER_OFF);
}

static void state_step(struct power *p)
{
    uint32_t t = now(p);
    switch (p->state) {
    case POWER_ON: {
        /* ROS's e-stop (the SYNC flag) pulls ESTOP_N, which holds every servo buck off in
           hardware. A stale SYNC lets go: the legs crouch and power down on their own then. */
        out(p, OUT_ESTOP, p->sync_estop && t - p->last_sync < SYNC_STALE_MS);
        if (held(p, in(p, IN_HALTED), &p->halted_t, HALTED_MS)) {
            after_halt(p);
            break;
        }
        bool low = p->bat_ok && (cell_min(&p->bat) < p->cfg.low_mv || p->gauge.soc < 2);
        if (held(p, low, &p->low_t, LOW_CELL_MS)) {
            enter(p, POWER_HALTING);
            break;
        }
        bool pg_bad = t - p->entered > 1000 && !in(p, IN_5V_PG);
        if (held(p, pg_bad, &p->pg_bad_t, PG_BAD_MS))
            p->faults |= PWR_FAULT_5V;
        else if (!pg_bad)
            p->faults &= ~PWR_FAULT_5V;
        break;
    }
    case POWER_HALTING:
        out(p, OUT_ESTOP, p->sync_estop && t - p->last_sync < SYNC_STALE_MS);
        if (held(p, in(p, IN_HALTED), &p->halted_t, HALTED_MS) || t - p->entered >= HALT_TIMEOUT_MS)
            after_halt(p);
        break;
    case POWER_CHARGE:
        if (held(p, !charger_in(p), &p->no_input_t, NO_INPUT_MS))
            enter(p, POWER_OFF);
        break;
    case POWER_OFF:
        /* TS2 low (the button) would wake the BMS straight back up; a charger on LD keeps it
           awake, and then we charge */
        if (charger_in(p)) {
            enter(p, POWER_CHARGE);
        } else if (!in(p, IN_BUTTON) && (!p->off_t || t - p->off_t >= OFF_RETRY_MS)) {
            p->off_t = t ? t : 1;
            bq_shutdown(&p->bms);
        }
        break;
    case POWER_FAULT:
        if (t - p->entered >= FAULT_RETRY_MS)
            enter(p, POWER_BOOT);
        break;
    case POWER_BOOT:
        break;
    }
}

static void led(struct power *p, uint32_t t)
{
    bool on;
    switch (p->state) {
    case POWER_ON: on = !(p->flags & PWR_LOW) || t % 1000 < 900; break;
    case POWER_CHARGE: on = (p->flags & PWR_CHARGING) ? t % 2000 < 1000 : true; break;
    case POWER_HALTING: on = t % 400 < 200; break;
    case POWER_FAULT: on = t % 200 < 100; break;
    default: on = false;
    }
    out(p, OUT_LED, on);
}

void power_init(struct power *p, const struct power_hal *hal, uint16_t cell_mask, bool bms_crc,
                const struct power_config *slot0, const struct power_config *slot1)
{
    memset(p, 0, sizeof(*p));
    p->hal = hal;
    const struct power_config *c = config_newest(slot0, slot1);
    if (c) {
        p->cfg = *c;
    } else {
        config_defaults(&p->cfg);
        p->faults |= PWR_FAULT_CONFIG;
    }
    p->bms = (struct bq76942){.hal = hal, .crc = bms_crc, .cell_mask = cell_mask, .cell_ntc = p->cfg.cell_ntc};
    p->chg = (struct bq25798){.hal = hal};
    for (int i = 0; i < OUTPUTS; i++)
        out(p, (enum pwr_out)i, false);
    uint32_t t = now(p);
    p->t_bms = p->t_chg = p->t_tx = p->t_slow = t;
    enter(p, POWER_BOOT);
    p->entered = t - BOOT_RETRY_MS; /* first try straight away */
}

void power_tick(struct power *p)
{
    uint32_t t = now(p);
    button(p);
    led(p, t);
    if (p->state == POWER_BOOT) {
        if (t - p->entered >= BOOT_RETRY_MS) {
            p->entered = t;
            boot_step(p);
        }
        return;
    }
    if (t - p->t_bms >= BMS_POLL_MS) {
        p->t_bms = t;
        poll_bms(p);
    }
    if (t - p->t_chg >= CHG_POLL_MS) {
        p->t_chg = t;
        poll_charger(p);
    }
    update_faults(p);
    state_step(p);
    if (t - p->t_tx >= TX_MS) {
        p->t_tx = t;
        send_state(p);
    }
    if (t - p->t_slow >= SLOW_TX_MS) {
        p->t_slow = t;
        send_slow(p);
    }
}

/* ---- config over CAN ---- */

static bool key_get(struct power *p, uint8_t key, int32_t arg, int32_t *v)
{
    switch (key) {
    case KEY_VERSION:
    case KEY_RESET_CAUSE:
    case KEY_CAN_ERRORS: *v = (int32_t)p->hal->diag(p->hal->ctx, key); return true;
    case KEY_UPTIME: *v = (int32_t)(now(p) / 1000); return true;
    case PKEY_CAPACITY: *v = p->cfg.capacity_mah; return true;
    case PKEY_CHARGE_MA: *v = p->cfg.charge_ma; return true;
    case PKEY_CHARGE_MV: *v = p->cfg.charge_mv; return true;
    case PKEY_INPUT_MA: *v = p->cfg.input_ma; return true;
    case PKEY_LOW_MV: *v = p->cfg.low_mv; return true;
    case PKEY_CELL_NTC: *v = p->cfg.cell_ntc; return true;
    case PKEY_SOC: *v = (int32_t)(p->gauge.soc * 10 + 0.5f); return true;
    case PKEY_SHUTDOWN: *v = p->state == POWER_HALTING; return true;
    case PKEY_BMS_MEM: {
        uint8_t d[4];
        if (arg < 0x9180 || arg > 0x93ff || !bq_mem_read(&p->bms, (uint16_t)arg, d, 4))
            return false;
        *v = (int32_t)(d[0] | d[1] << 8 | d[2] << 16 | (uint32_t)d[3] << 24);
        return true;
    }
    }
    return false;
}

static uint8_t range(int32_t v, int32_t lo, int32_t hi, uint16_t *field)
{
    if (v < lo || v > hi)
        return ST_BAD_VALUE;
    *field = (uint16_t)v;
    return ST_OK;
}

static uint8_t key_set(struct power *p, uint8_t key, int32_t v)
{
    uint8_t st;
    switch (key) {
    case PKEY_CAPACITY:
        st = range(v, 1000, 30000, &p->cfg.capacity_mah);
        p->gauge.capacity_mah = p->cfg.capacity_mah;
        return st;
    case PKEY_CHARGE_MA: st = range(v, 100, 5000, &p->cfg.charge_ma); break;
    case PKEY_CHARGE_MV: st = range(v, 12000, 16800, &p->cfg.charge_mv); break;
    case PKEY_INPUT_MA: st = range(v, 100, 3300, &p->cfg.input_ma); break;
    case PKEY_LOW_MV: return range(v, 3000, 3700, &p->cfg.low_mv);
    case PKEY_CELL_NTC:
        /* the BMS takes it at the next power-up: reconfiguring it now would open the FETs */
        return range(v, 0, 1, &p->cfg.cell_ntc);
    case PKEY_SOC:
        if (v < 0 || v > 1000)
            return ST_BAD_VALUE;
        gauge_set(&p->gauge, v / 10.0f);
        return ST_OK;
    case PKEY_SHUTDOWN:
        if (v != 1)
            return ST_BAD_VALUE;
        if (p->state != POWER_ON)
            return ST_BUSY;
        enter(p, POWER_HALTING);
        return ST_OK;
    default:
        return ST_BAD_KEY;
    }
    if (st == ST_OK) {
        struct charge_settings cs = charge_settings(p);
        bq25798_apply(&p->chg, &cs);
    }
    return st;
}

static void config_frame(struct power *p, const struct leg_cfg_msg *m)
{
    int32_t v = 0;
    if (m->joint != 0xff) {
        reply(p, m->key, 0, m->seq, ST_BAD_KEY);
        return;
    }
    switch (m->op) {
    case OP_READ:
        if (key_get(p, m->key, m->value, &v))
            reply(p, m->key, v, m->seq, ST_OK);
        else
            reply(p, m->key, 0, m->seq, ST_BAD_KEY);
        break;
    case OP_WRITE: {
        uint8_t st = key_set(p, m->key, m->value);
        config_seal(&p->cfg);
        reply(p, m->key, m->value, m->seq, st);
        break;
    }
    case OP_SAVE:
        reply(p, 0, 0, m->seq, save(p) ? ST_OK : ST_BAD_VALUE);
        break;
    case OP_DEFAULTS: {
        uint32_t seq = p->cfg.seq; /* or the next save would look older than what's in flash */
        config_defaults(&p->cfg);
        p->cfg.seq = seq;
        config_seal(&p->cfg);
        p->gauge.capacity_mah = p->cfg.capacity_mah;
        struct charge_settings cs = charge_settings(p);
        bq25798_apply(&p->chg, &cs);
        reply(p, 0, 0, m->seq, ST_OK);
        break;
    }
    default:
        reply(p, m->key, 0, m->seq, ST_BAD_KEY);
    }
}

void power_frame(struct power *p, const struct can_frame_t *f)
{
    struct sync_msg s;
    struct leg_cfg_msg m;
    if (can_unpack_sync(f, &s)) {
        p->sync_estop = s.estop;
        p->last_sync = now(p);
    } else if (f->id == (CAN_LEG_CONFIG | POWER_NODE) && can_unpack_leg_cfg(f, &m)) {
        config_frame(p, &m);
    }
}
