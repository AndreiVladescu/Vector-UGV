#include "leg.h"

#include <math.h>
#include <string.h>

#include "board.h"

#define SAMPLES 16
#define RESUME_MARGIN_US 30.0f
#define WAKE_SETTLE_MS 150
#define WAKE_BUCK_MS 60
#define TEST_DEG 5.0f
#define TEST_TOL_DEG 2.0f
#define TEST_TIMEOUT_MS 400
#define WAKE_TRIES 3
#define SYNC_TIMEOUT_MS 100
#define CROUCH_MS 2000
#define CROUCH_SPEED 30.0f /* deg/s */
#define MAX_SPEED 400.0f
#define IDLE_TX_MS 50
#define CAL_START_MS 800
#define CAL_SETTLE_MS 400
#define CAL_MAX_ERR_MV 60.0f
#define CAL_MIN_SPAN_MV 200.0f
#define OVERLOAD_MA 5000   /* three MG996Rs stalled draw ~7.5 A */
#define OVERLOAD_MS 500
#define HOT_C 85           /* NTC by the buck inductor */
#define COOL_C 70
#define PG_LOSS_MS 20
#define I_ZERO_AFTER_MS 100 /* after the buck turns off, before the amp output counts as zero */
#define TEST_OFF_MS 150
#define TEST_ON_MS 200

static const float crouch_deg[JOINTS] = {0.0f, 40.0f, -110.0f};

static uint32_t now(struct leg *l) { return l->hal->now_ms(l->hal->ctx); }

static void pwm(struct leg *l, int j, float us)
{
    if (us > 0) {
        if (us < 500) us = 500;
        if (us > 2500) us = 2500;
    }
    l->out_us[j] = us;
    l->hal->pwm_us(l->hal->ctx, j, (int)lroundf(us));
}

static void power(struct leg *l, bool on)
{
    if (on && !l->powered)
        l->power_t = now(l);
    if (!on && l->powered)
        l->off_t = now(l);
    l->powered = on;
    l->hal->buck(l->hal->ctx, on);
}

static void all_off(struct leg *l)
{
    for (int j = 0; j < JOINTS; j++)
        pwm(l, j, 0);
    power(l, false);
}

static void enter(struct leg *l, enum leg_state s)
{
    l->state = s;
    l->entered = now(l);
    if (s == LEG_OFF || s == LEG_FAULT)
        all_off(l);
    if (s == LEG_WAKE) {
        memset(&l->wake, 0, sizeof(l->wake));
        l->wake.joint = -1;
        power(l, true);
    }
}

static void reply(struct leg *l, uint8_t joint, uint8_t key, int32_t value, uint8_t seq, uint8_t status)
{
    struct can_frame_t f;
    struct leg_cfg_msg m = {joint, key, value, seq, status};
    can_pack_leg_cfg(&f, CAN_LEG_REPLY, l->node, &m);
    l->hal->send(l->hal->ctx, &f);
}

static float sample(struct leg *l, int ch, int n)
{
    uint16_t buf[64];
    l->hal->adc_mv(l->hal->ctx, ch, buf, n);
    return median_u16(buf, n);
}

static void read_inputs(struct leg *l)
{
    /* the pots are dead while the servos are unpowered: keep the last good angle */
    bool pots_live = l->powered && now(l) - l->power_t >= WAKE_BUCK_MS;
    for (int j = 0; j < JOINTS; j++) {
        l->mv[j] = sample(l, ADC_POT0 + j, SAMPLES);
        const struct joint_cfg *c = &l->cfg.joint[j];
        if (pots_live)
            l->pos_deg[j] = joint_us_to_deg(c, joint_mv_to_us(c, l->mv[j]));
    }
    /* the amp's output offset is whatever it reads with the buck off, tracked all the time */
    float i_mv = sample(l, ADC_I_LEG, 5);
    if (!l->powered && now(l) - l->off_t >= I_ZERO_AFTER_MS)
        l->i_zero_mv += (i_mv - l->i_zero_mv) * 0.05f;
    l->current_ma = (uint16_t)(fmaxf(0.0f, i_mv - l->i_zero_mv) * 1000.0f / (INA_GAIN * SHUNT_MOHM));
    l->vbat_mv = (uint16_t)(sample(l, ADC_VBAT, 5) * VBAT_RATIO * l->cfg.vbat_gain);
    l->rail_mv = (uint16_t)(sample(l, ADC_6V0, 5) * V6_RATIO * l->cfg.rail_gain);

    float v = sample(l, ADC_NTC, 5);
    if (v > 1.0f && v < VDDA_MV - 1.0f) {
        float r = NTC_PULLUP * v / (VDDA_MV - v);
        float c = 1.0f / (1.0f / 298.15f + logf(r / NTC_R25) / NTC_BETA) - 273.15f;
        l->temp_c = (int8_t)lroundf(fmaxf(-40.0f, fminf(125.0f, c))); /* a shorted NTC reads hot */
    }
}

static void send_state(struct leg *l, bool status)
{
    struct can_frame_t f;
    struct leg_state_msg s = {{l->pos_deg[0], l->pos_deg[1], l->pos_deg[2]}, l->current_ma};
    can_pack_leg_state(&f, l->node, &s);
    l->hal->send(l->hal->ctx, &f);
    if (!status)
        return;
    struct leg_status_msg st = {
        .tof_mm = l->hal->tof_mm ? l->hal->tof_mm(l->hal->ctx) : 0,
        .vbat_mv = l->vbat_mv, .rail_mv = l->rail_mv, .temp_c = l->temp_c,
        .faults = l->faults, .state = l->state,
    };
    can_pack_leg_status(&f, l->node, &st);
    l->hal->send(l->hal->ctx, &f);
}

void leg_init(struct leg *l, const struct leg_hal *hal, const struct leg_config *slot0, const struct leg_config *slot1)
{
    memset(l, 0, sizeof(*l));
    l->hal = hal;
    const struct leg_config *stored = config_newest(slot0, slot1);
    if (stored)
        l->cfg = *stored;
    else
        config_defaults(&l->cfg);
    l->node = hal->leg_id(hal->ctx);
    if (l->node < 1 || l->node > 6)
        l->faults |= FAULT_CONFIG;
    if (!config_calibrated(&l->cfg))
        l->faults |= FAULT_UNCALIBRATED;
    enter(l, LEG_OFF);
}

/* ---- wake: start each joint where it is, then prove it follows ---- */

static void wake_tick(struct leg *l)
{
    struct wake *w = &l->wake;
    uint32_t t = now(l);

    if (w->joint < 0) {
        if (t - l->entered < WAKE_BUCK_MS)
            return;
        if (!l->hal->buck_good(l->hal->ctx)) {
            l->faults |= FAULT_BUCK;
            enter(l, LEG_FAULT);
            return;
        }
        w->joint = 0;
        w->phase = 0;
    }

    int j = w->joint;
    const struct joint_cfg *c = &l->cfg.joint[j];
    float meas_us = joint_mv_to_us(c, l->mv[j]);

    switch (w->phase) {
    case 0:
        w->start_us = fminf(meas_us + RESUME_MARGIN_US, 2500.0f);
        pwm(l, j, w->start_us);
        w->t = t;
        w->phase = 1;
        break;
    case 1:
        if (t - w->t < WAKE_SETTLE_MS)
            break;
        /* test move away from whichever edge (angle limit or pulse range) is near */
        w->test_us = w->start_us + TEST_DEG * c->us_per_deg;
        if ((w->test_us > 2500.0f || joint_us_to_deg(c, w->test_us) > c->max_deg ||
             joint_us_to_deg(c, w->test_us) < c->min_deg) &&
            w->start_us - TEST_DEG * c->us_per_deg >= 500.0f)
            w->test_us = w->start_us - TEST_DEG * c->us_per_deg;
        w->before_us = meas_us;
        pwm(l, j, w->test_us);
        w->t = t;
        w->phase = 2;
        break;
    case 2:
        /* judge the move, not the absolute position, so a slightly wrong calibration still wakes */
        if (fabsf((meas_us - w->before_us) - (w->test_us - w->start_us)) < TEST_TOL_DEG * c->us_per_deg) {
            l->target_deg[j] = joint_us_to_deg(c, w->test_us);
            w->phase = 0;
            w->tries = 0;
            if (++w->joint == JOINTS) {
                l->faults &= ~FAULT_WAKE;
                enter(l, LEG_ACTIVE);
            }
        } else if (t - w->t > TEST_TIMEOUT_MS) {
            if (++w->tries >= WAKE_TRIES) {
                l->faults |= FAULT_WAKE;
                enter(l, LEG_FAULT);
                break;
            }
            /* a servo that ignores its first pulse wakes up when sent past where it sits */
            w->start_us = fminf(fmaxf(meas_us, w->start_us) + 60.0f * w->tries, 2500.0f);
            pwm(l, j, w->start_us);
            w->t = t;
            w->phase = 1;
        }
        break;
    }
}

/* ---- calibration: sweep hi -> lo -> hi, fit mV against us ---- */

static bool cal_next_joint(struct leg *l)
{
    struct cal *c = &l->cal;
    while (c->joint < JOINTS && !(c->mask & (1 << c->joint)))
        c->joint++;
    return c->joint < JOINTS;
}

static void cal_start(struct leg *l, uint8_t joint, enum cal_mode mode, uint8_t seq)
{
    if (l->state != LEG_OFF && l->state != LEG_FAULT) {
        reply(l, joint, 0, 0, seq, ST_BUSY);
        return;
    }
    memset(&l->cal, 0, sizeof(l->cal));
    l->cal.mask = joint == 0xff ? 0x7 : (uint8_t)(1 << joint);
    l->cal.mode = mode;
    l->faults &= ~FAULT_CAL;
    cal_next_joint(l);
    reply(l, joint, 0, mode, seq, ST_OK);
    enter(l, LEG_CALIBRATE);
    power(l, true);
}

static void cal_finish_joint(struct leg *l)
{
    struct cal *c = &l->cal;
    struct joint_cfg *jc = &l->cfg.joint[c->joint];
    struct linfit fit;
    double a, b;
    float lo = 1e9f, hi = -1e9f, err = 0;

    linfit_reset(&fit);
    for (int i = 0; i < 2 * CAL_POINTS; i++) {
        linfit_add(&fit, c->x[i], c->y[i]);
        lo = fminf(lo, c->y[i]);
        hi = fmaxf(hi, c->y[i]);
    }
    bool ok = linfit_solve(&fit, &a, &b);
    if (ok)
        for (int i = 0; i < 2 * CAL_POINTS; i++)
            err = fmaxf(err, (float)fabs((double)c->y[i] - (a + b * (double)c->x[i])));

    ok = ok && b > 0.5 && b < 3.0 && err < CAL_MAX_ERR_MV && hi - lo > CAL_MIN_SPAN_MV;
    if (ok) {
        jc->mid_mv = (float)a;
        jc->slope = (float)b;
        jc->fit_err_mv = err;
        jc->calibrated = true;
        reply(l, c->joint, KEY_WIPER_MID, lroundf(jc->mid_mv * 10), 0, ST_CAL_RESULT);
        reply(l, c->joint, KEY_WIPER_SLOPE, lroundf(jc->slope * 1e4f), 0, ST_CAL_RESULT);
        reply(l, c->joint, KEY_FIT_ERR, lroundf(err * 10), 0, ST_CAL_RESULT);
    } else {
        l->faults |= FAULT_CAL;
        reply(l, c->joint, KEY_FIT_ERR, lroundf(err * 10), 0, ST_CAL_FAILED);
    }
    pwm(l, c->joint, 0);
}

static void cal_tick(struct leg *l)
{
    struct cal *c = &l->cal;
    struct joint_cfg *jc = &l->cfg.joint[c->joint];
    uint32_t t = now(l);

    switch (c->phase) {
    case 0:
        if (c->mode == CAL_LIMITS) {
            joint_us_range(jc, &c->lo, &c->hi);
            c->lo -= 5.0f * jc->us_per_deg;
            c->hi += 5.0f * jc->us_per_deg;
        } else {
            c->lo = 500.0f;
            c->hi = 2500.0f;
        }
        c->lo = fmaxf(c->lo, 500.0f);
        c->hi = fminf(c->hi, 2500.0f);
        pwm(l, c->joint, c->hi); /* start above anywhere the servo can sit */
        c->t = t;
        c->point = 0;
        c->phase = 1;
        break;
    case 1:
        if (t - c->t >= CAL_START_MS)
            c->phase = 2;
        break;
    case 2: {
        int k = c->point < CAL_POINTS ? c->point : 2 * CAL_POINTS - 1 - c->point;
        float us = c->hi - (c->hi - c->lo) * k / (CAL_POINTS - 1);
        pwm(l, c->joint, us);
        c->x[c->point] = us - 1500.0f;
        c->t = t;
        c->phase = 3;
        break;
    }
    case 3:
        if (t - c->t < CAL_SETTLE_MS)
            break;
        c->y[c->point] = sample(l, ADC_POT0 + c->joint, 32);
        c->phase = ++c->point < 2 * CAL_POINTS ? 2 : 4;
        break;
    case 4:
        cal_finish_joint(l);
        c->joint++;
        c->phase = 0;
        if (!cal_next_joint(l)) {
            if (config_calibrated(&l->cfg))
                l->faults &= ~FAULT_UNCALIBRATED;
            config_seal(&l->cfg);
            reply(l, 0xff, 0, l->faults & FAULT_CAL ? 0 : 1, 0, ST_CAL_DONE);
            enter(l, LEG_OFF);
        }
        break;
    }
}

/* ---- self-test: supplies and sensors, for a fresh board, servos optional ---- */

static void test_item(struct leg *l, uint8_t item, int32_t v, bool ok)
{
    if (!ok)
        l->test.failed++;
    reply(l, 0xff, item, v, 0, ok ? ST_TEST_PASS : ST_TEST_FAIL);
}

static void test_start(struct leg *l, uint8_t seq)
{
    if (l->state != LEG_OFF && l->state != LEG_FAULT) {
        reply(l, 0xff, 0, 0, seq, ST_BUSY);
        return;
    }
    memset(&l->test, 0, sizeof(l->test));
    all_off(l);
    reply(l, 0xff, 0, 0, seq, ST_OK);
    enter(l, LEG_TEST);
}

static void test_done(struct leg *l)
{
    power(l, false);
    reply(l, 0xff, 0, l->test.failed, 0, ST_TEST_DONE);
    enter(l, LEG_OFF);
}

/* A calibration or self-test cut short still tells the host it ended, as a failure. */
static void abort_running(struct leg *l)
{
    if (l->state == LEG_CALIBRATE) {
        l->faults |= FAULT_CAL;
        reply(l, 0xff, 0, 0, 0, ST_CAL_DONE);
    } else if (l->state == LEG_TEST) {
        l->test.failed++;
        reply(l, 0xff, 0, l->test.failed, 0, ST_TEST_DONE);
    }
}

static void estop_now(struct leg *l)
{
    l->faults |= FAULT_ESTOP;
    abort_running(l);
    enter(l, LEG_OFF);
}

static void test_tick(struct leg *l)
{
    struct test *ts = &l->test;
    uint32_t t = now(l);

    if (ts->phase == 0) {
        if (t - l->entered < TEST_OFF_MS)
            return;
        int32_t i_zero = lroundf(l->i_zero_mv * 1000.0f / (INA_GAIN * SHUNT_MOHM));
        test_item(l, TEST_I_ZERO, i_zero, i_zero < 100);
        test_item(l, TEST_RAIL_OFF, l->rail_mv, l->rail_mv < 500);
        test_item(l, TEST_VBAT, l->vbat_mv, l->vbat_mv > 9000 && l->vbat_mv < 26000);
        test_item(l, TEST_TEMP, l->temp_c, l->temp_c > 0 && l->temp_c < 60);
        power(l, true);
        ts->t = t;
        ts->phase = 1;
    } else if (t - ts->t >= TEST_ON_MS) {
        test_item(l, TEST_POWER_GOOD, l->hal->buck_good(l->hal->ctx), l->hal->buck_good(l->hal->ctx));
        test_item(l, TEST_RAIL_ON, l->rail_mv, l->rail_mv > 5700 && l->rail_mv < 6300);
        test_item(l, TEST_I_ON, l->current_ma, l->current_ma < 1500);
        uint16_t tof = l->hal->tof_mm ? l->hal->tof_mm(l->hal->ctx) : 0;
        test_item(l, TEST_TOF, tof, tof != 0);
        for (int j = 0; j < JOINTS; j++)
            test_item(l, TEST_POT0 + j, lroundf(l->mv[j]), l->mv[j] > 100 && l->mv[j] < 3200);
        test_done(l);
    }
}

/* ---- config over CAN ---- */

static bool leg_key_get(struct leg *l, uint8_t key, int32_t *v)
{
    switch (key) {
    case KEY_VERSION:
    case KEY_RESET_CAUSE:
    case KEY_CAN_ERRORS:
    case KEY_ID_STRAPS:
        *v = l->hal->diag ? (int32_t)l->hal->diag(l->hal->ctx, key) : 0;
        return true;
    case KEY_UPTIME: *v = (int32_t)(now(l) / 1000); return true;
    case KEY_VBAT_GAIN: *v = lroundf(l->cfg.vbat_gain * 1e4f); return true;
    case KEY_RAIL_GAIN: *v = lroundf(l->cfg.rail_gain * 1e4f); return true;
    case KEY_I_ZERO: *v = lroundf(l->i_zero_mv * 1000.0f / (INA_GAIN * SHUNT_MOHM)); return true;
    }
    return false;
}

static uint8_t leg_key_set(struct leg_config *c, uint8_t key, int32_t v)
{
    if (key != KEY_VBAT_GAIN && key != KEY_RAIL_GAIN)
        return ST_BAD_KEY;
    if (v < 8000 || v > 12000) /* +-20 %: more than that is a wrong part, not tolerance */
        return ST_BAD_VALUE;
    if (key == KEY_VBAT_GAIN)
        c->vbat_gain = v / 1e4f;
    else
        c->rail_gain = v / 1e4f;
    return ST_OK;
}

static bool key_get(const struct joint_cfg *c, uint8_t key, int32_t *v)
{
    switch (key) {
    case KEY_WIPER_MID: *v = lroundf(c->mid_mv * 10); return true;
    case KEY_WIPER_SLOPE: *v = lroundf(c->slope * 1e4f); return true;
    case KEY_CENTER_US: *v = lroundf(c->center_us * 10); return true;
    case KEY_DIRECTION: *v = c->dir; return true;
    case KEY_US_PER_DEG: *v = lroundf(c->us_per_deg * 1000); return true;
    case KEY_MIN_DEG: *v = lroundf(c->min_deg * 100); return true;
    case KEY_MAX_DEG: *v = lroundf(c->max_deg * 100); return true;
    case KEY_FIT_ERR: *v = lroundf(c->fit_err_mv * 10); return true;
    case KEY_CALIBRATED: *v = c->calibrated; return true;
    }
    return false;
}

static uint8_t key_set(struct joint_cfg *c, uint8_t key, int32_t v)
{
    switch (key) {
    case KEY_WIPER_MID:
        if (v < 0 || v > 33000) return ST_BAD_VALUE;
        c->mid_mv = v / 10.0f;
        c->calibrated = true;
        return ST_OK;
    case KEY_WIPER_SLOPE:
        if (v < 5000 || v > 30000) return ST_BAD_VALUE;
        c->slope = v / 1e4f;
        return ST_OK;
    case KEY_CENTER_US:
        if (v < 5000 || v > 25000) return ST_BAD_VALUE;
        c->center_us = v / 10.0f;
        return ST_OK;
    case KEY_DIRECTION:
        if (v != 1 && v != -1) return ST_BAD_VALUE;
        c->dir = (int8_t)v;
        return ST_OK;
    case KEY_US_PER_DEG:
        if (v < 5000 || v > 20000) return ST_BAD_VALUE;
        c->us_per_deg = v / 1000.0f;
        return ST_OK;
    case KEY_MIN_DEG:
    case KEY_MAX_DEG: {
        float lo = key == KEY_MIN_DEG ? v / 100.0f : c->min_deg, hi = key == KEY_MAX_DEG ? v / 100.0f : c->max_deg;
        if (v < -18000 || v > 18000 || lo >= hi) return ST_BAD_VALUE;
        c->min_deg = lo;
        c->max_deg = hi;
        return ST_OK;
    }
    }
    return ST_BAD_KEY;
}

static void config_frame(struct leg *l, const struct leg_cfg_msg *m)
{
    /* joint settings, defaults and saving (the flash erase stalls the CPU for ~30 ms) only
       while the servos are off; the leg-wide corrections are harmless any time */
    bool busy = l->state != LEG_OFF && l->state != LEG_FAULT;
    int32_t v = 0;

    switch (m->op) {
    case OP_READ:
        if (m->joint == 0xff ? !leg_key_get(l, m->key, &v)
                             : m->joint >= JOINTS || !key_get(&l->cfg.joint[m->joint], m->key, &v))
            reply(l, m->joint, m->key, 0, m->seq, ST_BAD_KEY);
        else
            reply(l, m->joint, m->key, v, m->seq, ST_OK);
        break;
    case OP_WRITE:
        if (m->joint == 0xff) {
            uint8_t st = leg_key_set(&l->cfg, m->key, m->value);
            config_seal(&l->cfg);
            reply(l, m->joint, m->key, m->value, m->seq, st);
        } else if (busy) {
            reply(l, m->joint, m->key, m->value, m->seq, ST_BUSY);
        } else if (m->joint >= JOINTS) {
            reply(l, m->joint, m->key, m->value, m->seq, ST_BAD_KEY);
        } else {
            uint8_t st = key_set(&l->cfg.joint[m->joint], m->key, m->value);
            config_seal(&l->cfg);
            if (config_calibrated(&l->cfg))
                l->faults &= ~FAULT_UNCALIBRATED;
            reply(l, m->joint, m->key, m->value, m->seq, st);
        }
        break;
    case OP_SAVE: {
        if (busy) {
            reply(l, m->joint, 0, 0, m->seq, ST_BUSY);
            break;
        }
        /* write over the older slot; the other keeps the last good copy if power drops now */
        l->cfg.seq++;
        config_seal(&l->cfg);
        bool ok = l->hal->save(l->hal->ctx, l->cfg.seq & 1, &l->cfg, sizeof(l->cfg));
        if (!ok) {
            l->cfg.seq--; /* retry the same slot, never the good one */
            config_seal(&l->cfg);
        }
        reply(l, m->joint, 0, 0, m->seq, ok ? ST_OK : ST_BAD_VALUE);
        break;
    }
    case OP_CALIBRATE:
        if (m->joint >= JOINTS && m->joint != 0xff)
            reply(l, m->joint, 0, m->value, m->seq, ST_BAD_KEY);
        else
            cal_start(l, m->joint, m->value == CAL_LIMITS ? CAL_LIMITS : CAL_FULL, m->seq);
        break;
    case OP_SELFTEST:
        test_start(l, m->seq);
        break;
    case OP_DEFAULTS: {
        if (busy) {
            reply(l, m->joint, 0, 0, m->seq, ST_BUSY);
            break;
        }
        uint32_t seq = l->cfg.seq; /* or the next save would look older than what's in flash */
        config_defaults(&l->cfg);
        l->cfg.seq = seq;
        config_seal(&l->cfg);
        l->faults |= FAULT_UNCALIBRATED;
        reply(l, m->joint, 0, 0, m->seq, ST_OK);
        break;
    }
    default:
        reply(l, m->joint, m->key, 0, m->seq, ST_BAD_KEY);
    }
}

void leg_frame(struct leg *l, const struct can_frame_t *f)
{
    struct sync_msg s;
    struct leg_cmd_msg c;
    struct leg_cfg_msg m;

    if (can_unpack_sync(f, &s)) {
        l->last_sync = now(l);
        l->synced = true;
        if (s.estop && l->state != LEG_OFF)
            estop_now(l);
        send_state(l, (l->sync_count++ & 3) == (l->node & 3));
        return;
    }
    if (can_node(f->id) != l->node)
        return;
    if (can_unpack_leg_cmd(f, &c)) {
        if (c.enable && !l->enable)
            l->faults &= ~(FAULT_WATCHDOG | FAULT_ESTOP | FAULT_WAKE | FAULT_BUCK | FAULT_OVERLOAD);
        l->enable = c.enable;
        if (l->state == LEG_ACTIVE)
            for (int j = 0; j < JOINTS; j++)
                l->target_deg[j] = joint_clamp_deg(&l->cfg.joint[j], c.deg[j]);
    } else if (can_function(f->id) == CAN_LEG_CONFIG && can_unpack_leg_cfg(f, &m)) {
        config_frame(l, &m);
    }
}

/* Stalled servos, a hot buck or a lost power-good: everything off. The overload fault
   clears on the next enable, but a hot board can't wake until it has cooled down. */
static bool protect(struct leg *l)
{
    if (l->temp_c >= HOT_C)
        l->hot = true;
    else if (l->temp_c < COOL_C)
        l->hot = false;
    if (l->hot)
        l->faults |= FAULT_OVERLOAD;
    if (!l->powered) {
        l->over_ms = l->pg_bad_ms = 0;
        return false;
    }
    if (l->current_ma > OVERLOAD_MA)
        l->over_ms++;
    else if (l->over_ms > 0)
        l->over_ms--; /* leaky, so a short dip doesn't reset a real stall */
    /* the self-test reports power-good itself instead of tripping on it */
    bool pg = l->state == LEG_TEST || now(l) - l->power_t < WAKE_BUCK_MS || l->hal->buck_good(l->hal->ctx);
    l->pg_bad_ms = pg ? 0 : l->pg_bad_ms + 1;

    uint8_t f = (l->hot || l->over_ms > OVERLOAD_MS ? FAULT_OVERLOAD : 0) | (l->pg_bad_ms > PG_LOSS_MS ? FAULT_BUCK : 0);
    if (!f)
        return false;
    l->faults |= f;
    abort_running(l);
    enter(l, LEG_FAULT);
    return true;
}

static void follow(struct leg *l, float speed)
{
    float dt = 0.001f;
    for (int j = 0; j < JOINTS; j++) {
        const struct joint_cfg *c = &l->cfg.joint[j];
        float want = joint_deg_to_us(c, l->target_deg[j]);
        float step = speed * c->us_per_deg * dt;
        float us = l->out_us[j] > 0 ? l->out_us[j] : want;
        us += fmaxf(-step, fminf(step, want - us));
        pwm(l, j, us);
    }
}

void leg_tick(struct leg *l)
{
    uint32_t t = now(l);
    read_inputs(l);

    bool estop = l->hal->estop(l->hal->ctx);
    if (estop)
        l->faults |= FAULT_ESTOP;
    bool sync_lost = !l->synced || t - l->last_sync > SYNC_TIMEOUT_MS;
    bool blocked = estop || (l->faults & (FAULT_CONFIG | FAULT_UNCALIBRATED));
    protect(l);

    switch (l->state) {
    case LEG_OFF:
        if (l->enable && !sync_lost && !blocked &&
            !(l->faults & (FAULT_ESTOP | FAULT_WAKE | FAULT_BUCK | FAULT_OVERLOAD)))
            enter(l, LEG_WAKE);
        break;
    case LEG_WAKE:
        if (!l->enable || estop)
            enter(l, LEG_OFF);
        else
            wake_tick(l);
        break;
    case LEG_ACTIVE:
        if (!l->enable || estop) {
            enter(l, LEG_OFF);
        } else if (sync_lost) {
            l->faults |= FAULT_WATCHDOG;
            for (int j = 0; j < JOINTS; j++)
                l->target_deg[j] = joint_clamp_deg(&l->cfg.joint[j], crouch_deg[j]);
            enter(l, LEG_CROUCH);
        } else {
            follow(l, MAX_SPEED);
        }
        break;
    case LEG_CROUCH:
        if (estop || !l->enable || t - l->entered > CROUCH_MS) {
            enter(l, LEG_OFF);
        } else if (!sync_lost) {
            l->faults &= ~FAULT_WATCHDOG;
            enter(l, LEG_ACTIVE);
        } else {
            follow(l, CROUCH_SPEED);
        }
        break;
    case LEG_CALIBRATE:
        if (estop)
            estop_now(l);
        else
            cal_tick(l);
        break;
    case LEG_FAULT:
        if (!l->enable)
            enter(l, LEG_OFF);
        break;
    case LEG_TEST:
        if (estop)
            estop_now(l);
        else
            test_tick(l);
        break;
    }

    if (sync_lost && t - l->last_idle_tx >= IDLE_TX_MS) {
        l->last_idle_tx = t;
        send_state(l, true);
    }
}
