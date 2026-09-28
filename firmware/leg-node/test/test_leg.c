#include <math.h>
#include <stdio.h>
#include <string.h>

#include "leg_sim.h"
#include "tof_sim.h"
#include "vl53l1x.h"
#include "vector_can.h"

static int failures, checks;

#define CHECK(cond) do { \
    checks++; \
    if (!(cond)) { failures++; printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } \
} while (0)

#define CHECK_NEAR(a, b, tol) do { \
    checks++; \
    double _a = (a), _b = (b); \
    if (fabs(_a - _b) > (tol)) { failures++; printf("  FAIL %s:%d: %s = %g, want %g\n", __FILE__, __LINE__, #a, _a, _b); } \
} while (0)

/* ---- host side of the bus ---- */

#define MAX_RX 4096
static struct can_frame_t rx[MAX_RX];
static int nrx;

static void capture(void *user, const struct can_frame_t *f)
{
    (void)user;
    if (nrx < MAX_RX)
        rx[nrx++] = *f;
}

struct host {
    bool sync, enable;
    float deg[3];
    uint16_t counter;
};

static void run(struct leg_sim *s, struct host *h, int ms)
{
    for (int i = 0; i < ms; i++) {
        if (h && h->sync && s->t_ms % 5 == 0) {
            struct can_frame_t f;
            struct sync_msg sm = {h->counter++, 2, false};
            can_pack_sync(&f, &sm);
            leg_frame(&s->leg, &f);
            struct leg_cmd_msg cm = {{h->deg[0], h->deg[1], h->deg[2]}, h->enable, 0};
            can_pack_leg_cmd(&f, s->id, &cm);
            leg_frame(&s->leg, &f);
        }
        leg_sim_tick(s);
    }
}

static void config(struct leg_sim *s, uint8_t op, uint8_t joint, uint8_t key, int32_t value)
{
    struct can_frame_t f;
    struct leg_cfg_msg m = {joint, key, value, 1, op};
    can_pack_leg_cfg(&f, CAN_LEG_CONFIG, s->id, &m);
    leg_frame(&s->leg, &f);
}

static int replies(uint8_t status, struct leg_cfg_msg *last)
{
    int n = 0;
    struct leg_cfg_msg m;
    for (int i = 0; i < nrx; i++)
        if (can_function(rx[i].id) == CAN_LEG_REPLY && can_unpack_leg_cfg(&rx[i], &m) && m.op == status) {
            n++;
            if (last)
                *last = m;
        }
    return n;
}

static void calibrated_config(struct leg_config *c, const struct servo_sim *servo)
{
    config_defaults(c);
    for (int j = 0; j < JOINTS; j++) {
        c->joint[j].mid_mv = servo[j].mid_mv;
        c->joint[j].slope = servo[j].slope;
        c->joint[j].calibrated = true;
    }
    config_seal(c);
}

static void setup(struct leg_sim *s, const struct leg_config *stored)
{
    nrx = 0;
    leg_sim_init(s, 4, stored);
    s->tx = capture;
}

/* ---- tests ---- */

static void hex(const struct can_frame_t *f, char *out)
{
    for (int i = 0; i < f->len; i++)
        sprintf(out + 2 * i, "%02x", f->data[i]);
}

static void test_protocol(void)
{
    /* golden frames from protocol/vector.dbc via cantools */
    struct can_frame_t f;
    char h[17];

    can_pack_sync(&f, &(struct sync_msg){513, 2, true});
    hex(&f, h);
    CHECK(f.id == 0x000 && !strcmp(h, "01021200"));

    can_pack_leg_cmd(&f, 1, &(struct leg_cmd_msg){{-12.34f, 14.46f, -103.25f}, true, 200});
    hex(&f, h);
    CHECK(f.id == 0x011 && !strcmp(h, "2efba605abd701c8"));

    can_pack_leg_state(&f, 6, &(struct leg_state_msg){{45.0f, -0.01f, -179.99f}, 3210});
    hex(&f, h);
    CHECK(f.id == 0x026 && !strcmp(h, "9411ffffb1b98a0c"));

    can_pack_leg_status(&f, 2, &(struct leg_status_msg){1234, 15870, 6010, -5, 0x81, 2});
    hex(&f, h);
    CHECK(f.id == 0x032 && !strcmp(h, "d204339625fb8102"));
    struct leg_status_msg st;
    CHECK(can_unpack_leg_status(&f, &st) && st.vbat_mv == 15870 && st.rail_mv == 6010 && st.state == 2);

    can_pack_leg_cfg(&f, CAN_LEG_CONFIG, 4, &(struct leg_cfg_msg){1, 2, -14330, 7, 1});
    hex(&f, h);
    CHECK(f.id == 0x054 && !strcmp(h, "010206c8ffff0701"));

    can_pack_leg_cfg(&f, CAN_LEG_REPLY, 6, &(struct leg_cfg_msg){2, 1, 15930, 200, 4});
    hex(&f, h);
    CHECK(f.id == 0x066 && !strcmp(h, "02013a3e0000c804"));
}

static void test_joint(void)
{
    struct joint_cfg j = {.mid_mv = 1524, .slope = 1.476f, .center_us = 1500, .dir = -1,
                          .us_per_deg = 11.11f, .min_deg = -30, .max_deg = 60};
    CHECK_NEAR(joint_deg_to_us(&j, 10), 1388.9, 0.1);
    CHECK_NEAR(joint_us_to_deg(&j, joint_deg_to_us(&j, 37.5f)), 37.5, 1e-3);
    CHECK_NEAR(joint_mv_to_us(&j, 1524 + 1.476f * 300), 1800, 1e-2);
    float lo, hi;
    joint_us_range(&j, &lo, &hi);
    CHECK_NEAR(lo, 1500 - 60 * 11.11, 0.1);
    CHECK_NEAR(hi, 1500 + 30 * 11.11, 0.1);
    CHECK_NEAR(joint_clamp_deg(&j, 90), 60, 0);

    uint16_t v[9] = {1500, 1502, 1498, 1900, 1501, 1100, 1499, 1503, 1500};
    CHECK(median_u16(v, 9) == 1500);
}

static void test_refuses_uncalibrated(void)
{
    struct leg_sim s;
    struct host h = {.sync = true, .enable = true};
    setup(&s, NULL);
    run(&s, &h, 500);
    CHECK(s.leg.state == LEG_OFF);
    CHECK(s.leg.faults & FAULT_UNCALIBRATED);
    CHECK(!s.buck);
}

static void test_calibration(enum cal_mode mode)
{
    struct leg_sim s;
    setup(&s, NULL);
    /* three servos from docs/servos.md: R1_coxa, R2_femur, R3_tibia */
    const float mid[3] = {1646, 1522, 1524}, slope[3] = {1.366f, 1.465f, 1.476f};
    for (int j = 0; j < JOINTS; j++) {
        s.servo[j].mid_mv = mid[j];
        s.servo[j].slope = slope[j];
        s.servo[j].pos_us = 1100 + 300 * j;
        s.leg.cfg.joint[j].min_deg = -30;
        s.leg.cfg.joint[j].max_deg = 45;
    }
    config(&s, OP_CALIBRATE, 0xff, 0, mode);
    CHECK(s.leg.state == LEG_CALIBRATE);

    int lo = 3000, hi = 0;
    for (int i = 0; i < 40000 && s.leg.state == LEG_CALIBRATE; i++) {
        run(&s, NULL, 1);
        for (int j = 0; j < JOINTS; j++)
            if (s.servo[j].pulses) {
                lo = s.servo[j].last_us < lo ? s.servo[j].last_us : lo;
                hi = s.servo[j].last_us > hi ? s.servo[j].last_us : hi;
            }
    }
    CHECK(s.leg.state == LEG_OFF);
    CHECK(!(s.leg.faults & (FAULT_CAL | FAULT_UNCALIBRATED)));
    for (int j = 0; j < JOINTS; j++) {
        CHECK_NEAR(s.leg.cfg.joint[j].mid_mv, mid[j], 6);
        CHECK_NEAR(s.leg.cfg.joint[j].slope, slope[j], 0.01);
        CHECK(s.leg.cfg.joint[j].calibrated);
    }
    CHECK(replies(ST_CAL_RESULT, NULL) == 9);
    CHECK(replies(ST_CAL_DONE, NULL) == 1);
    if (mode == CAL_LIMITS) {
        /* -30..45 deg plus 5 deg margin */
        CHECK(lo >= 1500 - 35 * 11.11 - 1 && hi <= 1500 + 50 * 11.11 + 1);
    } else {
        CHECK(lo == 500 && hi == 2500);
    }
}

static void test_wake(const float *park, bool wrong_calibration)
{
    struct leg_sim s;
    struct leg_config c;
    struct servo_sim ref[JOINTS];
    for (int j = 0; j < JOINTS; j++)
        servo_sim_init(&ref[j], 1646, 1.366f, park[j]);
    calibrated_config(&c, ref);
    if (wrong_calibration)
        for (int j = 0; j < JOINTS; j++) {
            /* the leg thinks the servo is a different one: its estimate lands below */
            c.joint[j].mid_mv = 1700;
            c.joint[j].slope = 1.366f;
        }
    config_seal(&c);
    setup(&s, &c);
    for (int j = 0; j < JOINTS; j++)
        s.servo[j] = ref[j];

    struct host h = {.sync = true, .enable = true, .deg = {10, -20, 30}};
    run(&s, &h, 3000);
    CHECK(s.leg.state == LEG_ACTIVE);
    CHECK(!(s.leg.faults & FAULT_WAKE));
    if (!wrong_calibration)
        for (int j = 0; j < JOINTS; j++)
            CHECK_NEAR(s.leg.pos_deg[j], h.deg[j], 1.5);
    for (int j = 0; j < JOINTS; j++)
        CHECK(!s.servo[j].stuck);
}

static void test_dead_servo(void)
{
    struct leg_sim s;
    struct leg_config c;
    struct servo_sim ref[JOINTS];
    for (int j = 0; j < JOINTS; j++)
        servo_sim_init(&ref[j], 1560, 1.433f, 1500);
    calibrated_config(&c, ref);
    setup(&s, &c);
    s.servo[1].dead = true;
    struct host h = {.sync = true, .enable = true};
    run(&s, &h, 5000);
    CHECK(s.leg.state == LEG_FAULT);
    CHECK(s.leg.faults & FAULT_WAKE);
    CHECK(!s.buck);
}

static void test_watchdog(void)
{
    struct leg_sim s;
    struct leg_config c;
    struct servo_sim ref[JOINTS];
    for (int j = 0; j < JOINTS; j++)
        servo_sim_init(&ref[j], 1560, 1.433f, 1500);
    calibrated_config(&c, ref);
    setup(&s, &c);
    struct host h = {.sync = true, .enable = true};
    run(&s, &h, 2000);
    CHECK(s.leg.state == LEG_ACTIVE);

    h.sync = false;
    run(&s, &h, 200);
    CHECK(s.leg.state == LEG_CROUCH);
    CHECK(s.leg.faults & FAULT_WATCHDOG);

    h.sync = true;
    run(&s, &h, 50);
    CHECK(s.leg.state == LEG_ACTIVE);

    h.sync = false;
    run(&s, &h, 2500);
    CHECK(s.leg.state == LEG_OFF);
    CHECK(!s.buck);
}

static void test_estop(void)
{
    struct leg_sim s;
    struct leg_config c;
    struct servo_sim ref[JOINTS];
    for (int j = 0; j < JOINTS; j++)
        servo_sim_init(&ref[j], 1560, 1.433f, 1500);
    calibrated_config(&c, ref);
    setup(&s, &c);
    struct host h = {.sync = true, .enable = true};
    run(&s, &h, 2000);
    CHECK(s.leg.state == LEG_ACTIVE);
    s.estop = true;
    run(&s, &h, 2);
    CHECK(s.leg.state == LEG_OFF && !s.buck);
    float held = s.leg.pos_deg[1];
    run(&s, &h, 100);
    CHECK_NEAR(s.leg.pos_deg[1], held, 0.5); /* not the dead pot's 0 V */
    run(&s, &h, 500);
    CHECK(s.leg.state == LEG_OFF);
}

static void test_config(void)
{
    struct leg_sim s;
    setup(&s, NULL);
    for (int j = 0; j < JOINTS; j++) {
        config(&s, OP_WRITE, j, KEY_WIPER_MID, 15930);
        config(&s, OP_WRITE, j, KEY_WIPER_SLOPE, 13740);
    }
    config(&s, OP_WRITE, 1, KEY_MIN_DEG, -8000);
    config(&s, OP_WRITE, 1, KEY_DIRECTION, 2);
    config(&s, OP_WRITE, 1, 99, 0);
    CHECK(!(s.leg.faults & FAULT_UNCALIBRATED));
    CHECK_NEAR(s.leg.cfg.joint[2].slope, 1.374, 1e-4);
    CHECK_NEAR(s.leg.cfg.joint[1].min_deg, -80, 1e-4);
    CHECK(replies(ST_BAD_VALUE, NULL) == 1);
    CHECK(replies(ST_BAD_KEY, NULL) == 1);

    struct leg_cfg_msg m;
    nrx = 0;
    config(&s, OP_READ, 0, KEY_WIPER_MID, 0);
    CHECK(replies(ST_OK, &m) == 1 && m.value == 15930);

    config(&s, OP_SAVE, 0, 0, 0);
    CHECK(s.flash_written && config_valid(&s.flash[1]) && s.flash[1].seq == 1);
    struct leg_sim again;
    leg_sim_init(&again, 4, &s.flash[1]);
    CHECK_NEAR(again.leg.cfg.joint[1].mid_mv, 1593, 1e-3);
    CHECK(!(again.leg.faults & FAULT_UNCALIBRATED));

    /* saves alternate between the slots; a save that dies half way keeps the older copy */
    config(&s, OP_WRITE, 0, KEY_WIPER_MID, 16000);
    config(&s, OP_SAVE, 0, 0, 0);
    CHECK(config_valid(&s.flash[0]) && s.flash[0].seq == 2 && s.flash[1].seq == 1);
    CHECK(config_newest(&s.flash[0], &s.flash[1]) == &s.flash[0]);
    config(&s, OP_WRITE, 0, KEY_WIPER_MID, 16100);
    s.flash_fail = true;
    nrx = 0;
    config(&s, OP_SAVE, 0, 0, 0);
    CHECK(replies(ST_BAD_VALUE, NULL) == 1);
    CHECK(!config_valid(&s.flash[1]) && config_newest(&s.flash[0], &s.flash[1]) == &s.flash[0]);
    config(&s, OP_SAVE, 0, 0, 0); /* the retry goes to the same, broken slot */
    CHECK(config_valid(&s.flash[0]) && s.flash[0].seq == 2);
    CHECK(config_valid(&s.flash[1]) && s.flash[1].seq == 3);
    leg_init(&again.leg, &again.hal, &s.flash[0], &s.flash[1]);
    CHECK_NEAR(again.leg.cfg.joint[0].mid_mv, 1610, 1e-3);
    struct leg_config wrap_a = s.flash[0], wrap_b = s.flash[0];
    wrap_a.seq = 0xffffffffu;
    wrap_b.seq = 0;
    config_seal(&wrap_a);
    config_seal(&wrap_b);
    CHECK(config_newest(&wrap_a, &wrap_b) == &wrap_b);
    CHECK(config_newest(NULL, NULL) == NULL);
}

static void active_leg(struct leg_sim *s, struct host *h)
{
    struct leg_config c;
    struct servo_sim ref[JOINTS];
    for (int j = 0; j < JOINTS; j++)
        servo_sim_init(&ref[j], 1560, 1.433f, 1500);
    calibrated_config(&c, ref);
    setup(s, &c);
    *h = (struct host){.sync = true, .enable = true};
    run(s, h, 2000);
}

static void test_overcurrent(void)
{
    struct leg_sim s;
    struct host h;
    active_leg(&s, &h);
    CHECK(s.leg.state == LEG_ACTIVE);

    s.load_ma = 6500; /* a hard step, not a stall */
    run(&s, &h, 300);
    s.load_ma = 1500;
    run(&s, &h, 1000);
    CHECK(s.leg.state == LEG_ACTIVE && !(s.leg.faults & FAULT_OVERLOAD));

    s.load_ma = 7500; /* stalled */
    run(&s, &h, 450);
    CHECK(s.leg.state == LEG_ACTIVE);
    run(&s, &h, 100);
    CHECK(s.leg.state == LEG_FAULT && (s.leg.faults & FAULT_OVERLOAD) && !s.buck);

    s.load_ma = 500;
    h.enable = false;
    run(&s, &h, 50);
    h.enable = true;
    run(&s, &h, 2000);
    CHECK(s.leg.state == LEG_ACTIVE && !(s.leg.faults & FAULT_OVERLOAD));
}

static void test_overtemp(void)
{
    struct leg_sim s;
    struct host h;
    active_leg(&s, &h);
    s.ntc_mv = 383; /* 90 C */
    run(&s, &h, 5);
    CHECK(s.leg.state == LEG_FAULT && (s.leg.faults & FAULT_OVERLOAD) && !s.buck);
    CHECK(s.leg.temp_c >= 89 && s.leg.temp_c <= 91);

    s.ntc_mv = 600; /* ~70-75 C: cooler, but not cool enough */
    h.enable = false;
    run(&s, &h, 50);
    h.enable = true;
    run(&s, &h, 500);
    CHECK(s.leg.state == LEG_OFF && (s.leg.faults & FAULT_OVERLOAD));

    s.ntc_mv = 769; /* 60 C */
    h.enable = false;
    run(&s, &h, 50);
    h.enable = true;
    run(&s, &h, 2000);
    CHECK(s.leg.state == LEG_ACTIVE && !(s.leg.faults & FAULT_OVERLOAD));
}

static void test_power_good_lost(void)
{
    struct leg_sim s;
    struct host h;
    active_leg(&s, &h);
    s.pg_fail = true;
    run(&s, &h, 15);
    CHECK(s.leg.state == LEG_ACTIVE);
    run(&s, &h, 10);
    CHECK(s.leg.state == LEG_FAULT && (s.leg.faults & FAULT_BUCK) && !s.buck);
}

static int test_replies(uint8_t status, uint8_t item, int32_t *value)
{
    int n = 0;
    struct leg_cfg_msg m;
    for (int i = 0; i < nrx; i++)
        if (can_function(rx[i].id) == CAN_LEG_REPLY && can_unpack_leg_cfg(&rx[i], &m) && m.op == status &&
            (!item || m.key == item)) {
            n++;
            if (value)
                *value = m.value;
        }
    return n;
}

static void test_selftest(void)
{
    struct leg_sim s;
    struct leg_config c;
    struct servo_sim ref[JOINTS];
    for (int j = 0; j < JOINTS; j++)
        servo_sim_init(&ref[j], 1560, 1.433f, 1500);
    calibrated_config(&c, ref);
    setup(&s, &c);
    int32_t v = 0;

    run(&s, NULL, 300);
    nrx = 0;
    config(&s, OP_SELFTEST, 0xff, 0, 0);
    CHECK(s.leg.state == LEG_TEST && replies(ST_OK, NULL) == 1);
    run(&s, NULL, 100);
    CHECK(!s.buck);
    run(&s, NULL, 400);
    CHECK(s.leg.state == LEG_OFF && !s.buck);
    CHECK(test_replies(ST_TEST_PASS, 0, NULL) == 11 && test_replies(ST_TEST_FAIL, 0, NULL) == 0);
    CHECK(test_replies(ST_TEST_DONE, 0, &v) == 1 && v == 0);
    CHECK(test_replies(ST_TEST_PASS, TEST_RAIL_ON, &v) == 1 && v > 5900 && v < 6100);
    CHECK(test_replies(ST_TEST_PASS, TEST_VBAT, &v) == 1 && v > 15200 && v < 15400);

    /* no power-good and no ToF: two failures, and power-good doesn't trip the leg */
    s.pg_fail = true;
    s.tof = 0;
    nrx = 0;
    config(&s, OP_SELFTEST, 0xff, 0, 0);
    run(&s, NULL, 500);
    CHECK(test_replies(ST_TEST_FAIL, TEST_POWER_GOOD, NULL) == 1 && test_replies(ST_TEST_FAIL, TEST_TOF, NULL) == 1);
    CHECK(test_replies(ST_TEST_DONE, 0, &v) == 1 && v == 2);
    CHECK(s.leg.state == LEG_OFF);

    /* busy while walking */
    struct host h;
    active_leg(&s, &h);
    nrx = 0;
    config(&s, OP_SELFTEST, 0xff, 0, 0);
    CHECK(replies(ST_BUSY, NULL) == 1 && s.leg.state == LEG_ACTIVE);

    /* e-stop ends it early */
    setup(&s, &c);
    config(&s, OP_SELFTEST, 0xff, 0, 0);
    run(&s, NULL, 200);
    s.estop = true;
    run(&s, NULL, 2);
    CHECK(s.leg.state == LEG_OFF && !s.buck && test_replies(ST_TEST_DONE, 0, NULL) == 1);
}

static void test_leg_keys(void)
{
    struct leg_sim s;
    struct host h;
    struct leg_cfg_msg m;
    active_leg(&s, &h);

    nrx = 0;
    config(&s, OP_READ, 0xff, KEY_VERSION, 0);
    CHECK(replies(ST_OK, &m) == 1 && (uint32_t)m.value == 0x0abcdefu);
    config(&s, OP_READ, 0xff, KEY_UPTIME, 0);
    CHECK(replies(ST_OK, &m) == 2 && m.value == 2);
    config(&s, OP_READ, 0xff, KEY_WIPER_MID, 0);
    CHECK(replies(ST_BAD_KEY, NULL) == 1);

    /* current amp offset: learned while off, subtracted while on */
    setup(&s, NULL);
    active_leg(&s, &h);
    s.i_offset_mv = 15;
    h.enable = false;
    run(&s, &h, 400);
    nrx = 0;
    config(&s, OP_READ, 0xff, KEY_I_ZERO, 0);
    CHECK(replies(ST_OK, &m) == 1 && m.value == 75);
    h.enable = true;
    run(&s, &h, 2000);
    CHECK(s.leg.state == LEG_ACTIVE);
    CHECK_NEAR(s.leg.current_ma, 500, 5);

    /* divider correction */
    uint16_t before = s.leg.vbat_mv;
    config(&s, OP_WRITE, 0xff, KEY_VBAT_GAIN, 10100);
    run(&s, &h, 2);
    CHECK_NEAR(s.leg.vbat_mv, before * 1.01, 2);
    nrx = 0;
    config(&s, OP_WRITE, 0xff, KEY_RAIL_GAIN, 13000);
    config(&s, OP_WRITE, 0xff, KEY_UPTIME, 5);
    CHECK(replies(ST_BAD_VALUE, NULL) == 1 && replies(ST_BAD_KEY, NULL) == 1);
    config(&s, OP_SAVE, 0, 0, 0);
    struct leg_sim again;
    leg_sim_init(&again, 4, config_newest(&s.flash[0], &s.flash[1]));
    CHECK_NEAR(again.leg.cfg.vbat_gain, 1.01, 1e-4);
}

/* ---- VL53L1X, against the registers the SparkFun/ST driver writes ---- */

static uint16_t be16(const struct tof_sim *t, uint16_t reg) { return (uint16_t)(t->reg[reg] << 8 | t->reg[reg + 1]); }

static void tof_setup(struct tof_sim *t, struct vl53l1x *d, enum vl53l1x_mode mode, uint16_t budget, uint16_t period)
{
    tof_sim_init(t);
    *d = (struct vl53l1x){.ctx = t, .write = tof_sim_write, .read = tof_sim_read};
    vl53l1x_begin(d, mode, budget, period);
}

/* the leg firmware's loop: the sensor model runs every ms, the driver every 5 ms */
static int tof_run(struct tof_sim *t, struct vl53l1x *d, int ms, uint16_t *mm, uint8_t *st)
{
    int ranges = 0;
    for (int i = 0; i < ms; i++) {
        tof_sim_step(t, 1);
        if (t->t_ms % 5 == 0 && vl53l1x_run(d, t->t_ms, mm, st) == 1)
            ranges++;
    }
    return ranges;
}

static void test_tof(void)
{
    struct tof_sim t;
    struct vl53l1x d;
    uint16_t mm = 0;
    uint8_t st = 0;

    tof_setup(&t, &d, VL53L1X_SHORT, 20, 25);
    tof_run(&t, &d, 150, &mm, &st);
    CHECK(vl53l1x_ranging(&d) && d.starts == 1);
    CHECK(t.max_write <= 23); /* the config goes over in pieces */
    CHECK(t.reg[0x4B] == 0x14 && t.reg[0x60] == 0x07 && t.reg[0x63] == 0x05 && t.reg[0x69] == 0x38);
    CHECK(be16(&t, 0x78) == 0x0705 && be16(&t, 0x7A) == 0x0606);
    CHECK(be16(&t, 0x5E) == 0x0051 && be16(&t, 0x61) == 0x006E);
    CHECK(t.reg[0x08] == 0x09 && t.reg[0x0B] == 0x00 && t.reg[0x2E] == 0x01);
    CHECK((uint32_t)(be16(&t, 0x6C) << 16 | be16(&t, 0x6E)) == (uint32_t)(0x01A8 * 25 * 1.075f));

    t.distance_mm = 1234;
    int n = tof_run(&t, &d, 1000, &mm, &st);
    CHECK(n >= 38 && n <= 41 && mm == 1234 && st == 0); /* one every 25 ms */
    t.raw_status = 4; /* signal fail: nothing in range */
    CHECK(tof_run(&t, &d, 30, &mm, &st) >= 1 && mm == VL53L1X_NO_TARGET && st == 2);
    t.raw_status = 7;
    CHECK(tof_run(&t, &d, 30, &mm, &st) >= 1 && mm == VL53L1X_NO_TARGET && st == 7);
    t.raw_status = 9;

    /* unplugged, plugged back in (a fresh sensor) */
    t.present = false;
    tof_run(&t, &d, 20, &mm, &st);
    CHECK(!vl53l1x_ranging(&d));
    CHECK(tof_run(&t, &d, 2000, &mm, &st) == 0);
    uint32_t now = t.t_ms;
    tof_sim_init(&t);
    t.t_ms = now;
    tof_run(&t, &d, 700, &mm, &st);
    CHECK(vl53l1x_ranging(&d) && d.starts == 2);
    CHECK(tof_run(&t, &d, 100, &mm, &st) >= 3 && mm == 400);

    /* reset behind our back (a brown-out): answers, but never has a range again */
    now = t.t_ms;
    tof_sim_init(&t);
    t.t_ms = now;
    tof_run(&t, &d, 1800, &mm, &st);
    CHECK(vl53l1x_ranging(&d) && d.starts == 3);

    /* switching mode keeps the budget, as SetDistanceMode does */
    CHECK(vl53l1x_set_budget(&d, 33) && vl53l1x_set_mode(&d, VL53L1X_LONG));
    CHECK(t.reg[0x4B] == 0x0A && be16(&t, 0x78) == 0x0F0D && be16(&t, 0x5E) == 0x0060 && be16(&t, 0x61) == 0x006E);
    CHECK(!vl53l1x_set_budget(&d, 15)); /* short mode only */

    tof_setup(&t, &d, VL53L1X_LONG, 50, 50);
    tof_run(&t, &d, 200, &mm, &st);
    CHECK(vl53l1x_ranging(&d) && t.reg[0x4B] == 0x0A && be16(&t, 0x5E) == 0x00AD && be16(&t, 0x61) == 0x00C6);

    /* no sensor at all: tried every 500 ms, nothing else */
    tof_setup(&t, &d, VL53L1X_SHORT, 20, 25);
    t.present = false;
    CHECK(tof_run(&t, &d, 2000, &mm, &st) == 0 && !vl53l1x_ranging(&d));
    CHECK(t.failed_reads >= 4 && t.failed_reads <= 5);
    t.present = true;
    tof_run(&t, &d, 700, &mm, &st);
    CHECK(vl53l1x_ranging(&d));

    tof_setup(&t, &d, VL53L1X_SHORT, 20, 25);
    t.reg[0x0110] = 0xCD; /* some other ST part answering at 0x29 */
    tof_run(&t, &d, 2000, &mm, &st);
    CHECK(!vl53l1x_ranging(&d));

    tof_setup(&t, &d, VL53L1X_SHORT, 20, 25);
    t.reg[0x00E5] = 0; /* never finishes booting */
    tof_run(&t, &d, 2000, &mm, &st);
    CHECK(!vl53l1x_ranging(&d));
}

int main(void)
{
    const float park_high[3] = {2000, 2400, 1900}, park_mid[3] = {1500, 1200, 800};
    struct { const char *name; void (*fn)(void); } tests[] = {
        {"protocol", test_protocol},
        {"joint", test_joint},
        {"refuses_uncalibrated", test_refuses_uncalibrated},
        {"dead_servo", test_dead_servo},
        {"watchdog", test_watchdog},
        {"estop", test_estop},
        {"config", test_config},
        {"overcurrent", test_overcurrent},
        {"overtemp", test_overtemp},
        {"power_good_lost", test_power_good_lost},
        {"selftest", test_selftest},
        {"leg_keys", test_leg_keys},
        {"tof", test_tof},
    };
    for (unsigned i = 0; i < sizeof(tests) / sizeof(tests[0]); i++) {
        printf("%s\n", tests[i].name);
        tests[i].fn();
    }
    printf("calibration_full\n");
    test_calibration(CAL_FULL);
    printf("calibration_limits\n");
    test_calibration(CAL_LIMITS);
    printf("wake_parked_high\n");
    test_wake(park_high, false);
    printf("wake_parked_mid\n");
    test_wake(park_mid, false);
    printf("wake_wrong_calibration\n");
    test_wake(park_high, true);
    printf("wake_at_top_of_range\n");
    const float park_edge[3] = {2490, 2495, 510};
    test_wake(park_edge, false);

    printf("%d checks, %d failed\n", checks, failures);
    return failures != 0;
}
