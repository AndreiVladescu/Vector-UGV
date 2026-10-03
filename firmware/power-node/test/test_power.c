#include <math.h>
#include <stdio.h>
#include <string.h>

#include "power_sim.h"
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

#define MAX_RX 8192
static struct can_frame_t rx[MAX_RX];
static int nrx;

static void capture(void *user, const struct can_frame_t *f)
{
    (void)user;
    if (nrx < MAX_RX)
        rx[nrx++] = *f;
}

static int count(uint32_t id)
{
    int n = 0;
    for (int i = 0; i < nrx; i++)
        n += rx[i].id == id;
    return n;
}

/* an empty frame when there is none, which every unpack refuses */
static const struct can_frame_t *last(uint32_t id)
{
    static const struct can_frame_t none;
    for (int i = nrx - 1; i >= 0; i--)
        if (rx[i].id == id)
            return &rx[i];
    return &none;
}

static struct power_sim sim;

static void start(bool crc, bool button, bool charger)
{
    power_sim_init(&sim, crc);
    sim.tx = capture;
    nrx = 0;
    sim.button = button;
    sim.chg.vbus = charger;
    sim.chg.vbus_mv = charger ? 20000 : 0;
    power_sim_boot(&sim);
    power_sim_run(&sim, 50);
}

static void config(uint8_t op, uint8_t key, int32_t value, uint8_t seq)
{
    struct leg_cfg_msg m = {.joint = 0xff, .key = key, .value = value, .seq = seq, .op = op};
    struct can_frame_t f;
    can_pack_leg_cfg(&f, CAN_LEG_CONFIG, POWER_NODE, &m);
    power_frame(&sim.p, &f);
}

static bool reply(uint8_t seq, uint8_t status, int32_t *value)
{
    for (int i = nrx - 1; i >= 0; i--) {
        struct leg_cfg_msg m;
        if (rx[i].id == (CAN_LEG_REPLY | POWER_NODE) && can_unpack_leg_cfg(&rx[i], &m) && m.seq == seq) {
            if (value)
                *value = m.value;
            return m.op == status;
        }
    }
    return false;
}

static void sync(bool estop)
{
    struct sync_msg m = {.counter = 1, .estop = estop};
    struct can_frame_t f;
    can_pack_sync(&f, &m);
    power_frame(&sim.p, &f);
}

static void press(uint32_t ms)
{
    sim.button = true;
    power_sim_run(&sim, ms);
    sim.button = false;
    power_sim_run(&sim, 20);
}

static void test_button_wake(bool crc)
{
    printf("button wake%s\n", crc ? " (CRC)" : "");
    start(crc, true, false);
    sim.button = false;
    CHECK(sim.p.state == POWER_ON);
    CHECK(sim.bms.cfgupdates == 1);
    CHECK(sim.bms.rejected_writes == 0);
    CHECK(bq76942_sim_mem16(&sim.bms, 0x9304) == 0x0207);
    CHECK(sim.bms.mem[0x9308 - BQS_MEM_BASE] == 0x3D);
    CHECK(sim.bms.mem[0x92FE - BQS_MEM_BASE] == 0x00); /* TS2 left for the wake button */
    CHECK(sim.bms.fet_en);
    CHECK(bq76942_sim_fets(&sim.bms) == (BQ_FET_CHG | BQ_FET_DSG));
    CHECK(!sim.outs[OUT_CM5_OFF]);
    CHECK(!sim.outs[OUT_SHUTDOWN_REQ] && !sim.outs[OUT_ESTOP]);
    CHECK(sim.bms.mem[0x92FD - BQS_MEM_BASE] == 0x00); /* no cell thermistor by default */
    CHECK(sim.bms.mem[0x9303 - BQS_MEM_BASE] == 0x09); /* the die temperature stands in */
    CHECK(bq25798_sim_reg16(&sim.chg, CHG_VREG) == 1660);
    CHECK(bq25798_sim_reg16(&sim.chg, CHG_ICHG) == 300);

    nrx = 0;
    power_sim_run(&sim, 1000);
    CHECK(count(CAN_POWER_STATE | POWER_NODE) == 10);
    CHECK(count(CAN_POWER_CELLS | POWER_NODE) == 1);
    CHECK(count(CAN_POWER_DETAIL | POWER_NODE) == 1);
    struct power_state_msg ps;
    CHECK(can_unpack_power_state(last(CAN_POWER_STATE | POWER_NODE), &ps));
    CHECK(ps.pack_mv == 15200); /* mV now, not the default centivolts */
    CHECK(ps.state == POWER_ON);
    CHECK(ps.faults == PWR_FAULT_CONFIG); /* fresh flash */
    CHECK_NEAR(ps.soc_half / 2.0, 58, 1);
    struct power_cells_msg pc;
    CHECK(can_unpack_power_cells(last(CAN_POWER_CELLS | POWER_NODE), &pc));
    CHECK(pc.cell_mv[0] == 3800 && pc.cell_mv[3] == 3800);
    struct power_detail_msg pd;
    CHECK(can_unpack_power_detail(last(CAN_POWER_DETAIL | POWER_NODE), &pd));
    CHECK(pd.fets == (BQ_FET_CHG | BQ_FET_DSG) && !pd.usb);
    CHECK(sim.bms.crc_errors == 0);

    /* holding the button that woke it doesn't shut it straight down */
    start(crc, true, false);
    power_sim_run(&sim, 3000);
    CHECK(sim.p.state == POWER_ON);
}

static void test_mcu_reset_keeps_power(void)
{
    printf("MCU reset while on\n");
    start(false, true, false);
    sim.button = false;
    power_sim_run(&sim, 500);
    int cfgupdates = sim.bms.cfgupdates;
    power_sim_boot(&sim);
    for (int i = 0; i < 500; i++) {
        power_sim_run(&sim, 1);
        if (!(bq76942_sim_fets(&sim.bms) & BQ_FET_DSG))
            break;
    }
    CHECK(sim.bms.cfgupdates == cfgupdates); /* configured already: no CONFIG_UPDATE, no FET glitch */
    CHECK(bq76942_sim_fets(&sim.bms) & BQ_FET_DSG);
    CHECK(sim.p.state == POWER_ON);
}

static void test_crc_errors(void)
{
    printf("BMS CRC errors\n");
    power_sim_init(&sim, true);
    sim.tx = capture;
    sim.button = true;
    sim.bms.corrupt_next = 3;
    power_sim_boot(&sim);
    power_sim_run(&sim, 1000);
    CHECK(sim.p.bms.errors >= 3);
    CHECK(sim.p.state == POWER_ON);

    /* the BMS only takes frames with a good CRC */
    uint8_t bad[4] = {BQ_SUBCMD, 0x95, 0x00, 0x00};
    CHECK(!bq76942_sim_write(&sim.bms, bad, 4));
    CHECK(bq76942_sim_fets(&sim.bms) & BQ_FET_DSG);
}

static void test_charger_wake(void)
{
    printf("charger wake\n");
    start(false, false, true);
    CHECK(sim.p.state == POWER_CHARGE);
    CHECK(sim.outs[OUT_CM5_OFF]);
    CHECK(bq76942_sim_fets(&sim.bms) == (BQ_FET_CHG | BQ_FET_DSG));
    power_sim_run(&sim, 1100);
    CHECK(sim.p.flags & PWR_CHARGER);
    CHECK(sim.p.flags & PWR_CHARGING);
    struct power_detail_msg pd;
    CHECK(can_unpack_power_detail(last(CAN_POWER_DETAIL | POWER_NODE), &pd));
    CHECK(pd.usb && pd.input_mv == 20000 && pd.chg_stat == CHG_FAST);

    /* charging the pack moves the gauge */
    float soc = sim.p.gauge.soc;
    sim.bms.current_ma = 3000;
    power_sim_run(&sim, 60000);
    CHECK_NEAR(sim.p.gauge.soc - soc, 3000.0 / 60 * 100 / 8000, 0.05);

    /* done: 100 % */
    sim.chg.done = true;
    power_sim_run(&sim, 1100);
    CHECK_NEAR(sim.p.gauge.soc, 100, 0.01);
    CHECK(!(sim.p.flags & PWR_CHARGING));

    /* a press while charging turns the robot on */
    press(100);
    CHECK(sim.p.state == POWER_ON);
    CHECK(!sim.outs[OUT_CM5_OFF]);

    /* long press with the charger in: back to charging, not off */
    press(2100);
    CHECK(sim.p.state == POWER_HALTING);
    sim.halted = true;
    power_sim_run(&sim, 1100);
    CHECK(sim.p.state == POWER_CHARGE);
    sim.halted = false;

    /* unplugged: off after 5 s, SoC saved, BMS shut down */
    sim.chg.vbus = false;
    sim.bms.current_ma = 0;
    int saves = sim.saves;
    power_sim_run(&sim, 7000);
    CHECK(sim.bms.shutdown);
    CHECK(!sim.bms.awake);
    CHECK(sim.saves == saves + 1);
    const struct power_config *c = config_newest(&sim.flash[0], &sim.flash[1]);
    CHECK(c && fabsf(c->soc - 100) < 0.01f);
}

static void test_spurious_wake(void)
{
    printf("spurious wake\n");
    start(false, false, false);
    power_sim_run(&sim, 100);
    CHECK(sim.bms.shutdown && !sim.bms.awake);
    CHECK(bq76942_sim_fets(&sim.bms) == 0);
}

static void test_long_press_shutdown(void)
{
    printf("long press, CM5 halts\n");
    start(false, true, false);
    sim.button = false;
    power_sim_run(&sim, 500);
    sim.button = true;
    power_sim_run(&sim, 1900);
    CHECK(sim.p.state == POWER_ON);
    power_sim_run(&sim, 200);
    CHECK(sim.p.state == POWER_HALTING);
    CHECK(sim.outs[OUT_SHUTDOWN_REQ]);
    CHECK(!sim.outs[OUT_CM5_OFF]); /* the CM5 is still writing its SD card */
    power_sim_run(&sim, 5000);
    sim.halted = true;
    power_sim_run(&sim, 900);
    CHECK(sim.p.state == POWER_HALTING);
    power_sim_run(&sim, 200);
    CHECK(sim.p.state == POWER_OFF);
    CHECK(sim.outs[OUT_CM5_OFF]);
    CHECK(bq76942_sim_fets(&sim.bms) == 0);
    CHECK(!sim.bms.shutdown); /* button still held: TS2 low would wake it straight back up */
    sim.button = false;
    power_sim_run(&sim, 10);
    CHECK(sim.bms.shutdown && !sim.bms.awake);
}

static void test_halt_timeout(void)
{
    printf("CM5 never halts\n");
    start(false, true, false);
    sim.button = false;
    power_sim_run(&sim, 100);
    config(OP_WRITE, PKEY_SHUTDOWN, 1, 1);
    CHECK(reply(1, ST_OK, NULL));
    CHECK(sim.p.state == POWER_HALTING);
    power_sim_run(&sim, 29000);
    CHECK(sim.p.state == POWER_HALTING);
    power_sim_run(&sim, 1100);
    CHECK(!sim.bms.awake);

    /* or a second long press cuts it */
    start(false, true, false);
    sim.button = false;
    power_sim_run(&sim, 100);
    press(2100);
    CHECK(sim.p.state == POWER_HALTING);
    press(2100);
    CHECK(!sim.bms.awake);
}

static void test_cm5_halts_itself(void)
{
    printf("CM5 powered off from Linux\n");
    start(false, true, false);
    sim.button = false;
    power_sim_run(&sim, 100);
    sim.halted = true;
    power_sim_run(&sim, 1100);
    CHECK(!sim.bms.awake);
}

static void test_low_battery(void)
{
    printf("low battery\n");
    start(false, true, false);
    sim.button = false;
    power_sim_run(&sim, 100);
    bq76942_sim_cells(&sim.bms, 3250);
    power_sim_run(&sim, 200);
    CHECK(sim.p.flags & PWR_LOW);
    CHECK(sim.p.faults & PWR_FAULT_LOW_CELL);
    power_sim_run(&sim, 9000);
    CHECK(sim.p.state == POWER_ON);
    power_sim_run(&sim, 1000);
    CHECK(sim.p.state == POWER_HALTING);

    /* a dip under load that recovers doesn't count */
    start(false, true, false);
    sim.button = false;
    power_sim_run(&sim, 100);
    bq76942_sim_cells(&sim.bms, 3250);
    power_sim_run(&sim, 5000);
    bq76942_sim_cells(&sim.bms, 3600);
    power_sim_run(&sim, 200);
    bq76942_sim_cells(&sim.bms, 3250);
    power_sim_run(&sim, 6000);
    CHECK(sim.p.state == POWER_ON);
}

static void test_cuv_trip(void)
{
    printf("BMS undervoltage protection\n");
    start(false, true, false);
    sim.button = false;
    bq76942_sim_cells(&sim.bms, 2700);
    power_sim_run(&sim, 1100);
    CHECK(sim.p.faults & PWR_FAULT_BMS);
    struct power_detail_msg pd;
    CHECK(can_unpack_power_detail(last(CAN_POWER_DETAIL | POWER_NODE), &pd));
    CHECK(pd.safety_a & 0x04);
    CHECK(!(pd.fets & BQ_FET_DSG));
}

static void test_estop(void)
{
    printf("e-stop from ROS\n");
    start(false, true, false);
    sim.button = false;
    sync(false);
    power_sim_run(&sim, 10);
    CHECK(!sim.outs[OUT_ESTOP]);
    sync(true);
    power_sim_run(&sim, 10);
    CHECK(sim.outs[OUT_ESTOP]);
    power_sim_run(&sim, 100);
    CHECK(sim.p.flags & PWR_ESTOP);
    sync(false);
    power_sim_run(&sim, 10);
    CHECK(!sim.outs[OUT_ESTOP]);
    /* the CM5 went quiet with the flag set: let go, the legs crouch on their own */
    sync(true);
    power_sim_run(&sim, 600);
    CHECK(!sim.outs[OUT_ESTOP]);
    /* a stop while it's shutting down still holds */
    sync(true);
    config(OP_WRITE, PKEY_SHUTDOWN, 1, 99);
    power_sim_run(&sim, 10);
    CHECK(sim.p.state == POWER_HALTING && sim.outs[OUT_ESTOP]);
}

static void test_charger_watchdog(void)
{
    printf("charger loses its settings\n");
    start(false, true, false);
    sim.button = false;
    power_sim_run(&sim, 60000);
    CHECK(sim.chg.wd_expired == 0); /* fed every second */
    CHECK(bq25798_sim_reg16(&sim.chg, CHG_ICHG) == 300);
    bq25798_sim_init(&sim.chg); /* e.g. its supply dipped */
    power_sim_run(&sim, 1100);
    CHECK(bq25798_sim_reg16(&sim.chg, CHG_ICHG) == 300);
    CHECK(bq25798_sim_reg16(&sim.chg, CHG_VREG) == 1660);
    CHECK(sim.chg.regs[CHG_NTC1] & 1);
    CHECK(sim.p.chg.resets == 1);
}

static void test_config(void)
{
    printf("config over CAN\n");
    start(false, true, false);
    sim.button = false;
    power_sim_run(&sim, 100);
    int32_t v;

    config(OP_WRITE, PKEY_CHARGE_MA, 2000, 1);
    CHECK(reply(1, ST_OK, NULL));
    CHECK(bq25798_sim_reg16(&sim.chg, CHG_ICHG) == 200);
    config(OP_WRITE, PKEY_CHARGE_MA, 9000, 2);
    CHECK(reply(2, ST_BAD_VALUE, NULL));
    config(OP_WRITE, PKEY_CHARGE_MV, 17000, 3);
    CHECK(reply(3, ST_BAD_VALUE, NULL));
    config(OP_WRITE, KEY_VERSION, 1, 4);
    CHECK(reply(4, ST_BAD_KEY, NULL));
    config(OP_READ, KEY_VERSION, 0, 5);
    CHECK(reply(5, ST_OK, &v) && v == 0x1234567);
    config(OP_READ, PKEY_BMS_MEM, 0x9304, 6);
    CHECK(reply(6, ST_OK, &v) && (v & 0xffff) == 0x0207);
    config(OP_READ, PKEY_BMS_MEM, 0x1234, 7);
    CHECK(reply(7, ST_BAD_KEY, NULL));
    config(OP_CALIBRATE, 0, 0, 8);
    CHECK(reply(8, ST_BAD_KEY, NULL));

    config(OP_WRITE, PKEY_CELL_NTC, 2, 9);
    CHECK(reply(9, ST_BAD_VALUE, NULL));
    int cfgupdates = sim.bms.cfgupdates;
    config(OP_WRITE, PKEY_CELL_NTC, 1, 10);
    CHECK(reply(10, ST_OK, NULL));
    power_sim_run(&sim, 300);
    CHECK(sim.bms.cfgupdates == cfgupdates); /* not now: that would open the FETs */
    CHECK(bq76942_sim_fets(&sim.bms) & BQ_FET_DSG);

    config(OP_WRITE, PKEY_SOC, 500, 11);
    CHECK(reply(11, ST_OK, NULL));
    config(OP_READ, PKEY_SOC, 0, 12);
    CHECK(reply(12, ST_OK, &v) && v == 500);

    config(OP_SAVE, 0, 0, 13);
    CHECK(reply(13, ST_OK, NULL));
    const struct power_config *c = config_newest(&sim.flash[0], &sim.flash[1]);
    CHECK(c && c->charge_ma == 2000);
    config(OP_SAVE, 0, 0, 14);
    CHECK(sim.flash[0].seq != sim.flash[1].seq); /* alternates slots */
    sim.flash_fail = true;
    config(OP_SAVE, 0, 0, 15);
    CHECK(reply(15, ST_BAD_VALUE, NULL));
    sim.flash_fail = false;

    /* a restart picks the saved config up, and no config fault */
    power_sim_boot(&sim);
    power_sim_run(&sim, 300);
    CHECK(sim.p.cfg.charge_ma == 2000);
    CHECK(!(sim.p.faults & PWR_FAULT_CONFIG));

    config(OP_DEFAULTS, 0, 0, 16);
    CHECK(reply(16, ST_OK, NULL));
    CHECK(bq25798_sim_reg16(&sim.chg, CHG_ICHG) == 300);
    config(OP_READ, 3, 0, 17);
    CHECK(reply(17, ST_BAD_KEY, NULL));
    struct leg_cfg_msg m = {.joint = 0, .key = PKEY_CAPACITY, .seq = 18, .op = OP_READ};
    struct can_frame_t f;
    can_pack_leg_cfg(&f, CAN_LEG_CONFIG, POWER_NODE, &m);
    power_frame(&sim.p, &f);
    CHECK(reply(18, ST_BAD_KEY, NULL));
}

static void test_gauge(void)
{
    printf("gauge\n");
    CHECK_NEAR(gauge_ocv_soc(2900), 0, 0);
    CHECK_NEAR(gauge_ocv_soc(3725), 45, 0.01);
    CHECK_NEAR(gauge_ocv_soc(4250), 100, 0);

    struct gauge g;
    gauge_start(&g, 8000, -1, 3800);
    CHECK_NEAR(g.soc, 58, 0.01);
    gauge_start(&g, 8000, 50, 3800); /* saved value close enough: trusted */
    CHECK_NEAR(g.soc, 50, 0.01);
    gauge_start(&g, 8000, 90, 3800); /* charged elsewhere: the voltage wins */
    CHECK_NEAR(g.soc, 58, 0.01);
    gauge_update(&g, 1000);
    CHECK_NEAR(g.soc, 58, 0.01); /* first reading is the reference */
    gauge_update(&g, 200);
    CHECK_NEAR(g.soc, 48, 0.01);
    gauge_update(&g, -10000);
    CHECK_NEAR(g.soc, 0, 0);

    /* on the board, walking at 8 A for 6 minutes takes 800 mAh */
    start(false, true, false);
    sim.button = false;
    power_sim_run(&sim, 2000);
    float soc = sim.p.gauge.soc;
    sim.bms.current_ma = -8000;
    power_sim_run(&sim, 360000);
    CHECK_NEAR(soc - sim.p.gauge.soc, 10, 0.05);
    struct power_state_msg ps;
    CHECK(can_unpack_power_state(last(CAN_POWER_STATE | POWER_NODE), &ps));
    CHECK(ps.current_ma == -8000);
}

static void test_bms_comm_loss(void)
{
    printf("BMS stops answering\n");
    start(false, true, false);
    sim.button = false;
    power_sim_run(&sim, 200);
    sim.bms.fail_next = 1000000;
    power_sim_run(&sim, 1000);
    CHECK(sim.p.faults & PWR_FAULT_BMS_COMM);
    CHECK(sim.p.state == POWER_ON); /* the pack stays on, the BMS still protects it */
    sim.bms.fail_next = 0;
    power_sim_run(&sim, 300);
    CHECK(!(sim.p.faults & PWR_FAULT_BMS_COMM));

    printf("BMS unreachable at boot\n");
    power_sim_init(&sim, false);
    sim.tx = capture;
    sim.bms.fail_next = 1000000;
    power_sim_boot(&sim);
    power_sim_run(&sim, 2500);
    CHECK(sim.p.state == POWER_FAULT);
    CHECK(sim.outs[OUT_CM5_OFF]);
    sim.bms.fail_next = 0;
    sim.chg.vbus = true;
    power_sim_run(&sim, 6000);
    CHECK(sim.p.state == POWER_CHARGE);
}

static void test_5v(void)
{
    printf("5 V rail\n");
    start(false, true, false);
    sim.button = false;
    power_sim_run(&sim, 2000);
    CHECK(!(sim.p.faults & PWR_FAULT_5V));
    sim.outs[OUT_CM5_OFF] = true; /* stands in for a dead buck on the carrier */
    power_sim_run(&sim, 600);
    CHECK(sim.p.faults & PWR_FAULT_5V);
}

static void test_frames(void)
{
    printf("power frames\n");
    struct can_frame_t f;
    struct power_state_msg a = {.pack_mv = 16012, .current_ma = -12340, .soc_half = 157, .temp_c = -5,
                                .state = POWER_HALTING, .flags = PWR_LOW | PWR_ESTOP, .faults = 0x81}, b;
    can_pack_power_state(&f, &a);
    CHECK(f.id == 0x047);
    CHECK(can_unpack_power_state(&f, &b));
    CHECK(b.pack_mv == 16012 && b.current_ma == -12340 && b.soc_half == 157 && b.temp_c == -5);
    CHECK(b.state == POWER_HALTING && b.flags == (PWR_LOW | PWR_ESTOP) && b.faults == 0x81);
    struct power_detail_msg d = {.safety_a = 1, .safety_b = 2, .safety_c = 3, .fets = 5, .usb = true,
                                 .chg_stat = 7, .charger_fault = 9, .input_mv = 19800, .fet_temp_c = -3}, e;
    can_pack_power_detail(&f, &d);
    CHECK(f.id == 0x0A7);
    CHECK(can_unpack_power_detail(&f, &e));
    CHECK(e.safety_c == 3 && e.fets == 5 && e.usb && e.chg_stat == 7);
    CHECK(e.input_mv == 19800 && e.fet_temp_c == -3);
}

static void test_cell_ntc(void)
{
    printf("cell thermistor\n");
    /* none fitted: TS1 open reads as deep cold, the die temperature is used instead */
    start(false, true, false);
    sim.button = false;
    sim.bms.ts1_c = -40;
    sim.bms.int_c = 31;
    power_sim_run(&sim, 300);
    CHECK(sim.p.bat.cell_temp_c == 31);
    struct power_state_msg ps;
    CHECK(can_unpack_power_state(last(CAN_POWER_STATE | POWER_NODE), &ps));
    CHECK(ps.temp_c == 31);

    /* fitted: saved, and the BMS takes it at the next power-up */
    config(OP_WRITE, PKEY_CELL_NTC, 1, 1);
    config(OP_SAVE, 0, 0, 2);
    CHECK(reply(2, ST_OK, NULL));
    sim.bms.ts1_c = 52;
    power_sim_boot(&sim);
    power_sim_run(&sim, 300);
    CHECK(sim.bms.mem[0x92FD - BQS_MEM_BASE] == 0x07);
    CHECK(sim.bms.mem[0x9303 - BQS_MEM_BASE] == 0x01);
    CHECK(sim.p.bat.cell_temp_c == 52);
    CHECK(sim.p.faults & PWR_FAULT_HOT);
}

int main(void)
{
    test_frames();
    test_gauge();
    test_button_wake(false);
    test_button_wake(true);
    test_mcu_reset_keeps_power();
    test_crc_errors();
    test_charger_wake();
    test_spurious_wake();
    test_long_press_shutdown();
    test_halt_timeout();
    test_cm5_halts_itself();
    test_low_battery();
    test_cuv_trip();
    test_estop();
    test_charger_watchdog();
    test_config();
    test_bms_comm_loss();
    test_5v();
    test_cell_ntc();
    printf("%d checks, %d failed\n", checks, failures);
    return failures != 0;
}
