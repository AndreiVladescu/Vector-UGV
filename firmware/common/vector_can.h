#ifndef VECTOR_CAN_H
#define VECTOR_CAN_H

/* Leg bus frames, see protocol/vector.dbc. ID = (function << 4) | node. */

#include <stdbool.h>
#include <stdint.h>

enum {
    CAN_SYNC = 0x000,
    CAN_LEG_CMD = 0x010,
    CAN_LEG_STATE = 0x020,
    CAN_LEG_STATUS = 0x030,
    CAN_POWER_STATE = 0x040,
    CAN_LEG_CONFIG = 0x050,
    CAN_LEG_REPLY = 0x060,
    CAN_BOOT = 0x070,
    CAN_BOOT_REPLY = 0x080,
    CAN_POWER_CELLS = 0x090,
    CAN_POWER_DETAIL = 0x0A0,
};

/* the power board is node 7; it takes LEG_CONFIG / LEG_REPLY and BOOT frames like a leg */
#define POWER_NODE 7

enum power_state { POWER_BOOT, POWER_CHARGE, POWER_ON, POWER_HALTING, POWER_OFF, POWER_FAULT };

/* POWER_STATE flags (4 bits) */
enum { PWR_CHARGER = 1 << 0, PWR_CHARGING = 1 << 1, PWR_ESTOP = 1 << 2, PWR_LOW = 1 << 3 };

/* POWER_STATE faults */
enum {
    PWR_FAULT_BMS = 1 << 0,       /* a BMS protection has tripped, see POWER_DETAIL */
    PWR_FAULT_BMS_COMM = 1 << 1,
    PWR_FAULT_CHARGER = 1 << 2,
    PWR_FAULT_5V = 1 << 3,        /* 5 V rail not good while on */
    PWR_FAULT_LOW_CELL = 1 << 4,
    PWR_FAULT_IMBALANCE = 1 << 5, /* cells more than 100 mV apart */
    PWR_FAULT_CONFIG = 1 << 6,
    PWR_FAULT_HOT = 1 << 7,
};

/* power board config keys, joint 0xff; KEY_VERSION .. KEY_UPTIME work too */
enum power_key {
    PKEY_CAPACITY = 32, /* mAh */
    PKEY_CHARGE_MA,     /* charge current */
    PKEY_CHARGE_MV,     /* pack charge voltage */
    PKEY_INPUT_MA,      /* charger input current limit */
    PKEY_LOW_MV,        /* cell voltage that shuts the robot down */
    PKEY_CELL_NTC,      /* 1 = a thermistor on J-NTC (DIY pack), 0 = none, the BMS die temperature
                           stands in; saved, applies at the next power-up */
    PKEY_SOC,           /* 0.1 %, write to correct the gauge */
    PKEY_BMS_MEM,       /* read: value = BQ76942 data memory address, reply = 4 bytes from there */
    PKEY_SHUTDOWN,      /* write 1: ask the CM5 to halt, then power off */
};

enum leg_state { LEG_OFF, LEG_WAKE, LEG_ACTIVE, LEG_CROUCH, LEG_CALIBRATE, LEG_FAULT, LEG_TEST };

enum {
    FAULT_WATCHDOG = 1 << 0,
    FAULT_ESTOP = 1 << 1,
    FAULT_WAKE = 1 << 2,
    FAULT_UNCALIBRATED = 1 << 3,
    FAULT_BUCK = 1 << 4,
    FAULT_OVERLOAD = 1 << 5, /* overcurrent or overtemperature */
    FAULT_CAL = 1 << 6,
    FAULT_CONFIG = 1 << 7,
};

enum cfg_op { OP_READ, OP_WRITE, OP_SAVE, OP_CALIBRATE, OP_DEFAULTS, OP_SELFTEST };

enum cfg_key {
    KEY_WIPER_MID = 1,   /* 0.1 mV at 1500 us */
    KEY_WIPER_SLOPE,     /* 1e-4 mV/us */
    KEY_CENTER_US,       /* 0.1 us at joint angle 0 */
    KEY_DIRECTION,       /* +1 / -1 */
    KEY_US_PER_DEG,      /* 0.001 us */
    KEY_MIN_DEG,         /* 0.01 deg */
    KEY_MAX_DEG,         /* 0.01 deg */
    KEY_FIT_ERR,         /* 0.1 mV, read only */
    KEY_CALIBRATED,      /* 0 / 1, read only */

    /* leg-wide, joint 0xff */
    KEY_VERSION = 16,    /* firmware build, read only */
    KEY_RESET_CAUSE,     /* RESET_* bits of the last reset, read only */
    KEY_CAN_ERRORS,      /* bus-off count << 16 | dropped tx frames, read only */
    KEY_UPTIME,          /* s, read only */
    KEY_VBAT_GAIN,       /* 1e-4, divider correction */
    KEY_RAIL_GAIN,       /* 1e-4 */
    KEY_I_ZERO,          /* mA, current amp offset measured while off, read only */
    KEY_ID_STRAPS,       /* leg ID jumpers, side << 2 | position, read only */
};

enum { RESET_POWER = 1, RESET_PIN = 2, RESET_WATCHDOG = 4, RESET_SOFTWARE = 8, RESET_OTHER = 16 };

enum cfg_status {
    ST_OK, ST_BAD_KEY, ST_BAD_VALUE, ST_BUSY, ST_CAL_RESULT, ST_CAL_FAILED, ST_CAL_DONE,
    ST_TEST_PASS, ST_TEST_FAIL, ST_TEST_DONE,
};

/* self-test items, the key of each TEST_PASS / TEST_FAIL reply */
enum test_item {
    TEST_I_ZERO = 1, TEST_RAIL_OFF, TEST_VBAT, TEST_TEMP, /* buck off, mA / mV / mV / C */
    TEST_POWER_GOOD, TEST_RAIL_ON, TEST_I_ON,             /* buck on, 0/1 / mV / mA */
    TEST_TOF, TEST_POT0, TEST_POT1, TEST_POT2,            /* mm, wiper mV (pass: a servo is there) */
};

enum cal_mode { CAL_FULL, CAL_LIMITS };

struct can_frame_t {
    uint32_t id;
    uint8_t len;
    uint8_t data[8];
};

struct sync_msg { uint16_t counter; uint8_t mode; bool estop; };
struct leg_cmd_msg { float deg[3]; bool enable; uint8_t counter; };
struct leg_state_msg { float deg[3]; uint16_t current_ma; };
struct leg_status_msg { uint16_t tof_mm, vbat_mv, rail_mv; int8_t temp_c; uint8_t faults, state; };
struct leg_cfg_msg { uint8_t joint, key; int32_t value; uint8_t seq, op; };
struct power_state_msg { uint16_t pack_mv; int32_t current_ma; uint8_t soc_half; int8_t temp_c; uint8_t state, flags, faults; };
struct power_cells_msg { uint16_t cell_mv[4]; };
struct power_detail_msg {
    uint8_t safety_a, safety_b, safety_c;
    uint8_t fets;     /* BQ76942 FET Status bits 0-3: CHG, PCHG, DSG, PDSG */
    bool usb;         /* something on the USB-C port */
    uint8_t chg_stat; /* BQ25798 CHG_STAT */
    uint8_t charger_fault;
    uint16_t input_mv; /* USB-C VBUS */
    int8_t fet_temp_c;
};

static inline uint32_t can_function(uint32_t id) { return id & 0x7f0; }
static inline uint8_t can_node(uint32_t id) { return id & 0x00f; }

void can_pack_sync(struct can_frame_t *f, const struct sync_msg *m);
void can_pack_leg_cmd(struct can_frame_t *f, uint8_t node, const struct leg_cmd_msg *m);
void can_pack_leg_state(struct can_frame_t *f, uint8_t node, const struct leg_state_msg *m);
void can_pack_leg_status(struct can_frame_t *f, uint8_t node, const struct leg_status_msg *m);
void can_pack_leg_cfg(struct can_frame_t *f, uint32_t function, uint8_t node, const struct leg_cfg_msg *m);
void can_pack_power_state(struct can_frame_t *f, const struct power_state_msg *m);
void can_pack_power_cells(struct can_frame_t *f, const struct power_cells_msg *m);
void can_pack_power_detail(struct can_frame_t *f, const struct power_detail_msg *m);

bool can_unpack_sync(const struct can_frame_t *f, struct sync_msg *m);
bool can_unpack_leg_cmd(const struct can_frame_t *f, struct leg_cmd_msg *m);
bool can_unpack_leg_state(const struct can_frame_t *f, struct leg_state_msg *m);
bool can_unpack_leg_status(const struct can_frame_t *f, struct leg_status_msg *m);
/* LEG_CONFIG and LEG_REPLY share a layout; the last byte is op or status. */
bool can_unpack_leg_cfg(const struct can_frame_t *f, struct leg_cfg_msg *m);
bool can_unpack_power_state(const struct can_frame_t *f, struct power_state_msg *m);
bool can_unpack_power_cells(const struct can_frame_t *f, struct power_cells_msg *m);
bool can_unpack_power_detail(const struct can_frame_t *f, struct power_detail_msg *m);

#endif
