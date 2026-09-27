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
};

enum leg_state { LEG_OFF, LEG_WAKE, LEG_ACTIVE, LEG_CROUCH, LEG_CALIBRATE, LEG_FAULT };

enum {
    FAULT_WATCHDOG = 1 << 0,
    FAULT_ESTOP = 1 << 1,
    FAULT_WAKE = 1 << 2,
    FAULT_UNCALIBRATED = 1 << 3,
    FAULT_BUCK = 1 << 4,
    FAULT_OVERTEMP = 1 << 5,
    FAULT_CAL = 1 << 6,
    FAULT_CONFIG = 1 << 7,
};

enum cfg_op { OP_READ, OP_WRITE, OP_SAVE, OP_CALIBRATE, OP_DEFAULTS };

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
};

enum cfg_status { ST_OK, ST_BAD_KEY, ST_BAD_VALUE, ST_BUSY, ST_CAL_RESULT, ST_CAL_FAILED, ST_CAL_DONE };

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

static inline uint32_t can_function(uint32_t id) { return id & 0x7f0; }
static inline uint8_t can_node(uint32_t id) { return id & 0x00f; }

void can_pack_sync(struct can_frame_t *f, const struct sync_msg *m);
void can_pack_leg_cmd(struct can_frame_t *f, uint8_t node, const struct leg_cmd_msg *m);
void can_pack_leg_state(struct can_frame_t *f, uint8_t node, const struct leg_state_msg *m);
void can_pack_leg_status(struct can_frame_t *f, uint8_t node, const struct leg_status_msg *m);
void can_pack_leg_cfg(struct can_frame_t *f, uint32_t function, uint8_t node, const struct leg_cfg_msg *m);

bool can_unpack_sync(const struct can_frame_t *f, struct sync_msg *m);
bool can_unpack_leg_cmd(const struct can_frame_t *f, struct leg_cmd_msg *m);
bool can_unpack_leg_state(const struct can_frame_t *f, struct leg_state_msg *m);
bool can_unpack_leg_status(const struct can_frame_t *f, struct leg_status_msg *m);
/* LEG_CONFIG and LEG_REPLY share a layout; the last byte is op or status. */
bool can_unpack_leg_cfg(const struct can_frame_t *f, struct leg_cfg_msg *m);

#endif
