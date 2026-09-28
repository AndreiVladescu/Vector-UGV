#ifndef LEG_H
#define LEG_H

#include "config.h"
#include "filter.h"
#include "hal.h"

#define CAL_POINTS 11 /* per direction */

struct wake {
    int joint; /* -1 while waiting for the buck */
    uint8_t phase, tries;
    uint32_t t;
    float start_us, test_us, before_us;
};

struct test {
    uint8_t phase, failed;
    uint32_t t;
};

struct cal {
    uint8_t mask, joint, phase, point;
    enum cal_mode mode;
    uint32_t t;
    float lo, hi;
    float x[2 * CAL_POINTS], y[2 * CAL_POINTS];
};

struct leg {
    const struct leg_hal *hal;
    struct leg_config cfg;
    uint8_t node;
    enum leg_state state;
    uint8_t faults;
    bool enable;
    uint32_t entered, last_sync, last_idle_tx;
    uint8_t sync_count;
    bool synced;
    bool powered;
    uint32_t power_t;

    float mv[JOINTS];
    float pos_deg[JOINTS];
    float target_deg[JOINTS];
    float out_us[JOINTS]; /* 0 = no pulses */

    uint16_t current_ma, vbat_mv, rail_mv;
    int8_t temp_c;
    bool hot;
    uint16_t over_ms, pg_bad_ms;
    float i_zero_mv;
    uint32_t off_t;

    struct wake wake;
    struct cal cal;
    struct test test;
};

/* The two config slots from flash; NULL or invalid ones are skipped, defaults if neither is good. */
void leg_init(struct leg *l, const struct leg_hal *hal, const struct leg_config *slot0, const struct leg_config *slot1);
void leg_frame(struct leg *l, const struct can_frame_t *f);
/* Call every millisecond. */
void leg_tick(struct leg *l);

#endif
