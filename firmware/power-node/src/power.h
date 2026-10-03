#ifndef POWER_H
#define POWER_H

#include "bq25798.h"
#include "bq76942.h"
#include "config.h"
#include "gauge.h"

#define LONG_PRESS_MS 2000
#define HALT_TIMEOUT_MS 30000
#define LOW_CELL_MS 10000

struct power {
    const struct power_hal *hal;
    struct power_config cfg;
    struct bq76942 bms;
    struct bq25798 chg;
    struct gauge gauge;

    enum power_state state;
    uint32_t entered;
    uint8_t faults, flags;
    bool sync_estop;
    uint32_t last_sync;

    struct bq_reading bat;
    struct charger_reading ch;
    bool bat_ok, ch_ok;
    uint8_t bms_fails, boot_tries;
    uint32_t t_bms, t_chg, t_tx, t_slow;

    bool pressed, long_done; /* button */
    uint32_t press_t;
    uint32_t low_t, halted_t, no_input_t, pg_bad_t, off_t;
};

/* The two config slots from flash; NULL or invalid ones are skipped, defaults if neither is good. */
void power_init(struct power *p, const struct power_hal *hal, uint16_t cell_mask, bool bms_crc,
                const struct power_config *slot0, const struct power_config *slot1);
void power_frame(struct power *p, const struct can_frame_t *f);
/* call every millisecond */
void power_tick(struct power *p);

#endif
