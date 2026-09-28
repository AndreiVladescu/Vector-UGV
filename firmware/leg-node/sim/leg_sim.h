#ifndef LEG_SIM_H
#define LEG_SIM_H

#include "hal.h"
#include "leg.h"
#include "servo_sim.h"

/* One leg cell with three simulated servos behind the leg_hal interface. */
struct leg_sim {
    struct leg leg;
    struct leg_hal hal;
    struct servo_sim servo[JOINTS];
    uint32_t t_ms;
    uint8_t id;
    bool buck, estop, pg_fail;
    uint16_t load_ma, ntc_mv;       /* leg current while the buck is on, NTC voltage */
    uint16_t i_offset_mv, vbat_adc_mv, tof;
    struct leg_config flash[2];
    bool flash_written, flash_fail; /* flash_fail: the next save dies half way */
    void (*tx)(void *user, const struct can_frame_t *f);
    void *user;
};

void leg_sim_init(struct leg_sim *s, uint8_t id, const struct leg_config *stored);
/* Advance 1 ms: servos move, then the leg logic runs. */
void leg_sim_tick(struct leg_sim *s);

#endif
