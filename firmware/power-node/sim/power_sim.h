#ifndef POWER_SIM_H
#define POWER_SIM_H

/* The power board around the real power logic: both TI chips, the GPIOs, flash, and the
   MCU only running while the BMS's REG1 is up. */

#include "bq25798_sim.h"
#include "bq76942_sim.h"
#include "power.h"

struct power_sim {
    uint32_t t;
    struct bq76942_sim bms;
    struct bq25798_sim chg;
    bool outs[OUTPUTS];
    bool button, halted;
    struct power_config flash[2];
    bool flash_fail;
    int saves;
    void (*tx)(void *user, const struct can_frame_t *f);
    void *user;
    struct power_hal hal;
    struct power p;
};

void power_sim_init(struct power_sim *s, bool crc);
/* MCU reset: power_init from the simulated flash, chips untouched */
void power_sim_boot(struct power_sim *s);
void power_sim_run(struct power_sim *s, uint32_t ms);
bool power_sim_mcu_on(const struct power_sim *s);

#endif
