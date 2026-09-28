#ifndef TOF_SIM_H
#define TOF_SIM_H

#include <stdbool.h>
#include <stdint.h>

/* VL53L1X register model: enough of it for the driver's init and polling paths. */
struct tof_sim {
    uint8_t reg[0x200];
    bool present, ranging, ready;
    bool xshut_low, stuck; /* stuck: answers nothing until reset through XSHUT */
    uint32_t t_ms, next_ms;
    uint16_t distance_mm;
    uint8_t raw_status; /* as the sensor reports it, 9 = valid */
    int max_write, failed_reads;
};

void tof_sim_init(struct tof_sim *s);
void tof_sim_step(struct tof_sim *s, uint32_t ms);
bool tof_sim_write(void *ctx, uint16_t reg, const uint8_t *data, int n);
bool tof_sim_read(void *ctx, uint16_t reg, uint8_t *data, int n);
void tof_sim_shutdown(void *ctx, bool off);

#endif
