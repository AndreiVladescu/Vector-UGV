#ifndef BQ25798_SIM_H
#define BQ25798_SIM_H

/* BQ25798 registers with the PROG defaults (4S), the 40 s watchdog that restores them,
   input detection and a charging state. */

#include <stdbool.h>
#include <stdint.h>

struct bq25798_sim {
    uint8_t regs[0x49];
    bool vbus, done;  /* something on USB-C; VAC1 and VAC2 are tied to VBUS on the board */
    uint16_t vbus_mv, vbat_mv;
    uint32_t wd_ms; /* since the last WD_RST */
    int wd_expired, fail_next;
};

void bq25798_sim_init(struct bq25798_sim *s);
bool bq25798_sim_write(struct bq25798_sim *s, const uint8_t *d, int n);
bool bq25798_sim_read(struct bq25798_sim *s, uint8_t reg, uint8_t *out, int n);
void bq25798_sim_step(struct bq25798_sim *s, uint32_t ms);
uint16_t bq25798_sim_reg16(const struct bq25798_sim *s, uint8_t reg);

#endif
