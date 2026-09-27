#ifndef SERVO_SIM_H
#define SERVO_SIM_H

#include <stdbool.h>
#include <stdint.h>

/* MG996R as measured on the bench (docs/servos.md), including the restart quirk. */
struct servo_sim {
    float mid_mv, slope;  /* wiper = mid + slope * (pos - 1500) */
    float pos_us;         /* where the shaft is, in pulse terms */
    float speed_us_s;     /* ~0.14 s / 60 deg at 6 V */
    float spike_mv;       /* occasional noise spike size, 0 for none */
    bool powered, pulses, stuck, dead;
    int last_us;
    uint32_t rng;
};

void servo_sim_init(struct servo_sim *s, float mid_mv, float slope, float pos_us);
void servo_sim_power(struct servo_sim *s, bool on);
void servo_sim_pulse(struct servo_sim *s, int us); /* 0 = pulses stop */
void servo_sim_step(struct servo_sim *s, float dt);
uint16_t servo_sim_wiper_mv(struct servo_sim *s);

#endif
