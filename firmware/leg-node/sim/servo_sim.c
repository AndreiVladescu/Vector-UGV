#include "servo_sim.h"

#include <math.h>

void servo_sim_init(struct servo_sim *s, float mid_mv, float slope, float pos_us)
{
    *s = (struct servo_sim){
        .mid_mv = mid_mv, .slope = slope, .pos_us = pos_us,
        .speed_us_s = 4700.0f, .spike_mv = 300.0f, .rng = 12345,
    };
}

void servo_sim_power(struct servo_sim *s, bool on)
{
    s->powered = on;
    if (!on) {
        s->pulses = false;
        s->stuck = false;
    }
}

void servo_sim_pulse(struct servo_sim *s, int us)
{
    if (!s->powered)
        return;
    if (us == 0) {
        s->pulses = false;
        return;
    }
    if (!s->pulses && us < s->pos_us - 8.0f)
        s->stuck = true;
    if (s->stuck && us > s->pos_us + 8.0f)
        s->stuck = false;
    s->pulses = true;
    s->last_us = us;
}

void servo_sim_step(struct servo_sim *s, float dt)
{
    if (!s->powered || !s->pulses || s->stuck || s->dead)
        return;
    float err = s->last_us - s->pos_us, step = s->speed_us_s * dt;
    s->pos_us += fmaxf(-step, fminf(step, err));
}

static uint32_t next(struct servo_sim *s)
{
    s->rng = s->rng * 1664525u + 1013904223u;
    return s->rng >> 8;
}

uint16_t servo_sim_wiper_mv(struct servo_sim *s)
{
    if (!s->powered)
        return 0;
    float mv = s->mid_mv + s->slope * (s->pos_us - 1500.0f);
    mv += (float)(next(s) % 21) - 10.0f;
    if (s->spike_mv > 0 && next(s) % 20 == 0)
        mv += (next(s) & 1 ? 1.0f : -1.0f) * s->spike_mv;
    if (mv < 0) mv = 0;
    if (mv > 3300) mv = 3300;
    return (uint16_t)mv;
}
