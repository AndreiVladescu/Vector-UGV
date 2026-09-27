#include "leg_sim.h"

#include <string.h>

static uint32_t now_ms(void *ctx) { return ((struct leg_sim *)ctx)->t_ms; }

static void pwm_us(void *ctx, int joint, int us)
{
    servo_sim_pulse(&((struct leg_sim *)ctx)->servo[joint], us);
}

static void adc_mv(void *ctx, int ch, uint16_t *out, int n)
{
    struct leg_sim *s = ctx;
    for (int i = 0; i < n; i++) {
        switch (ch) {
        case ADC_POT0: case ADC_POT1: case ADC_POT2:
            out[i] = servo_sim_wiper_mv(&s->servo[ch - ADC_POT0]);
            break;
        case ADC_I_LEG: out[i] = s->buck ? 100 : 0; break;  /* 500 mA */
        case ADC_NTC: out[i] = 1650; break;                 /* 25 C */
        case ADC_VBAT: out[i] = 2000; break;                /* ~15.3 V */
        case ADC_6V0: out[i] = s->buck ? 2430 : 0; break;   /* ~6.0 V */
        default: out[i] = 0;
        }
    }
}

static void buck(void *ctx, bool on)
{
    struct leg_sim *s = ctx;
    s->buck = on;
    for (int j = 0; j < JOINTS; j++)
        servo_sim_power(&s->servo[j], on);
}

static bool buck_good(void *ctx) { return ((struct leg_sim *)ctx)->buck; }
static bool estop(void *ctx) { return ((struct leg_sim *)ctx)->estop; }
static uint8_t leg_id(void *ctx) { return ((struct leg_sim *)ctx)->id; }
static uint16_t tof_mm(void *ctx) { (void)ctx; return 400; }

static void send(void *ctx, const struct can_frame_t *f)
{
    struct leg_sim *s = ctx;
    if (s->tx)
        s->tx(s->user, f);
}

static bool save(void *ctx, const void *data, int len)
{
    struct leg_sim *s = ctx;
    if (len != sizeof(s->flash))
        return false;
    memcpy(&s->flash, data, sizeof(s->flash));
    s->flash_written = true;
    return true;
}

void leg_sim_init(struct leg_sim *s, uint8_t id, const struct leg_config *stored)
{
    memset(s, 0, sizeof(*s));
    s->id = id;
    s->hal = (struct leg_hal){s, now_ms, pwm_us, adc_mv, buck, buck_good, estop, leg_id, tof_mm, send, save};
    for (int j = 0; j < JOINTS; j++)
        servo_sim_init(&s->servo[j], 1560.0f, 1.433f, 1500.0f);
    leg_init(&s->leg, &s->hal, stored);
}

void leg_sim_tick(struct leg_sim *s)
{
    s->t_ms++;
    for (int j = 0; j < JOINTS; j++)
        servo_sim_step(&s->servo[j], 0.001f);
    leg_tick(&s->leg);
}
