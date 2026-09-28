#include "leg_sim.h"

#include <string.h>

#include "board.h"

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
        case ADC_I_LEG: out[i] = s->i_offset_mv + (s->buck ? (uint16_t)(s->load_ma * INA_GAIN * SHUNT_MOHM / 1000) : 0); break;
        case ADC_NTC: out[i] = s->ntc_mv; break;
        case ADC_VBAT: out[i] = s->vbat_adc_mv; break;
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

static bool buck_good(void *ctx) { struct leg_sim *s = ctx; return s->buck && !s->pg_fail; }
static bool estop(void *ctx) { return ((struct leg_sim *)ctx)->estop; }
static uint8_t leg_id(void *ctx) { return ((struct leg_sim *)ctx)->id; }
static uint16_t tof_mm(void *ctx) { return ((struct leg_sim *)ctx)->tof; }

static uint32_t diag(void *ctx, uint8_t key)
{
    (void)ctx;
    return key == KEY_VERSION ? 0x0abcdefu : key == KEY_RESET_CAUSE ? RESET_POWER : 0;
}

static void send(void *ctx, const struct can_frame_t *f)
{
    struct leg_sim *s = ctx;
    if (s->tx)
        s->tx(s->user, f);
}

static bool save(void *ctx, int slot, const void *data, int len)
{
    struct leg_sim *s = ctx;
    if (len != sizeof(s->flash[0]) || slot < 0 || slot > 1)
        return false;
    if (s->flash_fail) {
        memset(&s->flash[slot], 0xff, sizeof(s->flash[slot])); /* erased, then the power went */
        s->flash_fail = false;
        return false;
    }
    memcpy(&s->flash[slot], data, sizeof(s->flash[slot]));
    s->flash_written = true;
    return true;
}

void leg_sim_init(struct leg_sim *s, uint8_t id, const struct leg_config *stored)
{
    memset(s, 0, sizeof(*s));
    s->id = id;
    s->load_ma = 500;
    s->ntc_mv = 1650;      /* 25 C */
    s->vbat_adc_mv = 2000; /* ~15.3 V */
    s->tof = 400;
    s->hal = (struct leg_hal){s, now_ms, pwm_us, adc_mv, buck, buck_good, estop, leg_id, tof_mm, send, save, diag};
    for (int j = 0; j < JOINTS; j++)
        servo_sim_init(&s->servo[j], 1560.0f, 1.433f, 1500.0f);
    leg_init(&s->leg, &s->hal, stored, NULL);
}

void leg_sim_tick(struct leg_sim *s)
{
    s->t_ms++;
    for (int j = 0; j < JOINTS; j++)
        servo_sim_step(&s->servo[j], 0.001f);
    leg_tick(&s->leg);
}
