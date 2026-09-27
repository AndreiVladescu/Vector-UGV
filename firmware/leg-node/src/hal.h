#ifndef HAL_H
#define HAL_H

#include <stdbool.h>
#include <stdint.h>

#include "vector_can.h"

enum adc_ch { ADC_POT0, ADC_POT1, ADC_POT2, ADC_I_LEG, ADC_NTC, ADC_VBAT, ADC_6V0, ADC_CHANNELS };

/* Everything the leg logic needs from the board. */
struct leg_hal {
    void *ctx;
    uint32_t (*now_ms)(void *ctx);
    void (*pwm_us)(void *ctx, int joint, int us); /* 0 stops the pulses */
    void (*adc_mv)(void *ctx, int ch, uint16_t *out, int n);
    void (*buck)(void *ctx, bool on);
    bool (*buck_good)(void *ctx);
    bool (*estop)(void *ctx); /* true while the e-stop line is pulled low */
    uint8_t (*leg_id)(void *ctx);
    uint16_t (*tof_mm)(void *ctx);
    void (*send)(void *ctx, const struct can_frame_t *f);
    bool (*save)(void *ctx, const void *data, int len);
};

#endif
