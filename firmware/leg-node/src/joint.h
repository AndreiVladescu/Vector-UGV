#ifndef JOINT_H
#define JOINT_H

#include <stdbool.h>
#include <stdint.h>

enum { COXA, FEMUR, TIBIA, JOINTS };

/*
 * Wiper:  mV = mid + slope * (us - 1500)          (per servo, from calibration)
 * Angle:  us = center_us + dir * deg * us_per_deg (per joint, from the mechanics)
 */
struct joint_cfg {
    float mid_mv;
    float slope;
    float center_us;
    int8_t dir;
    float us_per_deg;
    float min_deg, max_deg;
    float fit_err_mv;
    bool calibrated;
};

float joint_deg_to_us(const struct joint_cfg *j, float deg);
float joint_us_to_deg(const struct joint_cfg *j, float us);
float joint_mv_to_us(const struct joint_cfg *j, float mv);
float joint_clamp_deg(const struct joint_cfg *j, float deg);
/* Pulse range covering min..max deg, low to high. */
void joint_us_range(const struct joint_cfg *j, float *lo, float *hi);

#endif
