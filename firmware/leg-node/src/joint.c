#include "joint.h"

float joint_deg_to_us(const struct joint_cfg *j, float deg)
{
    return j->center_us + j->dir * deg * j->us_per_deg;
}

float joint_us_to_deg(const struct joint_cfg *j, float us)
{
    return (us - j->center_us) / (j->dir * j->us_per_deg);
}

float joint_mv_to_us(const struct joint_cfg *j, float mv)
{
    return 1500.0f + (mv - j->mid_mv) / j->slope;
}

float joint_clamp_deg(const struct joint_cfg *j, float deg)
{
    if (deg < j->min_deg) return j->min_deg;
    if (deg > j->max_deg) return j->max_deg;
    return deg;
}

void joint_us_range(const struct joint_cfg *j, float *lo, float *hi)
{
    float a = joint_deg_to_us(j, j->min_deg), b = joint_deg_to_us(j, j->max_deg);
    *lo = a < b ? a : b;
    *hi = a < b ? b : a;
}
