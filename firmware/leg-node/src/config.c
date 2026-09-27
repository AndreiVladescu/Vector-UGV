#include "config.h"

#include <stddef.h>
#include <string.h>

static uint32_t crc32(const void *data, size_t len)
{
    const uint8_t *p = data;
    uint32_t crc = 0xffffffffu;
    while (len--) {
        crc ^= *p++;
        for (int i = 0; i < 8; i++)
            crc = (crc >> 1) ^ (0xedb88320u & -(crc & 1));
    }
    return ~crc;
}

void config_defaults(struct leg_config *c)
{
    memset(c, 0, sizeof(*c));
    c->magic = CONFIG_MAGIC;
    for (int j = 0; j < JOINTS; j++) {
        /* average of the 18 bench-calibrated servos, good enough to find the servo */
        c->joint[j] = (struct joint_cfg){
            .mid_mv = 1560.0f, .slope = 1.433f,
            .center_us = 1500.0f, .dir = 1, .us_per_deg = 11.11f,
            .min_deg = -80.0f, .max_deg = 80.0f,
        };
    }
    config_seal(c);
}

void config_seal(struct leg_config *c)
{
    c->crc = crc32(c, offsetof(struct leg_config, crc));
}

bool config_valid(const struct leg_config *c)
{
    return c->magic == CONFIG_MAGIC && c->crc == crc32(c, offsetof(struct leg_config, crc));
}

bool config_calibrated(const struct leg_config *c)
{
    for (int j = 0; j < JOINTS; j++)
        if (!c->joint[j].calibrated)
            return false;
    return true;
}
