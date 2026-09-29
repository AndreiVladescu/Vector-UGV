#include "config.h"

#include <stddef.h>
#include <string.h>

#include "boot.h"

void config_defaults(struct power_config *c)
{
    memset(c, 0, sizeof(*c));
    c->magic = CONFIG_MAGIC;
    c->capacity_mah = 8000; /* 4S3P of ~2.7 Ah scooter cells, derated */
    c->charge_ma = 3000;    /* 60 W from 20 V is about 3.4 A into the pack */
    c->charge_mv = 16600;   /* 4.15 V per cell: a little capacity for a lot of cycle life */
    c->input_ma = 3000;
    c->low_mv = 3300;
    c->soc = -1;
    config_seal(c);
}

void config_seal(struct power_config *c)
{
    c->crc = boot_crc32(0, (const uint8_t *)c, offsetof(struct power_config, crc));
}

bool config_valid(const struct power_config *c)
{
    return c->magic == CONFIG_MAGIC && c->crc == boot_crc32(0, (const uint8_t *)c, offsetof(struct power_config, crc));
}

const struct power_config *config_newest(const struct power_config *a, const struct power_config *b)
{
    bool va = a && config_valid(a), vb = b && config_valid(b);
    if (va && vb)
        return (int32_t)(a->seq - b->seq) >= 0 ? a : b;
    return va ? a : vb ? b : NULL;
}
