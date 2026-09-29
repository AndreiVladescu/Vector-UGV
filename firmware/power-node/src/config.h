#ifndef CONFIG_H
#define CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#define CONFIG_MAGIC 0x56455031u /* "VEP1" */

struct power_config {
    uint32_t magic;
    uint16_t capacity_mah, charge_ma, charge_mv, input_ma, low_mv;
    uint16_t reserved;
    float soc; /* at the last power-off, -1 = unknown */
    uint32_t seq; /* bumped on every save; two flash slots, the newer valid one wins */
    uint32_t crc;
};

void config_defaults(struct power_config *c);
void config_seal(struct power_config *c);
bool config_valid(const struct power_config *c);
/* either may be NULL; NULL when neither is valid */
const struct power_config *config_newest(const struct power_config *a, const struct power_config *b);

#endif
