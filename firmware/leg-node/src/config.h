#ifndef CONFIG_H
#define CONFIG_H

#include "joint.h"

#define CONFIG_MAGIC 0x56454733u /* "VEG3" */

struct leg_config {
    uint32_t magic;
    struct joint_cfg joint[JOINTS];
    float vbat_gain, rail_gain; /* divider corrections, 1.0 = nominal */
    uint32_t seq; /* bumped on every save; two flash slots, the newer valid one wins */
    uint32_t crc;
};

void config_defaults(struct leg_config *c);
void config_seal(struct leg_config *c);
bool config_valid(const struct leg_config *c);
bool config_calibrated(const struct leg_config *c);
/* either may be NULL; NULL when neither is valid */
const struct leg_config *config_newest(const struct leg_config *a, const struct leg_config *b);

#endif
