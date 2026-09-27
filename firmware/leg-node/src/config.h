#ifndef CONFIG_H
#define CONFIG_H

#include "joint.h"

#define CONFIG_MAGIC 0x56454731u /* "VEG1" */

struct leg_config {
    uint32_t magic;
    struct joint_cfg joint[JOINTS];
    uint32_t crc;
};

void config_defaults(struct leg_config *c);
void config_seal(struct leg_config *c);
bool config_valid(const struct leg_config *c);
bool config_calibrated(const struct leg_config *c);

#endif
