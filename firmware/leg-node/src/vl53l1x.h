#ifndef VL53L1X_H
#define VL53L1X_H

#include <stdbool.h>
#include <stdint.h>

/* VL53L1X ranging, following ST's ultra lite driver (the one SparkFun ships; Pololu's
   library writes the same registers). Registers are 16-bit, multi-byte values big-endian. */

#define VL53L1X_ADDR 0x29
#define VL53L1X_NO_TARGET 0xffff

enum vl53l1x_mode { VL53L1X_SHORT = 1, VL53L1X_LONG = 2 }; /* ~1.3 m / ~4 m, short copes better with sunlight */

struct vl53l1x {
    void *ctx;
    bool (*write)(void *ctx, uint16_t reg, const uint8_t *data, int n);
    bool (*read)(void *ctx, uint16_t reg, uint8_t *data, int n);

    enum vl53l1x_mode mode;
    uint16_t budget_ms, period_ms;
    uint8_t state, step, errors, ready_level;
    bool tried;
    uint32_t t;
    uint16_t starts; /* times ranging started; more than one means it dropped out */
};

/* Sets what to run; vl53l1x_run does the rest. period_ms must be >= budget_ms. */
void vl53l1x_begin(struct vl53l1x *d, enum vl53l1x_mode mode, uint16_t budget_ms, uint16_t period_ms);

/* Call every few ms, never blocks for more than one short I2C transfer. Finds the sensor
   (every 500 ms while there's none), loads ST's default config in pieces, runs the VHV
   calibration, starts ranging, and starts over if the sensor stops answering or stops
   producing ranges. 1 = new range in *mm and *status, 0 = none this time, -1 = not ranging.
   *mm is VL53L1X_NO_TARGET when nothing valid is in range; *status is ST's range status
   (0 valid, 1 sigma, 2 signal, 4 out of bounds, 7 wrap). */
int vl53l1x_run(struct vl53l1x *d, uint32_t now_ms, uint16_t *mm, uint8_t *status);
bool vl53l1x_ranging(const struct vl53l1x *d);

bool vl53l1x_set_mode(struct vl53l1x *d, enum vl53l1x_mode mode);
bool vl53l1x_set_budget(struct vl53l1x *d, uint16_t ms); /* 15 (short only), 20, 33, 50, 100, 200, 500 */
bool vl53l1x_set_period(struct vl53l1x *d, uint16_t ms);
bool vl53l1x_start(struct vl53l1x *d);
bool vl53l1x_stop(struct vl53l1x *d);

#endif
