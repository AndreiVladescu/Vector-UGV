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
    void (*delay_ms)(void *ctx, uint32_t ms);
    uint8_t ready_level;
};

/* Boot check, ST default config, VHV calibration, then continuous ranging.
   Blocks for about 100 ms. period_ms must be >= budget_ms. */
bool vl53l1x_init(struct vl53l1x *d, enum vl53l1x_mode mode, uint16_t budget_ms, uint16_t period_ms);
bool vl53l1x_set_mode(struct vl53l1x *d, enum vl53l1x_mode mode);
bool vl53l1x_set_budget(struct vl53l1x *d, uint16_t ms); /* 15 (short only), 20, 33, 50, 100, 200, 500 */
bool vl53l1x_set_period(struct vl53l1x *d, uint16_t ms);
bool vl53l1x_start(struct vl53l1x *d);
bool vl53l1x_stop(struct vl53l1x *d);

/* Non-blocking: 1 if a new range was read (and the interrupt cleared), 0 if none yet,
   -1 on a bus error. *mm is VL53L1X_NO_TARGET when nothing valid is in range;
   *status is ST's range status (0 valid, 1 sigma, 2 signal, 4 out of bounds, 7 wrap). */
int vl53l1x_poll(struct vl53l1x *d, uint16_t *mm, uint8_t *status);

#endif
