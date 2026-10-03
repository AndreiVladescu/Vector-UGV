#ifndef HAL_H
#define HAL_H

#include <stdbool.h>
#include <stdint.h>

#include "vector_can.h"

enum pwr_out {
    OUT_CM5_OFF,      /* pulls the carrier's 5V_EN low: CM5 off (pulled on, so an MCU reset doesn't drop it) */
    OUT_ESTOP,        /* pulls ESTOP_N low: every leg cell's servo buck is held off */
    OUT_SHUTDOWN_REQ, /* J-SYSCTL SHUTDOWN_REQ_N low: the CM5 halts */
    OUT_LED,
    OUTPUTS
};

enum pwr_in {
    IN_BUTTON, /* power button held */
    IN_HALTED, /* CM5 gpio-poweroff: halted */
    IN_ESTOP,  /* ESTOP_N read back low */
    IN_5V_PG,  /* the carrier's 5 V buck PG */
    INPUTS
};

/* Everything the power logic needs from the board. I2C transfers are raw bytes (the
   BQ76942 driver adds its CRC); a read writes reg and then reads n bytes after a
   repeated start. */
struct power_hal {
    void *ctx;
    uint32_t (*now_ms)(void *ctx);
    void (*delay_ms)(void *ctx, uint32_t ms);
    bool (*i2c_write)(void *ctx, uint8_t addr, const uint8_t *data, int n);
    bool (*i2c_read)(void *ctx, uint8_t addr, uint8_t reg, uint8_t *data, int n);
    void (*out)(void *ctx, enum pwr_out pin, bool on);
    bool (*in)(void *ctx, enum pwr_in pin);
    void (*send)(void *ctx, const struct can_frame_t *f);
    bool (*save)(void *ctx, int slot, const void *data, int len); /* slot 0 or 1 */
    uint32_t (*diag)(void *ctx, uint8_t key); /* KEY_VERSION, KEY_RESET_CAUSE, KEY_CAN_ERRORS */
};

#endif
