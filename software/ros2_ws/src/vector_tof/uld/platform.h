/* Linux i2c-dev platform layer for ST's VL53L8CX ULD, in place of ST's empty platform.h. */
#ifndef _PLATFORM_H_
#define _PLATFORM_H_

#include <stdint.h>
#include <string.h>

typedef struct {
    uint16_t address; /* 8-bit, 0x52 by default, as the ULD expects */
    int fd;           /* open /dev/i2c-N */
} VL53L8CX_Platform;

#define VL53L8CX_NB_TARGET_PER_ZONE 1U

uint8_t VL53L8CX_RdByte(VL53L8CX_Platform *p, uint16_t reg, uint8_t *value);
uint8_t VL53L8CX_WrByte(VL53L8CX_Platform *p, uint16_t reg, uint8_t value);
uint8_t VL53L8CX_RdMulti(VL53L8CX_Platform *p, uint16_t reg, uint8_t *values, uint32_t size);
uint8_t VL53L8CX_WrMulti(VL53L8CX_Platform *p, uint16_t reg, uint8_t *values, uint32_t size);
void VL53L8CX_SwapBuffer(uint8_t *buffer, uint16_t size);
uint8_t VL53L8CX_WaitMs(VL53L8CX_Platform *p, uint32_t ms);

#endif
