#include "platform.h"

#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <time.h>

/* Registers auto-increment, so long transfers (the 32 KB firmware pages, the 1.4 KB
   results) go out as chunks at increasing addresses; smaller than any I2C driver's limit. */
#define CHUNK 1024

static uint8_t xfer(VL53L8CX_Platform *p, uint16_t reg, uint8_t *data, uint32_t n, int read)
{
    uint8_t buf[2 + CHUNK];
    for (uint32_t off = 0; off < n; off += CHUNK) {
        uint16_t len = n - off > CHUNK ? CHUNK : (uint16_t)(n - off);
        uint16_t r = (uint16_t)(reg + off);
        buf[0] = r >> 8;
        buf[1] = r & 0xff;
        struct i2c_msg msgs[2] = {
            {.addr = p->address >> 1, .flags = 0, .len = 2, .buf = buf},
            {.addr = p->address >> 1, .flags = I2C_M_RD, .len = len, .buf = data + off},
        };
        if (!read) {
            memcpy(buf + 2, data + off, len);
            msgs[0].len = 2 + len;
        }
        struct i2c_rdwr_ioctl_data io = {.msgs = msgs, .nmsgs = read ? 2 : 1};
        if (ioctl(p->fd, I2C_RDWR, &io) < 0)
            return 1;
    }
    return 0;
}

uint8_t VL53L8CX_RdByte(VL53L8CX_Platform *p, uint16_t reg, uint8_t *value) { return xfer(p, reg, value, 1, 1); }
uint8_t VL53L8CX_WrByte(VL53L8CX_Platform *p, uint16_t reg, uint8_t value) { return xfer(p, reg, &value, 1, 0); }
uint8_t VL53L8CX_RdMulti(VL53L8CX_Platform *p, uint16_t reg, uint8_t *v, uint32_t n) { return xfer(p, reg, v, n, 1); }
uint8_t VL53L8CX_WrMulti(VL53L8CX_Platform *p, uint16_t reg, uint8_t *v, uint32_t n) { return xfer(p, reg, v, n, 0); }

/* the sensor sends 32-bit words big-endian */
void VL53L8CX_SwapBuffer(uint8_t *buffer, uint16_t size)
{
    for (uint32_t i = 0; i + 3 < size; i += 4) {
        uint32_t w = (uint32_t)buffer[i] << 24 | (uint32_t)buffer[i + 1] << 16 | (uint32_t)buffer[i + 2] << 8 | buffer[i + 3];
        memcpy(&buffer[i], &w, 4);
    }
}

uint8_t VL53L8CX_WaitMs(VL53L8CX_Platform *p, uint32_t ms)
{
    (void)p;
    struct timespec ts = {.tv_sec = ms / 1000, .tv_nsec = (long)(ms % 1000) * 1000000L};
    nanosleep(&ts, NULL);
    return 0;
}
