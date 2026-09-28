#ifndef BOOT_H
#define BOOT_H

#include <stdbool.h>
#include <stdint.h>

#include "vector_can.h"

/* CAN bootloader protocol. Host -> node on CAN_BOOT | node, replies on CAN_BOOT_REPLY | node,
   byte 0 = op (reply: op | 0x80), reply byte 1 = status. See protocol/README.md. */

#define BOOT_VERSION 1
#define BOOT_ENTER_MAGIC 0x746f6f62u /* "boot" */

enum boot_op { BOOT_ENTER = 1, BOOT_INFO, BOOT_ERASE, BOOT_DATA, BOOT_DONE, BOOT_RUN };
enum boot_status { BOOT_OK, BOOT_BAD_OP, BOOT_BAD_SEQ, BOOT_FLASH_ERR, BOOT_BAD_CRC, BOOT_TOO_BIG, BOOT_NO_IMAGE, BOOT_BUSY };

struct boot_io {
    void *ctx;
    const uint8_t *image;       /* the application area, readable */
    uint32_t size;              /* its size, a whole number of pages */
    uint32_t page;
    uint32_t sram_lo, sram_hi;  /* valid initial stack pointer range */
    uint32_t image_addr;        /* where the image sits in the address map */
    bool (*erase)(void *ctx, uint32_t offset, uint32_t len);
    bool (*program)(void *ctx, uint32_t offset, const uint8_t data[8]);
    void (*send)(void *ctx, const struct can_frame_t *f);
};

struct boot {
    const struct boot_io *io;
    uint8_t node;
    uint32_t erased, written;
    uint8_t seq;
    uint8_t head[8], acc[8];
};

void boot_init(struct boot *b, const struct boot_io *io, uint8_t node);
/* true when the host asked to run a valid image */
bool boot_frame(struct boot *b, const struct can_frame_t *f);
void boot_reply(const struct boot_io *io, uint8_t node, uint8_t op, uint8_t status, uint32_t arg);
bool boot_image_valid(const struct boot_io *io);
bool boot_is_enter(const struct can_frame_t *f, uint8_t node);
uint32_t boot_crc32(uint32_t crc, const uint8_t *p, uint32_t len);

#endif
