#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "boot.h"
#include "flash_sim.h"

static int failures, checks;

#define CHECK(cond) do { \
    checks++; \
    if (!(cond)) { failures++; printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } \
} while (0)

static struct flash_sim flash;
static struct boot b;
static struct can_frame_t last;
static int replies;

static void capture(void *user, const struct can_frame_t *f) { (void)user; last = *f; replies++; }

static uint32_t arg(void) { return last.data[2] | last.data[3] << 8 | last.data[4] << 16 | (uint32_t)last.data[5] << 24; }

static bool send(uint8_t op, const uint8_t *payload, int n)
{
    struct can_frame_t f = {.id = CAN_BOOT | 3, .len = (uint8_t)(1 + n), .data = {op}};
    memcpy(&f.data[1], payload, n);
    return boot_frame(&b, &f);
}

static uint8_t send32(uint8_t op, uint32_t v)
{
    uint8_t p[4] = {v, v >> 8, v >> 16, v >> 24};
    send(op, p, 4);
    return last.data[1];
}

static void make_image(uint8_t *img, int len)
{
    for (int i = 0; i < len; i++)
        img[i] = (uint8_t)(i * 7 + 3);
    const uint32_t sp = 0x200077F0u, reset = FLASH_SIM_ADDR + 0x101;
    memcpy(img, &sp, 4);
    memcpy(img + 4, &reset, 4);
}

/* host side, as leg_config.py does it: pad to 8, 6 bytes per frame */
static int upload(const uint8_t *img, int len, bool corrupt)
{
    int padded = (len + 7) & ~7;
    uint8_t *p = malloc(padded);
    memset(p, 0xff, padded);
    memcpy(p, img, len);
    if (send32(BOOT_ERASE, padded) != BOOT_OK)
        return -1;
    uint8_t seq = 0;
    for (int off = 0; off < padded; off += 6, seq++) {
        uint8_t d[7] = {seq};
        int n = padded - off < 6 ? padded - off : 6;
        memcpy(&d[1], &p[off], n);
        if (corrupt && off == 600)
            d[1] ^= 1;
        send(BOOT_DATA, d, 1 + n);
        if (last.data[1] != BOOT_OK)
            return -2;
    }
    uint32_t crc = boot_crc32(0, p, padded);
    free(p);
    return send32(BOOT_DONE, crc);
}

static void reset(void)
{
    flash_sim_init(&flash);
    flash.tx = capture;
    boot_init(&b, &flash.io, 3);
}

int main(void)
{
    static uint8_t img[33241];
    make_image(img, sizeof(img));

    printf("empty\n");
    reset();
    CHECK(!boot_image_valid(&flash.io));
    CHECK(!send(BOOT_RUN, NULL, 0) && last.data[1] == BOOT_NO_IMAGE);
    send(BOOT_INFO, NULL, 0);
    CHECK(last.id == (CAN_BOOT_REPLY | 3) && last.data[0] == (BOOT_INFO | 0x80) && arg() == (BOOT_VERSION | (FLASH_SIM_SIZE / 1024) << 16));

    printf("other node ignored\n");
    struct can_frame_t other = {.id = CAN_BOOT | 4, .len = 1, .data = {BOOT_INFO}};
    replies = 0;
    boot_frame(&b, &other);
    CHECK(replies == 0);

    printf("enter\n");
    struct can_frame_t enter = {.id = CAN_BOOT | 3, .len = 5, .data = {BOOT_ENTER, 'b', 'o', 'o', 't'}};
    CHECK(boot_is_enter(&enter, 3) && !boot_is_enter(&enter, 2));
    enter.data[4] = 'x';
    CHECK(!boot_is_enter(&enter, 3));

    printf("upload\n");
    CHECK(upload(img, sizeof(img), false) == BOOT_OK);
    CHECK(arg() == 33248);
    CHECK(memcmp(flash.mem, img, sizeof(img)) == 0);
    CHECK(flash.mem[33241] == 0xff);
    CHECK(flash.erases == 17);
    CHECK(boot_image_valid(&flash.io));
    CHECK(send(BOOT_RUN, NULL, 0) && last.data[1] == BOOT_OK);
    send(BOOT_INFO, NULL, 0);
    CHECK(arg() & 0x100);

    printf("bad crc leaves no runnable image\n");
    CHECK(upload(img, sizeof(img), true) == BOOT_BAD_CRC);
    CHECK(!boot_image_valid(&flash.io));
    CHECK(flash.mem[0] == 0xff);
    CHECK(upload(img, sizeof(img), false) == BOOT_OK);

    printf("interrupted upload\n");
    send32(BOOT_ERASE, 4096);
    uint8_t d[7] = {0, 1, 2, 3, 4, 5, 6};
    send(BOOT_DATA, d, 7);
    CHECK(!boot_image_valid(&flash.io));

    printf("resent frame, skipped frame\n");
    send(BOOT_DATA, (uint8_t[]){1, 1, 2, 3, 4, 5, 6}, 7);
    CHECK(last.data[1] == BOOT_OK && last.data[2] == 2);
    send(BOOT_DATA, (uint8_t[]){1, 9, 9, 9, 9, 9, 9}, 7); /* ack lost, host sends it again */
    CHECK(last.data[1] == BOOT_OK && last.data[2] == 2 && b.written == 12 && b.acc[3] == 6);
    send(BOOT_DATA, (uint8_t[]){5, 1, 2, 3, 4, 5, 6}, 7);
    CHECK(last.data[1] == BOOT_BAD_SEQ);

    printf("too big\n");
    CHECK(send32(BOOT_ERASE, FLASH_SIM_SIZE + 1) == BOOT_TOO_BIG);
    send32(BOOT_ERASE, 2048);
    int st = BOOT_OK;
    for (int i = 0; i < 342 && st == BOOT_OK; i++) {
        send(BOOT_DATA, (uint8_t[]){(uint8_t)i, 0, 0, 0, 0, 0, 0}, 7);
        st = last.data[1];
    }
    CHECK(st == BOOT_TOO_BIG);

    printf("%d checks, %d failed\n", checks, failures);
    return failures != 0;
}
