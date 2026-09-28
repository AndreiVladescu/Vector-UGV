#ifndef FLASH_SIM_H
#define FLASH_SIM_H

#include "boot.h"

/* Application flash of an STM32C092 in RAM: 2 KB pages, doubleword programming only into
   erased cells. Stands behind the bootloader protocol in the tests and in sim_legs. */
#define FLASH_SIM_ADDR 0x08004000u
#define FLASH_SIM_SIZE (236u * 1024)

struct flash_sim {
    uint8_t mem[FLASH_SIM_SIZE];
    struct boot_io io;
    void (*tx)(void *user, const struct can_frame_t *f);
    void *user;
    int erases, programs;
};

void flash_sim_init(struct flash_sim *s);

#endif
