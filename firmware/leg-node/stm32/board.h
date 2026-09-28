#ifndef BOARD_H
#define BOARD_H

#include <stdbool.h>

#include "stm32c0xx_hal.h"

#include "board_pins.h"
#include "vector_can.h"

/* Flash: 16 KB bootloader, the application, the last two pages for the leg config (two
   slots, so a save cut short by a power loss keeps the older copy).
   A word at the top of RAM (outside both linker maps) survives a reset and tells the
   bootloader what to do next. */
#define BOOT_ADDR 0x08000000u
#define APP_ADDR 0x08004000u
#define CONFIG_ADDR 0x0803F000u /* slot 0; slot 1 one page up */
#define APP_SIZE (CONFIG_ADDR - APP_ADDR)
#define BOOT_FLAG (*(volatile uint32_t *)0x200077F0u)
#define BOOT_FLAG_ENTER 0xB0071E47u /* stay in the bootloader */
#define BOOT_FLAG_RUN 0xB0070A99u  /* jump to the application straight away */
#define BOOT_RESET_CAUSE (*(volatile uint32_t *)0x200077F4u) /* RESET_* bits, set by the bootloader */

void board_fail(void);
/* RESET_* bits from the RCC flags, which it then clears */
uint32_t board_reset_cause(void);
void board_clocks(void);
void board_gpio(void); /* CAN pins, LED, leg ID; buck enable driven low */
/* only SYNC and frames to this node get into the receive FIFO */
void board_can(FDCAN_HandleTypeDef *h, uint8_t node);
uint8_t board_leg_id(void);
/* raw jumpers: side << 2 | position, 0xff on the Nucleo */
uint8_t board_id_straps(void);
bool board_can_send(FDCAN_HandleTypeDef *h, const struct can_frame_t *f);
bool board_can_recv(FDCAN_HandleTypeDef *h, struct can_frame_t *f);
/* restart the controller after bus-off; true when it had to */
bool board_can_recover(FDCAN_HandleTypeDef *h);

#endif
