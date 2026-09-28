#ifndef MCU_H
#define MCU_H

#include <stdbool.h>

#include "stm32c0xx_hal.h"

#include "vector_can.h"

/* Flash: 16 KB bootloader, the application, the last two pages for the node config (two
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

void mcu_fail(void);
/* RESET_* bits from the RCC flags, which it then clears */
uint32_t mcu_reset_cause(void);
void mcu_clocks(void);
/* only SYNC and frames to this node get into the receive FIFO */
void mcu_can(FDCAN_HandleTypeDef *h, uint8_t node);
bool mcu_can_send(FDCAN_HandleTypeDef *h, const struct can_frame_t *f);
bool mcu_can_recv(FDCAN_HandleTypeDef *h, struct can_frame_t *f);
/* restart the controller after bus-off; true when it had to */
bool mcu_can_recover(FDCAN_HandleTypeDef *h);

#endif
