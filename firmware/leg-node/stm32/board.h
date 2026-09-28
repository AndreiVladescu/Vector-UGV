#ifndef BOARD_H
#define BOARD_H

#include "board_pins.h"
#include "mcu.h"

void board_gpio(void); /* CAN pins, LED, leg ID; buck enable driven low */
uint8_t board_node_id(void);
/* raw jumpers: side << 2 | position, 0xff on the Nucleo */
uint8_t board_id_straps(void);

#endif
