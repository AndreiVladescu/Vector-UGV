#ifndef BOARD_H
#define BOARD_H

#include "board_pins.h"
#include "mcu.h"

void board_gpio(void); /* CAN pins, LED; the power outputs in their "leave it on" state */
uint8_t board_node_id(void);

#endif
