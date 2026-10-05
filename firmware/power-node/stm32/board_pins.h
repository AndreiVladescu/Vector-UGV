#ifndef BOARD_PINS_H
#define BOARD_PINS_H

/* Pin map per board. BOARD_POWER is the power board (IC103, STM32C092KCT6, LQFP32),
   BOARD_NUCLEO a NUCLEO-C092RC with the BQ76942 and BQ25798 EVMs on its I2C for bring-up. */

#if defined(BOARD_NUCLEO)

#if defined(NUCLEO_CAN_PA11)
/* the boards' CAN pins, to the on-board transceiver through wires PA11-PD0 and PA12-PD1 on the
   morpho header; PD0/PD1 stay in their reset (analog) state */
#define CAN_PORT GPIOA
#define CAN_PINS (GPIO_PIN_11 | GPIO_PIN_12)
#else
#define CAN_PORT GPIOD
#define CAN_PINS (GPIO_PIN_0 | GPIO_PIN_1)
#endif
#define CAN_STBY_PORT GPIOD /* MCP2562FD standby, low = on */
#define CAN_STBY_PIN GPIO_PIN_2
#define LED_PORT GPIOA
#define LED_PIN GPIO_PIN_5
#define BMS_CRC 0 /* the EVM carries a plain BQ76942 */

#else /* BOARD_POWER */

#define CAN_PORT GPIOA
#define CAN_PINS (GPIO_PIN_11 | GPIO_PIN_12)
#define LED_PORT GPIOA
#define LED_PIN GPIO_PIN_15
#define BMS_CRC 1 /* BQ7694202: I2C CRC on from the factory */

#endif

/* BQ76942 and BQ25798 share I2C1 at 400 kHz (0x08 and 0x6B) */
#define I2C_PORT GPIOB
#define I2C_PINS (GPIO_PIN_6 | GPIO_PIN_7)

/* Outputs. CM5_OFF drives the 2N7002 on the carrier's 5V_EN (gate pull-down, so a reset or
   the bootloader leaves the CM5 on). ESTOP_N and SHUTDOWN_REQ_N are open drain, active low;
   ESTOP_N is read back on its own pin. */
#define OUT_CM5_OFF_PORT GPIOB
#define OUT_CM5_OFF_PIN GPIO_PIN_0
#define OUT_ESTOP_PORT GPIOA
#define OUT_ESTOP_PIN GPIO_PIN_8
#define OUT_SHUTDOWN_PORT GPIOA
#define OUT_SHUTDOWN_PIN GPIO_PIN_9

/* Inputs. BUTTON through a diode from the button (which pulls TS2 low), MCU pull-up;
   HALTED needs a pull-down on the board (the CM5 pin floats until Linux halts); 5V_PG is the
   carrier buck's open-drain PG. BMS_ALERT and CHG_INT are wired for later, the firmware polls. */
#define IN_BUTTON_PORT GPIOA
#define IN_BUTTON_PIN GPIO_PIN_0
#define IN_HALTED_PORT GPIOA
#define IN_HALTED_PIN GPIO_PIN_1
#define IN_PG_PORT GPIOA
#define IN_PG_PIN GPIO_PIN_6
#define IN_ALERT_PORT GPIOB
#define IN_ALERT_PIN GPIO_PIN_1
#define IN_CHG_INT_PORT GPIOB
#define IN_CHG_INT_PIN GPIO_PIN_2

#endif
