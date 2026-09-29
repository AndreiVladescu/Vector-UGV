#ifndef BOARD_PINS_H
#define BOARD_PINS_H

/* Pin map per board. BOARD_POWER is the power board (STM32C092KCT6, LQFP32; the board isn't
   drawn yet, so this is the plan for the schematic), BOARD_NUCLEO a NUCLEO-C092RC with the
   BQ76942 and BQ25798 EVMs on its I2C for bring-up. */

#if defined(BOARD_NUCLEO)

#define CAN_PORT GPIOD
#define CAN_PINS (GPIO_PIN_0 | GPIO_PIN_1)
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

/* Outputs. 5V_OFF and the side cuts drive small N-FETs (gate pull-downs, so a reset or the
   bootloader leaves everything on): 5V_OFF pulls the buck's EN low, SIDE_x_OFF the LM5069's
   UVLO. RUN and SHUTDOWN_REQ_N are open drain, active low. */
#define OUT_5V_OFF_PORT GPIOB
#define OUT_5V_OFF_PIN GPIO_PIN_0
#define OUT_SIDE_L_PORT GPIOB
#define OUT_SIDE_L_PIN GPIO_PIN_1
#define OUT_SIDE_R_PORT GPIOB
#define OUT_SIDE_R_PIN GPIO_PIN_2
#define OUT_RUN_PORT GPIOA
#define OUT_RUN_PIN GPIO_PIN_8
#define OUT_SHUTDOWN_PORT GPIOA
#define OUT_SHUTDOWN_PIN GPIO_PIN_9

/* Inputs. BUTTON through a diode from the button (which pulls TS2 low), MCU pull-up;
   HALTED needs a pull-down on the board (the CM5 pin floats until Linux halts);
   RUN_SENSE low = e-stop; 5V_PG is the LM61460's open-drain PG. */
#define IN_BUTTON_PORT GPIOA
#define IN_BUTTON_PIN GPIO_PIN_0
#define IN_HALTED_PORT GPIOA
#define IN_HALTED_PIN GPIO_PIN_1
#define IN_RUN_PORT GPIOA
#define IN_RUN_PIN GPIO_PIN_4
#define IN_PG_PORT GPIOA
#define IN_PG_PIN GPIO_PIN_6

#endif
