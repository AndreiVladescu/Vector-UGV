#ifndef BOARD_PINS_H
#define BOARD_PINS_H

/* Pin map per board. BOARD_SIDE is the side board (LQFP32, see leg-node.ioc),
   BOARD_NUCLEO the NUCLEO-C092RC for bring-up. */

#if defined(BOARD_NUCLEO)

/* ST-LINK VCP owns PA2/PA3 and the green LED sits on PA5, so the tibia pot moves to PA4.
   Current, NTC and supply sense aren't wired on the Nucleo. */
#define POT_PORT GPIOA
#define POT_PINS (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4)
#define POT_CHANNELS {ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_4}
#define ADC_EXTRA_CHANNELS 0

#define CAN_PORT GPIOD
#define CAN_PINS (GPIO_PIN_0 | GPIO_PIN_1)
#define CAN_STBY_PORT GPIOD /* MCP2562FD standby, low = on */
#define CAN_STBY_PIN GPIO_PIN_2

#define LED_PORT GPIOA
#define LED_PIN GPIO_PIN_5

#ifndef LEG_ID
#define LEG_ID 1
#endif

#else /* BOARD_SIDE */

#define POT_PORT GPIOA
#define POT_PINS (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2)
#define POT_CHANNELS {ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2}
/* PA3 I_LEG, PA4 NTC, PA5 VBAT, PA6 6V0 */
#define ADC_EXTRA_CHANNELS 4
#define EXTRA_PINS (GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6)
#define EXTRA_CHANNELS {ADC_CHANNEL_3, ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_6}

#define CAN_PORT GPIOA
#define CAN_PINS (GPIO_PIN_11 | GPIO_PIN_12)

#define LED_PORT GPIOA
#define LED_PIN GPIO_PIN_15

#define BUCK_EN_PORT GPIOB
#define BUCK_EN_PIN GPIO_PIN_0
#define BUCK_PG_PORT GPIOB
#define BUCK_PG_PIN GPIO_PIN_1
#define ESTOP_PORT GPIOB
#define ESTOP_PIN GPIO_PIN_2
#define ID_PORT GPIOB /* jumpers to GND, read with pull-ups */
#define ID_B0 GPIO_PIN_3
#define ID_B1 GPIO_PIN_8
#define ID_B2 GPIO_PIN_9

#endif

/* same on both boards */
#define PWM_PORT GPIOA
#define PWM_PINS (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10)

/* VL53L1X on I2C1; on the Nucleo these are on the morpho header */
#define TOF_PORT GPIOB
#define TOF_I2C_PINS (GPIO_PIN_6 | GPIO_PIN_7)
#define TOF_XSHUT_PIN GPIO_PIN_5

#endif
