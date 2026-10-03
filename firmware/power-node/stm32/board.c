/* Power board / Nucleo specifics: GPIO shared with the bootloader, and the node ID. */

#include "board.h"

void board_gpio(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitTypeDef g;

    g = (GPIO_InitTypeDef){.Pin = CAN_PINS, .Mode = GPIO_MODE_AF_PP, .Speed = GPIO_SPEED_FREQ_HIGH,
                           .Alternate = GPIO_AF4_FDCAN1};
    HAL_GPIO_Init(CAN_PORT, &g);

    g = (GPIO_InitTypeDef){.Pin = LED_PIN, .Mode = GPIO_MODE_OUTPUT_PP};
    HAL_GPIO_Init(LED_PORT, &g);

#if defined(BOARD_NUCLEO)
    HAL_GPIO_WritePin(CAN_STBY_PORT, CAN_STBY_PIN, GPIO_PIN_RESET);
    g = (GPIO_InitTypeDef){.Pin = CAN_STBY_PIN, .Mode = GPIO_MODE_OUTPUT_PP};
    HAL_GPIO_Init(CAN_STBY_PORT, &g);
#endif

    /* N-FET gate low: the CM5 stays on */
    HAL_GPIO_WritePin(OUT_CM5_OFF_PORT, OUT_CM5_OFF_PIN, GPIO_PIN_RESET);
    g = (GPIO_InitTypeDef){.Pin = OUT_CM5_OFF_PIN, .Mode = GPIO_MODE_OUTPUT_PP};
    HAL_GPIO_Init(OUT_CM5_OFF_PORT, &g);
    /* open drain released: no e-stop, no shutdown request */
    HAL_GPIO_WritePin(GPIOA, OUT_ESTOP_PIN | OUT_SHUTDOWN_PIN, GPIO_PIN_SET);
    g = (GPIO_InitTypeDef){.Pin = OUT_ESTOP_PIN | OUT_SHUTDOWN_PIN, .Mode = GPIO_MODE_OUTPUT_OD};
    HAL_GPIO_Init(GPIOA, &g);
}

uint8_t board_node_id(void) { return POWER_NODE; }
