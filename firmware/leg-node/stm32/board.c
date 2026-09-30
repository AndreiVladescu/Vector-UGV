/* Side board / Nucleo specifics: GPIO shared with the bootloader, and the node ID. */

#include "board.h"
#include "leg_id.h"

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
#else
    HAL_GPIO_WritePin(BUCK_EN_PORT, BUCK_EN_PIN, GPIO_PIN_RESET);
    g = (GPIO_InitTypeDef){.Pin = BUCK_EN_PIN, .Mode = GPIO_MODE_OUTPUT_PP};
    HAL_GPIO_Init(BUCK_EN_PORT, &g);
    g = (GPIO_InitTypeDef){.Pin = ID_POS0 | ID_POS1 | ID_SIDE, .Mode = GPIO_MODE_INPUT, .Pull = GPIO_PULLDOWN};
    HAL_GPIO_Init(ID_PORT, &g);
#endif
}

uint8_t board_id_straps(void)
{
#if defined(BOARD_NUCLEO)
    return 0xff;
#else
    uint32_t in = ID_PORT->IDR;
    return (uint8_t)(((in & ID_POS0) ? 1 : 0) | ((in & ID_POS1) ? 2 : 0) | ((in & ID_SIDE) ? 4 : 0));
#endif
}

uint8_t board_node_id(void)
{
#if defined(BOARD_NUCLEO)
    return LEG_ID;
#else
    return leg_node_from_straps(board_id_straps()); /* 0: the leg stays off with a config fault */
#endif
}
