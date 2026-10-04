/* IO MCU pins. Unused pins (PA5, PA8, PB6, PB7) stay analog. */

#include "board.h"

void board_gpio(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef g;

    g = (GPIO_InitTypeDef){.Pin = HOST_PINS, .Mode = GPIO_MODE_AF_PP, .Pull = GPIO_PULLUP,
                           .Speed = GPIO_SPEED_FREQ_HIGH, .Alternate = HOST_AF};
    HAL_GPIO_Init(HOST_PORT, &g);
    g = (GPIO_InitTypeDef){.Pin = ELRS_PINS, .Mode = GPIO_MODE_AF_PP, .Pull = GPIO_PULLUP,
                           .Speed = GPIO_SPEED_FREQ_HIGH, .Alternate = ELRS_AF};
    HAL_GPIO_Init(ELRS_PORT, &g);
    g = (GPIO_InitTypeDef){.Pin = LIDAR_PIN, .Mode = GPIO_MODE_AF_PP, .Pull = GPIO_PULLUP, .Alternate = LIDAR_AF};
    HAL_GPIO_Init(LIDAR_PORT, &g);
    g = (GPIO_InitTypeDef){.Pin = GNSS_TX_PIN, .Mode = GPIO_MODE_AF_PP, .Alternate = GNSS_TX_AF};
    HAL_GPIO_Init(GNSS_PORT, &g);
    g = (GPIO_InitTypeDef){.Pin = GNSS_RX_PIN, .Mode = GPIO_MODE_AF_PP, .Pull = GPIO_PULLUP, .Alternate = GNSS_RX_AF};
    HAL_GPIO_Init(GNSS_PORT, &g);
    g = (GPIO_InitTypeDef){.Pin = GNSS_PPS_PIN, .Mode = GPIO_MODE_INPUT, .Pull = GPIO_PULLDOWN};
    HAL_GPIO_Init(GNSS_PPS_PORT, &g);

    g = (GPIO_InitTypeDef){.Pin = LIDAR_PWM_PIN, .Mode = GPIO_MODE_AF_PP, .Alternate = LIDAR_PWM_AF};
    HAL_GPIO_Init(LIDAR_PWM_PORT, &g);
    g = (GPIO_InitTypeDef){.Pin = BUZZER_PIN, .Mode = GPIO_MODE_AF_PP, .Alternate = BUZZER_AF};
    HAL_GPIO_Init(BUZZER_PORT, &g);

    g = (GPIO_InitTypeDef){.Pin = LORA_SPI_PINS, .Mode = GPIO_MODE_AF_PP, .Speed = GPIO_SPEED_FREQ_HIGH,
                           .Alternate = GPIO_AF0_SPI1};
    HAL_GPIO_Init(LORA_SPI_PORT, &g);
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET);
    g = (GPIO_InitTypeDef){.Pin = LORA_NSS_PIN, .Mode = GPIO_MODE_OUTPUT_PP, .Speed = GPIO_SPEED_FREQ_HIGH};
    HAL_GPIO_Init(LORA_NSS_PORT, &g);
    HAL_GPIO_WritePin(LORA_NRST_PORT, LORA_NRST_PIN, GPIO_PIN_RESET); /* held in reset until main lets go */
    g = (GPIO_InitTypeDef){.Pin = LORA_NRST_PIN, .Mode = GPIO_MODE_OUTPUT_OD};
    HAL_GPIO_Init(LORA_NRST_PORT, &g);
    g = (GPIO_InitTypeDef){.Pin = LORA_DIO0_PIN, .Mode = GPIO_MODE_INPUT, .Pull = GPIO_PULLDOWN};
    HAL_GPIO_Init(LORA_DIO0_PORT, &g);
    g = (GPIO_InitTypeDef){.Pin = LORA_DIO1_PIN, .Mode = GPIO_MODE_INPUT, .Pull = GPIO_PULLDOWN};
    HAL_GPIO_Init(LORA_DIO1_PORT, &g);

    g = (GPIO_InitTypeDef){.Pin = ADC_PINS, .Mode = GPIO_MODE_ANALOG};
    HAL_GPIO_Init(ADC_PORT, &g);

    HAL_GPIO_WritePin(LTE_EN_PORT, LTE_EN_PIN, GPIO_PIN_RESET);
    g = (GPIO_InitTypeDef){.Pin = LTE_EN_PIN, .Mode = GPIO_MODE_OUTPUT_PP};
    HAL_GPIO_Init(LTE_EN_PORT, &g);
    g = (GPIO_InitTypeDef){.Pin = LTE_STATUS_PIN, .Mode = GPIO_MODE_INPUT, .Pull = GPIO_PULLDOWN};
    HAL_GPIO_Init(LTE_STATUS_PORT, &g);

    g = (GPIO_InitTypeDef){.Pin = LED_PIN, .Mode = GPIO_MODE_OUTPUT_PP};
    HAL_GPIO_Init(LED_PORT, &g);
}
