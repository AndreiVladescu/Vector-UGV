/* Setup shared by the leg application and the bootloader. */

#include "board.h"

#define CAN_BITRATE 1000000u

void board_fail(void)
{
    __disable_irq();
    for (;;) {
    }
}

uint32_t board_reset_cause(void)
{
    uint32_t f = RCC->CSR2, cause = 0;
    if (f & (RCC_CSR2_IWDGRSTF | RCC_CSR2_WWDGRSTF))
        cause |= RESET_WATCHDOG;
    if (f & RCC_CSR2_SFTRSTF)
        cause |= RESET_SOFTWARE;
    if (f & RCC_CSR2_PWRRSTF)
        cause |= RESET_POWER;
    if (f & (RCC_CSR2_LPWRRSTF | RCC_CSR2_OBLRSTF))
        cause |= RESET_OTHER;
    if (!cause && (f & RCC_CSR2_PINRSTF)) /* the pin flag comes with every reset, so only alone */
        cause = RESET_PIN;
    RCC->CSR2 |= RCC_CSR2_RMVF;
    return cause;
}

void board_clocks(void)
{
    RCC_OscInitTypeDef osc = {
        .OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE,
        .HSIState = RCC_HSI_ON,
        .HSIDiv = RCC_HSI_DIV1,
        .HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT,
        .HSEState = RCC_HSE_ON,
    };
    RCC_ClkInitTypeDef clk = {
        .ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1,
        .SYSCLKSource = RCC_SYSCLKSOURCE_HSI,
        .SYSCLKDivider = RCC_SYSCLK_DIV1,
        .AHBCLKDivider = RCC_HCLK_DIV1,
        .APB1CLKDivider = RCC_APB1_DIV1,
    };
    RCC_PeriphCLKInitTypeDef per = {
        .PeriphClockSelection = RCC_PERIPHCLK_FDCAN1,
        .Fdcan1ClockSelection = RCC_FDCAN1CLKSOURCE_HSE,
    };
    if (HAL_RCC_OscConfig(&osc) != HAL_OK || HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_1) != HAL_OK ||
        HAL_RCCEx_PeriphCLKConfig(&per) != HAL_OK)
        board_fail();
}

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

void board_can(FDCAN_HandleTypeDef *h, uint8_t node)
{
    __HAL_RCC_FDCAN1_CLK_ENABLE();
    /* smallest prescaler giving at most 25 time quanta per bit, sample point near 87.5 %:
       25 MHz -> 25 tq, 40 MHz -> 20 tq, 48 MHz (Nucleo) -> 24 tq */
    _Static_assert(HSE_VALUE % CAN_BITRATE == 0, "crystal must be a whole multiple of the bit rate");
    uint32_t prescaler = 1;
    while (HSE_VALUE / (CAN_BITRATE * prescaler) > 25 || HSE_VALUE % (CAN_BITRATE * prescaler))
        prescaler++;
    const uint32_t tq = HSE_VALUE / (CAN_BITRATE * prescaler);
    const uint32_t seg2 = (tq + 4) / 8 < 2 ? 2 : (tq + 4) / 8, seg1 = tq - 1 - seg2;

    h->Instance = FDCAN1;
    h->Init.ClockDivider = FDCAN_CLOCK_DIV1;
    h->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    h->Init.Mode = FDCAN_MODE_NORMAL;
    h->Init.AutoRetransmission = ENABLE;
    h->Init.TransmitPause = DISABLE;
    h->Init.ProtocolException = DISABLE;
    h->Init.NominalPrescaler = prescaler;
    h->Init.NominalSyncJumpWidth = seg2;
    h->Init.NominalTimeSeg1 = seg1;
    h->Init.NominalTimeSeg2 = seg2;
    h->Init.DataPrescaler = prescaler;
    h->Init.DataSyncJumpWidth = seg2;
    h->Init.DataTimeSeg1 = seg1;
    h->Init.DataTimeSeg2 = seg2;
    h->Init.StdFiltersNbr = 2;
    h->Init.ExtFiltersNbr = 0;
    h->Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    /* the other legs' commands arrive at 200 Hz each; the hardware drops them */
    FDCAN_FilterTypeDef sync = {.IdType = FDCAN_STANDARD_ID, .FilterIndex = 0, .FilterType = FDCAN_FILTER_MASK,
                                .FilterConfig = FDCAN_FILTER_TO_RXFIFO0, .FilterID1 = CAN_SYNC, .FilterID2 = 0x7FF};
    FDCAN_FilterTypeDef mine = sync;
    mine.FilterIndex = 1;
    mine.FilterID1 = node;
    mine.FilterID2 = 0x00F;
    if (HAL_FDCAN_Init(h) != HAL_OK || HAL_FDCAN_ConfigFilter(h, &sync) != HAL_OK ||
        HAL_FDCAN_ConfigFilter(h, &mine) != HAL_OK ||
        HAL_FDCAN_ConfigGlobalFilter(h, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK ||
        HAL_FDCAN_Start(h) != HAL_OK)
        board_fail();
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

uint8_t board_leg_id(void)
{
#if defined(BOARD_NUCLEO)
    return LEG_ID;
#else
    uint8_t s = board_id_straps(), pos = s & 3;
    return pos ? (uint8_t)(pos + ((s & 4) ? 3 : 0)) : 0; /* 0: the leg stays off with a config fault */
#endif
}

bool board_can_send(FDCAN_HandleTypeDef *h, const struct can_frame_t *f)
{
    FDCAN_TxHeaderTypeDef th = {
        .Identifier = f->id, .IdType = FDCAN_STANDARD_ID, .TxFrameType = FDCAN_DATA_FRAME,
        .DataLength = f->len, .ErrorStateIndicator = FDCAN_ESI_ACTIVE, .BitRateSwitch = FDCAN_BRS_OFF,
        .FDFormat = FDCAN_CLASSIC_CAN, .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
    };
    return HAL_FDCAN_GetTxFifoFreeLevel(h) > 0 && HAL_FDCAN_AddMessageToTxFifoQ(h, &th, f->data) == HAL_OK;
}

bool board_can_recv(FDCAN_HandleTypeDef *h, struct can_frame_t *f)
{
    FDCAN_RxHeaderTypeDef rh;
    while (HAL_FDCAN_GetRxFifoFillLevel(h, FDCAN_RX_FIFO0) > 0) {
        if (HAL_FDCAN_GetRxMessage(h, FDCAN_RX_FIFO0, &rh, f->data) != HAL_OK)
            return false;
        if (rh.IdType != FDCAN_STANDARD_ID || rh.RxFrameType != FDCAN_DATA_FRAME)
            continue;
        f->id = rh.Identifier;
        f->len = (uint8_t)(rh.DataLength > 8 ? 8 : rh.DataLength);
        return true;
    }
    return false;
}

bool board_can_recover(FDCAN_HandleTypeDef *h)
{
    /* Bus-off sets INIT and the controller stays silent. Clearing INIT starts the
       recovery: it rejoins after 129 x 11 recessive bits, about 1.4 ms at 1 Mbit/s. */
    if (!(h->Instance->PSR & FDCAN_PSR_BO) || !(h->Instance->CCCR & FDCAN_CCCR_INIT))
        return false;
    CLEAR_BIT(h->Instance->CCCR, FDCAN_CCCR_INIT);
    return true;
}
