/* Leg node on the STM32C092: peripherals, the leg_hal implementation and the 1 ms loop. */

#include <string.h>

#include "stm32c0xx_hal.h"

#include "board_pins.h"
#include "leg.h"
#include "vl53l1x.h"

#define ADC_ROUNDS 16
#define ADC_CH (3 + ADC_EXTRA_CHANNELS)
#define CONFIG_ADDR (FLASH_BASE + FLASH_SIZE - FLASH_PAGE_SIZE)
#define CAN_BITRATE 1000000u
#define I2C_TIMING_400K 0x50330309u /* RM table for a 48 MHz I2C clock */
#define TOF_POLL_MS 5
#define TOF_STALE_MS 200

static ADC_HandleTypeDef hadc;
static DMA_HandleTypeDef hdma;
static TIM_HandleTypeDef htim;
static FDCAN_HandleTypeDef hcan;
static IWDG_HandleTypeDef hiwdg;
static I2C_HandleTypeDef hi2c;
static struct vl53l1x tof;
static bool tof_ok;
static uint16_t tof_last_mm;
static uint32_t tof_last_t, tof_errors;
static volatile uint16_t adc_buf[ADC_ROUNDS * ADC_CH];
static struct leg leg;
static uint32_t can_tx_dropped;

void SysTick_Handler(void) { HAL_IncTick(); }

static void fail(void)
{
    __disable_irq();
    for (;;) {
    }
}

static void clocks(void)
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
        fail();
}

static void gpio(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};

    g = (GPIO_InitTypeDef){.Pin = POT_PINS, .Mode = GPIO_MODE_ANALOG};
    HAL_GPIO_Init(POT_PORT, &g);
#if ADC_EXTRA_CHANNELS
    g.Pin = EXTRA_PINS;
    HAL_GPIO_Init(GPIOA, &g);
#endif

    g = (GPIO_InitTypeDef){.Pin = PWM_PINS, .Mode = GPIO_MODE_AF_PP, .Speed = GPIO_SPEED_FREQ_LOW,
                           .Alternate = GPIO_AF2_TIM1};
    HAL_GPIO_Init(PWM_PORT, &g);

    g = (GPIO_InitTypeDef){.Pin = CAN_PINS, .Mode = GPIO_MODE_AF_PP, .Speed = GPIO_SPEED_FREQ_HIGH,
                           .Alternate = GPIO_AF4_FDCAN1};
    HAL_GPIO_Init(CAN_PORT, &g);

    g = (GPIO_InitTypeDef){.Pin = LED_PIN, .Mode = GPIO_MODE_OUTPUT_PP};
    HAL_GPIO_Init(LED_PORT, &g);

    g = (GPIO_InitTypeDef){.Pin = TOF_I2C_PINS, .Mode = GPIO_MODE_AF_OD, .Pull = GPIO_PULLUP,
                           .Speed = GPIO_SPEED_FREQ_HIGH, .Alternate = GPIO_AF6_I2C1};
    HAL_GPIO_Init(TOF_PORT, &g);
    HAL_GPIO_WritePin(TOF_PORT, TOF_XSHUT_PIN, GPIO_PIN_RESET); /* sensor held in reset until tof() */
    g = (GPIO_InitTypeDef){.Pin = TOF_XSHUT_PIN, .Mode = GPIO_MODE_OUTPUT_PP};
    HAL_GPIO_Init(TOF_PORT, &g);

#if defined(BOARD_NUCLEO)
    HAL_GPIO_WritePin(CAN_STBY_PORT, CAN_STBY_PIN, GPIO_PIN_RESET);
    g = (GPIO_InitTypeDef){.Pin = CAN_STBY_PIN, .Mode = GPIO_MODE_OUTPUT_PP};
    HAL_GPIO_Init(CAN_STBY_PORT, &g);
#else
    HAL_GPIO_WritePin(BUCK_EN_PORT, BUCK_EN_PIN, GPIO_PIN_RESET);
    g = (GPIO_InitTypeDef){.Pin = BUCK_EN_PIN, .Mode = GPIO_MODE_OUTPUT_PP};
    HAL_GPIO_Init(BUCK_EN_PORT, &g);
    g = (GPIO_InitTypeDef){.Pin = BUCK_PG_PIN, .Mode = GPIO_MODE_INPUT};
    HAL_GPIO_Init(BUCK_PG_PORT, &g);
    g = (GPIO_InitTypeDef){.Pin = ESTOP_PIN | ID_B0 | ID_B1 | ID_B2, .Mode = GPIO_MODE_INPUT, .Pull = GPIO_PULLUP};
    HAL_GPIO_Init(GPIOB, &g);
#endif
}

static void pwm_timer(void)
{
    __HAL_RCC_TIM1_CLK_ENABLE();
    htim.Instance = TIM1;
    htim.Init.Prescaler = 47; /* 1 MHz: compare values are microseconds */
    htim.Init.Period = 19999; /* 50 Hz */
    htim.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim) != HAL_OK)
        fail();
    TIM_OC_InitTypeDef oc = {.OCMode = TIM_OCMODE_PWM1, .Pulse = 0, .OCPolarity = TIM_OCPOLARITY_HIGH};
    const uint32_t ch[3] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3};
    for (int i = 0; i < 3; i++)
        if (HAL_TIM_PWM_ConfigChannel(&htim, &oc, ch[i]) != HAL_OK || HAL_TIM_PWM_Start(&htim, ch[i]) != HAL_OK)
            fail();
}

static void adc(void)
{
    __HAL_RCC_ADC_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    hdma.Instance = DMA1_Channel1;
    hdma.Init.Request = DMA_REQUEST_ADC1;
    hdma.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma.Init.MemInc = DMA_MINC_ENABLE;
    hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma.Init.Mode = DMA_CIRCULAR;
    hdma.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma) != HAL_OK)
        fail();
    __HAL_LINKDMA(&hadc, DMA_Handle, hdma);

    /* 24 MHz ADC clock, 160.5 + 12.5 cycles per sample: one pass over all channels ~50 us,
       so the 16 samples the median sees span most of a millisecond */
    hadc.Instance = ADC1;
    hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
    hadc.Init.Resolution = ADC_RESOLUTION_12B;
    hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc.Init.ScanConvMode = ADC_SCAN_SEQ_FIXED;
    hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    hadc.Init.ContinuousConvMode = ENABLE;
    hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc.Init.DMAContinuousRequests = ENABLE;
    hadc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    hadc.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_160CYCLES_5;
    if (HAL_ADC_Init(&hadc) != HAL_OK)
        fail();

    const uint32_t pots[] = POT_CHANNELS;
    ADC_ChannelConfTypeDef ch = {.Rank = ADC_RANK_CHANNEL_NUMBER, .SamplingTime = ADC_SAMPLINGTIME_COMMON_1};
    for (int i = 0; i < 3; i++) {
        ch.Channel = pots[i];
        if (HAL_ADC_ConfigChannel(&hadc, &ch) != HAL_OK)
            fail();
    }
#if ADC_EXTRA_CHANNELS
    const uint32_t extra[] = EXTRA_CHANNELS;
    for (int i = 0; i < ADC_EXTRA_CHANNELS; i++) {
        ch.Channel = extra[i];
        if (HAL_ADC_ConfigChannel(&hadc, &ch) != HAL_OK)
            fail();
    }
#endif
    if (HAL_ADCEx_Calibration_Start(&hadc) != HAL_OK ||
        HAL_ADC_Start_DMA(&hadc, (uint32_t *)adc_buf, ADC_ROUNDS * ADC_CH) != HAL_OK)
        fail();
}

static void can(void)
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

    hcan.Instance = FDCAN1;
    hcan.Init.ClockDivider = FDCAN_CLOCK_DIV1;
    hcan.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    hcan.Init.Mode = FDCAN_MODE_NORMAL;
    hcan.Init.AutoRetransmission = ENABLE;
    hcan.Init.TransmitPause = DISABLE;
    hcan.Init.ProtocolException = DISABLE;
    hcan.Init.NominalPrescaler = prescaler;
    hcan.Init.NominalSyncJumpWidth = seg2;
    hcan.Init.NominalTimeSeg1 = seg1;
    hcan.Init.NominalTimeSeg2 = seg2;
    hcan.Init.DataPrescaler = prescaler;
    hcan.Init.DataSyncJumpWidth = seg2;
    hcan.Init.DataTimeSeg1 = seg1;
    hcan.Init.DataTimeSeg2 = seg2;
    hcan.Init.StdFiltersNbr = 0;
    hcan.Init.ExtFiltersNbr = 0;
    hcan.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    if (HAL_FDCAN_Init(&hcan) != HAL_OK ||
        HAL_FDCAN_ConfigGlobalFilter(&hcan, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT, FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK ||
        HAL_FDCAN_Start(&hcan) != HAL_OK)
        fail();
}

static bool tof_write(void *ctx, uint16_t reg, const uint8_t *data, int n)
{
    (void)ctx;
    return HAL_I2C_Mem_Write(&hi2c, VL53L1X_ADDR << 1, reg, I2C_MEMADD_SIZE_16BIT, (uint8_t *)data, n, 10) == HAL_OK;
}

static bool tof_read(void *ctx, uint16_t reg, uint8_t *data, int n)
{
    (void)ctx;
    return HAL_I2C_Mem_Read(&hi2c, VL53L1X_ADDR << 1, reg, I2C_MEMADD_SIZE_16BIT, data, n, 10) == HAL_OK;
}

static void tof_delay(void *ctx, uint32_t ms) { (void)ctx; HAL_Delay(ms); }

/* Runs before the watchdog starts: init blocks for ~100 ms. A sensor plugged in later
   is only picked up after a reset. */
static void tof_start(void)
{
    __HAL_RCC_I2C1_CLK_ENABLE();
    hi2c.Instance = I2C1;
    hi2c.Init.Timing = I2C_TIMING_400K;
    hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c) != HAL_OK)
        fail();

    HAL_GPIO_WritePin(TOF_PORT, TOF_XSHUT_PIN, GPIO_PIN_SET);
    HAL_Delay(2);
    tof = (struct vl53l1x){.write = tof_write, .read = tof_read, .delay_ms = tof_delay};
    tof_ok = vl53l1x_init(&tof, VL53L1X_SHORT, 20, 25);
}

static void tof_poll(uint32_t t)
{
    uint16_t mm;
    uint8_t status;
    if (!tof_ok || t % TOF_POLL_MS)
        return;
    int r = vl53l1x_poll(&tof, &mm, &status);
    if (r > 0) {
        tof_last_mm = mm;
        tof_last_t = t;
    } else if (r < 0) {
        tof_errors++;
    }
}

static void watchdog(void)
{
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_32; /* ~1 kHz from LSI */
    hiwdg.Init.Reload = 100;                  /* 100 ms: a hung loop resets, the buck enable drops */
    hiwdg.Init.Window = IWDG_WINDOW_DISABLE;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
        fail();
}

/* ---- leg_hal ---- */

static uint32_t hal_now(void *ctx) { (void)ctx; return HAL_GetTick(); }

static void hal_pwm(void *ctx, int joint, int us)
{
    (void)ctx;
    static const uint32_t ch[3] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3};
    __HAL_TIM_SET_COMPARE(&htim, ch[joint], us);
}

static void hal_adc(void *ctx, int ch, uint16_t *out, int n)
{
    (void)ctx;
    int slot = ch <= ADC_POT2 ? ch : 3 + (ch - ADC_I_LEG);
    if (slot >= ADC_CH) {
        /* not wired on this board: fixed, harmless values */
        uint16_t v = ch == ADC_NTC ? 1650 : 0;
        for (int i = 0; i < n; i++)
            out[i] = v;
        return;
    }
    if (n > ADC_ROUNDS)
        n = ADC_ROUNDS;
    for (int i = 0; i < n; i++)
        out[i] = (uint16_t)(adc_buf[i * ADC_CH + slot] * 3300u / 4095u);
}

static void hal_buck(void *ctx, bool on)
{
    (void)ctx;
#if defined(BOARD_NUCLEO)
    (void)on;
#else
    HAL_GPIO_WritePin(BUCK_EN_PORT, BUCK_EN_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
}

static bool hal_buck_good(void *ctx)
{
    (void)ctx;
#if defined(BOARD_NUCLEO)
    return true;
#else
    return HAL_GPIO_ReadPin(BUCK_PG_PORT, BUCK_PG_PIN) == GPIO_PIN_SET;
#endif
}

static bool hal_estop(void *ctx)
{
    (void)ctx;
#if defined(BOARD_NUCLEO)
    return false;
#else
    return HAL_GPIO_ReadPin(ESTOP_PORT, ESTOP_PIN) == GPIO_PIN_RESET;
#endif
}

static uint8_t hal_leg_id(void *ctx)
{
    (void)ctx;
#if defined(BOARD_NUCLEO)
    return LEG_ID;
#else
    uint32_t in = ~ID_PORT->IDR; /* a fitted jumper pulls the pin low */
    return (uint8_t)(((in & ID_B0) ? 1 : 0) | ((in & ID_B1) ? 2 : 0) | ((in & ID_B2) ? 4 : 0));
#endif
}

/* 0 = no sensor or no fresh reading, 0xffff = nothing in range */
static uint16_t hal_tof(void *ctx)
{
    (void)ctx;
    return tof_ok && HAL_GetTick() - tof_last_t < TOF_STALE_MS ? tof_last_mm : 0;
}

static void hal_send(void *ctx, const struct can_frame_t *f)
{
    (void)ctx;
    FDCAN_TxHeaderTypeDef h = {
        .Identifier = f->id, .IdType = FDCAN_STANDARD_ID, .TxFrameType = FDCAN_DATA_FRAME,
        .DataLength = f->len, .ErrorStateIndicator = FDCAN_ESI_ACTIVE, .BitRateSwitch = FDCAN_BRS_OFF,
        .FDFormat = FDCAN_CLASSIC_CAN, .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
    };
    if (HAL_FDCAN_GetTxFifoFreeLevel(&hcan) == 0 || HAL_FDCAN_AddMessageToTxFifoQ(&hcan, &h, f->data) != HAL_OK)
        can_tx_dropped++;
}

static bool hal_save(void *ctx, const void *data, int len)
{
    (void)ctx;
    uint64_t words[(sizeof(struct leg_config) + 7) / 8];
    if (len > (int)sizeof(words))
        return false;
    memset(words, 0xff, sizeof(words));
    memcpy(words, data, len);

    FLASH_EraseInitTypeDef erase = {.TypeErase = FLASH_TYPEERASE_PAGES,
                                    .Page = (CONFIG_ADDR - FLASH_BASE) / FLASH_PAGE_SIZE, .NbPages = 1};
    uint32_t bad;
    bool ok = HAL_FLASH_Unlock() == HAL_OK;
    HAL_IWDG_Refresh(&hiwdg);
    ok = ok && HAL_FLASHEx_Erase(&erase, &bad) == HAL_OK;
    for (unsigned i = 0; ok && i < sizeof(words) / 8; i++)
        ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, CONFIG_ADDR + 8 * i, words[i]) == HAL_OK;
    HAL_FLASH_Lock();
    return ok && memcmp((const void *)CONFIG_ADDR, data, len) == 0;
}

static const struct leg_hal hal = {
    NULL, hal_now, hal_pwm, hal_adc, hal_buck, hal_buck_good, hal_estop, hal_leg_id, hal_tof, hal_send, hal_save,
};

static void can_poll(void)
{
    FDCAN_RxHeaderTypeDef h;
    struct can_frame_t f;
    while (HAL_FDCAN_GetRxFifoFillLevel(&hcan, FDCAN_RX_FIFO0) > 0) {
        if (HAL_FDCAN_GetRxMessage(&hcan, FDCAN_RX_FIFO0, &h, f.data) != HAL_OK)
            break;
        if (h.IdType != FDCAN_STANDARD_ID || h.RxFrameType != FDCAN_DATA_FRAME)
            continue;
        f.id = h.Identifier;
        f.len = (uint8_t)(h.DataLength > 8 ? 8 : h.DataLength);
        leg_frame(&leg, &f);
    }
}

static void led(uint32_t t)
{
    /* on: active, slow blink: off, fast blink: fault or wake, double blink: calibrating */
    bool on;
    switch (leg.state) {
    case LEG_ACTIVE: on = true; break;
    case LEG_FAULT:
    case LEG_WAKE: on = t % 200 < 100; break;
    case LEG_CALIBRATE: on = t % 1000 < 100 || (t % 1000 >= 200 && t % 1000 < 300); break;
    default: on = t % 2000 < 100;
    }
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

int main(void)
{
    HAL_Init();
    clocks();
    gpio();
    pwm_timer();
    adc();
    can();
    tof_start();

    leg_init(&leg, &hal, (const struct leg_config *)CONFIG_ADDR);
    watchdog();

    uint32_t last = HAL_GetTick();
    for (;;) {
        can_poll();
        uint32_t t = HAL_GetTick();
        if (t != last) {
            last = t;
            tof_poll(t);
            leg_tick(&leg);
            led(t);
            HAL_IWDG_Refresh(&hiwdg);
        }
    }
}
