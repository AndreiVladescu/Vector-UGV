/* Leg node on the STM32C092: peripherals, the leg_hal implementation and the 1 ms loop. */

#include <string.h>

#include "board.h"
#include "boot.h"
#include "fw_version.h"
#include "leg.h"
#include "vl53l1x.h"

#define ADC_ROUNDS 16
#define ADC_CH (3 + ADC_EXTRA_CHANNELS)
#define I2C_TIMING_400K 0x50330309u /* RM table for a 48 MHz I2C clock */
#define TOF_POLL_MS 5
#define TOF_STALE_MS 200
#define TXQ_LEN 16

static ADC_HandleTypeDef hadc;
static DMA_HandleTypeDef hdma;
static TIM_HandleTypeDef htim;
static FDCAN_HandleTypeDef hcan;
static IWDG_HandleTypeDef hiwdg;
static I2C_HandleTypeDef hi2c;
static struct vl53l1x tof;
static uint16_t tof_last_mm;
static uint32_t tof_last_t;
static volatile uint16_t adc_buf[ADC_ROUNDS * ADC_CH] __attribute__((aligned(4)));
static struct leg leg;
static uint32_t can_tx_dropped, can_bus_off;
/* the controller holds 3 frames; a self-test or calibration result comes as a burst of up to 8 */
static struct can_frame_t txq[TXQ_LEN];
static uint8_t txq_head, txq_n;

void SysTick_Handler(void) { HAL_IncTick(); }

static void gpio(void)
{
    board_gpio();
    GPIO_InitTypeDef g;

    g = (GPIO_InitTypeDef){.Pin = DBG_PIN, .Mode = GPIO_MODE_OUTPUT_PP, .Speed = GPIO_SPEED_FREQ_HIGH};
    HAL_GPIO_Init(DBG_PORT, &g);

    g = (GPIO_InitTypeDef){.Pin = POT_PINS, .Mode = GPIO_MODE_ANALOG};
    HAL_GPIO_Init(POT_PORT, &g);
#if ADC_EXTRA_CHANNELS
    g.Pin = EXTRA_PINS;
    HAL_GPIO_Init(GPIOA, &g);
#endif

    g = (GPIO_InitTypeDef){.Pin = PWM_PINS, .Mode = GPIO_MODE_AF_PP, .Speed = GPIO_SPEED_FREQ_LOW,
                           .Alternate = GPIO_AF2_TIM1};
    HAL_GPIO_Init(PWM_PORT, &g);

    g = (GPIO_InitTypeDef){.Pin = TOF_I2C_PINS, .Mode = GPIO_MODE_AF_OD, .Pull = GPIO_PULLUP,
                           .Speed = GPIO_SPEED_FREQ_HIGH, .Alternate = GPIO_AF6_I2C1};
    HAL_GPIO_Init(TOF_PORT, &g);
    HAL_GPIO_WritePin(TOF_PORT, TOF_XSHUT_PIN, GPIO_PIN_RESET); /* sensor held in reset until tof_start() */
    g = (GPIO_InitTypeDef){.Pin = TOF_XSHUT_PIN, .Mode = GPIO_MODE_OUTPUT_PP};
    HAL_GPIO_Init(TOF_PORT, &g);

#if defined(BOARD_SIDE)
    g = (GPIO_InitTypeDef){.Pin = BUCK_PG_PIN, .Mode = GPIO_MODE_INPUT};
    HAL_GPIO_Init(BUCK_PG_PORT, &g);
    g = (GPIO_InitTypeDef){.Pin = ESTOP_PIN, .Mode = GPIO_MODE_INPUT, .Pull = GPIO_PULLUP};
    HAL_GPIO_Init(ESTOP_PORT, &g);
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
        mcu_fail();
    TIM_OC_InitTypeDef oc = {.OCMode = TIM_OCMODE_PWM1, .Pulse = 0, .OCPolarity = TIM_OCPOLARITY_HIGH};
    const uint32_t ch[3] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3};
    for (int i = 0; i < 3; i++)
        if (HAL_TIM_PWM_ConfigChannel(&htim, &oc, ch[i]) != HAL_OK || HAL_TIM_PWM_Start(&htim, ch[i]) != HAL_OK)
            mcu_fail();
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
        mcu_fail();
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
        mcu_fail();

    const uint32_t pots[] = POT_CHANNELS;
    ADC_ChannelConfTypeDef ch = {.Rank = ADC_RANK_CHANNEL_NUMBER, .SamplingTime = ADC_SAMPLINGTIME_COMMON_1};
    for (int i = 0; i < 3; i++) {
        ch.Channel = pots[i];
        if (HAL_ADC_ConfigChannel(&hadc, &ch) != HAL_OK)
            mcu_fail();
    }
#if ADC_EXTRA_CHANNELS
    const uint32_t extra[] = EXTRA_CHANNELS;
    for (int i = 0; i < ADC_EXTRA_CHANNELS; i++) {
        ch.Channel = extra[i];
        if (HAL_ADC_ConfigChannel(&hadc, &ch) != HAL_OK)
            mcu_fail();
    }
#endif
    if (HAL_ADCEx_Calibration_Start(&hadc) != HAL_OK ||
        HAL_ADC_Start_DMA(&hadc, (uint32_t *)(uintptr_t)adc_buf, ADC_ROUNDS * ADC_CH) != HAL_OK)
        mcu_fail();
}

static bool tof_write(void *ctx, uint16_t reg, const uint8_t *data, int n)
{
    (void)ctx;
    return HAL_I2C_Mem_Write(&hi2c, VL53L1X_ADDR << 1, reg, I2C_MEMADD_SIZE_16BIT, (uint8_t *)data, n, 3) == HAL_OK;
}

static bool tof_read(void *ctx, uint16_t reg, uint8_t *data, int n)
{
    (void)ctx;
    return HAL_I2C_Mem_Read(&hi2c, VL53L1X_ADDR << 1, reg, I2C_MEMADD_SIZE_16BIT, data, n, 3) == HAL_OK;
}

/* XSHUT low resets the sensor, which lets go of a bus it was holding; the I2C block
   gets a fresh start with it */
static void tof_shutdown(void *ctx, bool off)
{
    (void)ctx;
    HAL_GPIO_WritePin(TOF_PORT, TOF_XSHUT_PIN, off ? GPIO_PIN_RESET : GPIO_PIN_SET);
    if (off && (HAL_I2C_DeInit(&hi2c) != HAL_OK || HAL_I2C_Init(&hi2c) != HAL_OK))
        mcu_fail();
}

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
        mcu_fail();
    tof = (struct vl53l1x){.write = tof_write, .read = tof_read, .shutdown = tof_shutdown};
    vl53l1x_begin(&tof, VL53L1X_SHORT, 20, 25);
}

/* the driver finds the sensor, also one plugged in later, and never blocks for long */
static void tof_poll(uint32_t t)
{
    uint16_t mm;
    uint8_t status;
    if (t % TOF_POLL_MS == 0 && vl53l1x_run(&tof, t, &mm, &status) == 1) {
        tof_last_mm = mm;
        tof_last_t = t;
    }
}

static void watchdog(void)
{
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_32; /* ~1 kHz from LSI */
    hiwdg.Init.Reload = 100;                  /* 100 ms: a hung loop resets, the buck enable drops */
    hiwdg.Init.Window = IWDG_WINDOW_DISABLE;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
        mcu_fail();
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
    /* more than the buffer holds repeats it, which leaves the median where it was */
    for (int i = 0; i < n; i++)
        out[i] = (uint16_t)(adc_buf[(i % ADC_ROUNDS) * ADC_CH + slot] * 3300u / 4095u);
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

static uint8_t hal_leg_id(void *ctx) { (void)ctx; return board_node_id(); }

/* 0 = no sensor or no fresh reading, 0xffff = nothing in range */
static uint16_t hal_tof(void *ctx)
{
    (void)ctx;
    return vl53l1x_ranging(&tof) && HAL_GetTick() - tof_last_t < TOF_STALE_MS ? tof_last_mm : 0;
}

static void can_flush(void)
{
    while (txq_n && mcu_can_send(&hcan, &txq[txq_head])) {
        txq_head = (txq_head + 1) % TXQ_LEN;
        txq_n--;
    }
}

static void hal_send(void *ctx, const struct can_frame_t *f)
{
    (void)ctx;
    can_flush();
    if (txq_n == 0 && mcu_can_send(&hcan, f))
        return;
    if (txq_n == TXQ_LEN) {
        can_tx_dropped++;
        return;
    }
    txq[(txq_head + txq_n++) % TXQ_LEN] = *f;
}

static bool hal_save(void *ctx, int slot, const void *data, int len)
{
    (void)ctx;
    const uint32_t addr = CONFIG_ADDR + (slot ? FLASH_PAGE_SIZE : 0);
    uint64_t words[(sizeof(struct leg_config) + 7) / 8];
    if (len > (int)sizeof(words))
        return false;
    memset(words, 0xff, sizeof(words));
    memcpy(words, data, len);

    FLASH_EraseInitTypeDef erase = {.TypeErase = FLASH_TYPEERASE_PAGES,
                                    .Page = (addr - FLASH_BASE) / FLASH_PAGE_SIZE, .NbPages = 1};
    uint32_t bad;
    bool ok = HAL_FLASH_Unlock() == HAL_OK;
    HAL_IWDG_Refresh(&hiwdg);
    ok = ok && HAL_FLASHEx_Erase(&erase, &bad) == HAL_OK;
    for (unsigned i = 0; ok && i < sizeof(words) / 8; i++)
        ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + 8 * i, words[i]) == HAL_OK;
    HAL_FLASH_Lock();
    return ok && memcmp((const void *)addr, data, len) == 0;
}

static uint32_t hal_diag(void *ctx, uint8_t key)
{
    (void)ctx;
    switch (key) {
    case KEY_VERSION: return FW_VERSION;
    case KEY_RESET_CAUSE: return BOOT_RESET_CAUSE;
    case KEY_ID_STRAPS: return board_id_straps();
    case KEY_CAN_ERRORS: return (can_bus_off > 0xffff ? 0xffff : can_bus_off) << 16 | (can_tx_dropped > 0xffff ? 0xffff : can_tx_dropped);
    }
    return 0;
}

static const struct leg_hal hal = {
    NULL, hal_now, hal_pwm, hal_adc, hal_buck, hal_buck_good, hal_estop, hal_leg_id, hal_tof, hal_send, hal_save, hal_diag,
};

static void boot_send(void *ctx, const struct can_frame_t *f) { (void)ctx; mcu_can_send(&hcan, f); }

static void enter_bootloader(void)
{
    /* only while the servos are unpowered and nothing is moving */
    static const struct boot_io io = {.send = boot_send};
    if (leg.state != LEG_OFF && leg.state != LEG_FAULT) {
        boot_reply(&io, leg.node, BOOT_ENTER, BOOT_BUSY, 0);
        return;
    }
    hal_buck(NULL, false);
    BOOT_FLAG = BOOT_FLAG_ENTER;
    HAL_Delay(1); /* let the CAN controller finish sending */
    NVIC_SystemReset();
}

static void can_poll(void)
{
    struct can_frame_t f;
    while (mcu_can_recv(&hcan, &f)) {
        if (boot_is_enter(&f, leg.node))
            enter_bootloader();
        else
            leg_frame(&leg, &f);
    }
}

static void led(uint32_t t)
{
    /* on: active; fast blink: fault, wake or a bad leg ID; double blink: calibrating or
       self-test; off: the leg number every 3 s, short blinks left, long blinks right */
    bool on;
    switch (leg.state) {
    case LEG_ACTIVE: on = true; break;
    case LEG_FAULT:
    case LEG_WAKE: on = t % 200 < 100; break;
    case LEG_CALIBRATE:
    case LEG_TEST: on = t % 1000 < 100 || (t % 1000 >= 200 && t % 1000 < 300); break;
    default:
        if (leg.node < 1 || leg.node > 6) {
            on = t % 200 < 100;
        } else {
            uint32_t n = (leg.node - 1) % 3 + 1, slot = t % 3000 / 500, in_slot = t % 500;
            on = slot < n && in_slot < (leg.node > 3 ? 350u : 100u);
        }
    }
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

int main(void)
{
    HAL_Init();
    mcu_clocks();
    gpio();
    pwm_timer();
    adc();
    mcu_can(&hcan, board_node_id());
    tof_start();
    HAL_Delay(2); /* the DMA fills the ADC buffer once */

    leg_init(&leg, &hal, (const struct leg_config *)CONFIG_ADDR,
             (const struct leg_config *)(CONFIG_ADDR + FLASH_PAGE_SIZE));
    watchdog();

    uint32_t last = HAL_GetTick();
    for (;;) {
        if (mcu_can_recover(&hcan))
            can_bus_off++;
        can_flush();
        can_poll();
        uint32_t t = HAL_GetTick();
        if (t != last) {
            last = t;
            DBG_PORT->BSRR = DBG_PIN;
            tof_poll(t);
            leg_tick(&leg);
            led(t);
            HAL_IWDG_Refresh(&hiwdg);
            DBG_PORT->BRR = DBG_PIN;
        }
    }
}
