/* Power node on the STM32C092: peripherals, the power_hal implementation and the 1 ms loop. */

#include <string.h>

#include "board.h"
#include "boot.h"
#include "fw_version.h"
#include "power.h"

#define I2C_TIMING_400K 0x50330309u /* RM table for a 48 MHz I2C clock */
#define I2C_TIMEOUT_MS 5
#define TXQ_LEN 16
#define CELL_MASK 0x0207 /* cells on VC1, VC2, VC3 and VC10 */

static FDCAN_HandleTypeDef hcan;
static IWDG_HandleTypeDef hiwdg;
static I2C_HandleTypeDef hi2c;
static struct power power;
static uint32_t can_tx_dropped, can_bus_off;
static struct can_frame_t txq[TXQ_LEN];
static uint8_t txq_head, txq_n;

void SysTick_Handler(void) { HAL_IncTick(); }

static void gpio(void)
{
    board_gpio();
    GPIO_InitTypeDef g;

    g = (GPIO_InitTypeDef){.Pin = I2C_PINS, .Mode = GPIO_MODE_AF_OD, .Pull = GPIO_NOPULL,
                           .Speed = GPIO_SPEED_FREQ_HIGH, .Alternate = GPIO_AF6_I2C1};
    HAL_GPIO_Init(I2C_PORT, &g);

    g = (GPIO_InitTypeDef){.Pin = IN_BUTTON_PIN, .Mode = GPIO_MODE_INPUT, .Pull = GPIO_PULLUP};
    HAL_GPIO_Init(IN_BUTTON_PORT, &g);
    g = (GPIO_InitTypeDef){.Pin = IN_HALTED_PIN, .Mode = GPIO_MODE_INPUT, .Pull = GPIO_PULLDOWN};
    HAL_GPIO_Init(IN_HALTED_PORT, &g);
    g = (GPIO_InitTypeDef){.Pin = IN_PG_PIN, .Mode = GPIO_MODE_INPUT, .Pull = GPIO_PULLUP};
    HAL_GPIO_Init(IN_PG_PORT, &g);
    g = (GPIO_InitTypeDef){.Pin = IN_ALERT_PIN | IN_CHG_INT_PIN, .Mode = GPIO_MODE_INPUT, .Pull = GPIO_PULLUP};
    HAL_GPIO_Init(GPIOB, &g);
}

static void i2c(void)
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
}

static void watchdog(void)
{
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_64; /* ~500 Hz from LSI */
    hiwdg.Init.Reload = 1000;                 /* 2 s: the BMS setup blocks for a few hundred ms */
    hiwdg.Init.Window = IWDG_WINDOW_DISABLE;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
        mcu_fail();
}

/* ---- power_hal ---- */

static uint32_t hal_now(void *ctx) { (void)ctx; return HAL_GetTick(); }

static void hal_delay(void *ctx, uint32_t ms)
{
    (void)ctx;
    HAL_Delay(ms);
    HAL_IWDG_Refresh(&hiwdg);
}

/* a failed transfer can leave the peripheral busy; start it over */
static bool i2c_done(HAL_StatusTypeDef st)
{
    if (st == HAL_OK)
        return true;
    if (HAL_I2C_DeInit(&hi2c) != HAL_OK || HAL_I2C_Init(&hi2c) != HAL_OK)
        mcu_fail();
    return false;
}

static bool hal_i2c_write(void *ctx, uint8_t addr, const uint8_t *data, int n)
{
    (void)ctx;
    return i2c_done(HAL_I2C_Master_Transmit(&hi2c, addr << 1, (uint8_t *)data, n, I2C_TIMEOUT_MS));
}

static bool hal_i2c_read(void *ctx, uint8_t addr, uint8_t reg, uint8_t *data, int n)
{
    (void)ctx;
    return i2c_done(HAL_I2C_Mem_Read(&hi2c, addr << 1, reg, I2C_MEMADD_SIZE_8BIT, data, n, I2C_TIMEOUT_MS));
}

static void hal_out(void *ctx, enum pwr_out pin, bool on)
{
    (void)ctx;
    switch (pin) {
    case OUT_CM5_OFF: HAL_GPIO_WritePin(OUT_CM5_OFF_PORT, OUT_CM5_OFF_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET); break;
    case OUT_ESTOP: HAL_GPIO_WritePin(OUT_ESTOP_PORT, OUT_ESTOP_PIN, on ? GPIO_PIN_RESET : GPIO_PIN_SET); break;
    case OUT_SHUTDOWN_REQ: HAL_GPIO_WritePin(OUT_SHUTDOWN_PORT, OUT_SHUTDOWN_PIN, on ? GPIO_PIN_RESET : GPIO_PIN_SET); break;
    case OUT_LED: HAL_GPIO_WritePin(LED_PORT, LED_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET); break;
    default: break;
    }
}

static bool hal_in(void *ctx, enum pwr_in pin)
{
    (void)ctx;
    switch (pin) {
    case IN_BUTTON: return HAL_GPIO_ReadPin(IN_BUTTON_PORT, IN_BUTTON_PIN) == GPIO_PIN_RESET;
    case IN_HALTED: return HAL_GPIO_ReadPin(IN_HALTED_PORT, IN_HALTED_PIN) == GPIO_PIN_SET;
    case IN_ESTOP: return HAL_GPIO_ReadPin(OUT_ESTOP_PORT, OUT_ESTOP_PIN) == GPIO_PIN_RESET; /* open drain reads the line */
    case IN_5V_PG: return HAL_GPIO_ReadPin(IN_PG_PORT, IN_PG_PIN) == GPIO_PIN_SET;
    default: return false;
    }
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
    uint64_t words[(sizeof(struct power_config) + 7) / 8];
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
    case KEY_CAN_ERRORS: return (can_bus_off > 0xffff ? 0xffff : can_bus_off) << 16 | (can_tx_dropped > 0xffff ? 0xffff : can_tx_dropped);
    }
    return 0;
}

static const struct power_hal hal = {
    NULL, hal_now, hal_delay, hal_i2c_write, hal_i2c_read, hal_out, hal_in, hal_send, hal_save, hal_diag,
};

static void boot_send(void *ctx, const struct can_frame_t *f) { (void)ctx; mcu_can_send(&hcan, f); }

/* The outputs hold their "on" state through the reset (gate pull-downs), so an update
   doesn't cut the CM5 or the legs; not while a shutdown is under way. */
static void enter_bootloader(void)
{
    static const struct boot_io io = {.send = boot_send};
    if (power.state == POWER_HALTING || power.state == POWER_OFF) {
        boot_reply(&io, POWER_NODE, BOOT_ENTER, BOOT_BUSY, 0);
        return;
    }
    BOOT_FLAG = BOOT_FLAG_ENTER;
    HAL_Delay(1);
    NVIC_SystemReset();
}

static void can_poll(void)
{
    struct can_frame_t f;
    while (mcu_can_recv(&hcan, &f)) {
        if (boot_is_enter(&f, POWER_NODE))
            enter_bootloader();
        else
            power_frame(&power, &f);
    }
}

int main(void)
{
    HAL_Init();
    mcu_clocks();
    gpio();
    mcu_can(&hcan, POWER_NODE);
    i2c();
    watchdog();

    power_init(&power, &hal, CELL_MASK, BMS_CRC, (const struct power_config *)CONFIG_ADDR,
               (const struct power_config *)(CONFIG_ADDR + FLASH_PAGE_SIZE));

    uint32_t last = HAL_GetTick();
    for (;;) {
        if (mcu_can_recover(&hcan))
            can_bus_off++;
        can_flush();
        can_poll();
        uint32_t t = HAL_GetTick();
        if (t != last) {
            last = t;
            power_tick(&power);
            HAL_IWDG_Refresh(&hiwdg);
        }
    }
}
