/* IO MCU on the STM32C092: four UARTs on interrupts and ring buffers, SPI1 to the RFM95W,
   ADC, the two PWM timers, and the io_hal for the logic in src/. */

#include <string.h>

#include "board.h"
#include "fw_version.h"
#include "io.h"

#define PCLK_HZ 48000000u
#define BOOT_FLAG_ROM 0xB0075707u /* set before a reset: start ST's ROM bootloader */
#define SYSMEM_ADDR 0x1FFF0000u

/* ---- UARTs ---- */

struct uart {
    USART_TypeDef *u;
    uint8_t *rx, *tx;
    uint16_t rx_size, tx_size; /* powers of two */
    volatile uint16_t rx_head, rx_tail, tx_head, tx_tail;
    volatile uint32_t errors;
};

static uint8_t host_rx[512], host_tx[4096], elrs_rx[256], elrs_tx[128], lidar_rx[512], gnss_rx[512];
static struct uart uarts[PORTS] = {
    [PORT_HOST] = {USART1, host_rx, host_tx, sizeof(host_rx), sizeof(host_tx)},
    [PORT_ELRS] = {USART2, elrs_rx, elrs_tx, sizeof(elrs_rx), sizeof(elrs_tx)},
    [PORT_LIDAR] = {USART4, lidar_rx, NULL, sizeof(lidar_rx), 0},
    [PORT_GNSS] = {USART3, gnss_rx, NULL, sizeof(gnss_rx), 0},
};

static void uart_init(struct uart *p, uint32_t baud, bool tx)
{
    USART_TypeDef *u = p->u;
    u->CR1 = 0;
    u->BRR = (PCLK_HZ + baud / 2) / baud;
    u->CR3 = 0; /* overruns are flagged and counted */
    u->CR1 = USART_CR1_RE | (tx ? USART_CR1_TE : 0) | USART_CR1_RXNEIE_RXFNEIE | USART_CR1_UE;
}

static void uart_irq(struct uart *p)
{
    USART_TypeDef *u = p->u;
    const uint32_t isr = u->ISR;
    if (isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE)) {
        u->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF;
        p->errors++;
    }
    if (isr & USART_ISR_RXNE_RXFNE) {
        const uint8_t b = (uint8_t)u->RDR;
        const uint16_t next = (p->rx_head + 1) & (p->rx_size - 1);
        if (next != p->rx_tail) {
            p->rx[p->rx_head] = b;
            p->rx_head = next;
        } else {
            p->errors++;
        }
    }
    if ((u->CR1 & USART_CR1_TXEIE_TXFNFIE) && (isr & USART_ISR_TXE_TXFNF)) {
        if (p->tx_tail != p->tx_head) {
            u->TDR = p->tx[p->tx_tail];
            p->tx_tail = (p->tx_tail + 1) & (p->tx_size - 1);
        } else {
            u->CR1 &= ~USART_CR1_TXEIE_TXFNFIE;
        }
    }
}

void USART1_IRQHandler(void) { uart_irq(&uarts[PORT_HOST]); }
void USART2_IRQHandler(void) { uart_irq(&uarts[PORT_ELRS]); }
void USART3_4_IRQHandler(void)
{
    uart_irq(&uarts[PORT_GNSS]);
    uart_irq(&uarts[PORT_LIDAR]);
}

static bool uart_write(struct uart *p, const uint8_t *d, int n)
{
    if (!p->tx_size)
        return false;
    const uint16_t used = (p->tx_head - p->tx_tail) & (p->tx_size - 1);
    if (n > p->tx_size - 1 - used)
        return false;
    uint16_t h = p->tx_head;
    for (int i = 0; i < n; i++) {
        p->tx[h] = d[i];
        h = (h + 1) & (p->tx_size - 1);
    }
    p->tx_head = h;
    p->u->CR1 |= USART_CR1_TXEIE_TXFNFIE; /* a race with the ISR clearing it only costs one empty interrupt */
    return true;
}

static int uart_read(struct uart *p, uint8_t *d, int max)
{
    int n = 0;
    while (n < max && p->rx_tail != p->rx_head) {
        d[n++] = p->rx[p->rx_tail];
        p->rx_tail = (p->rx_tail + 1) & (p->rx_size - 1);
    }
    return n;
}

static void uarts_init(void)
{
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_USART4_CLK_ENABLE();
    uart_init(&uarts[PORT_HOST], 1000000, true);
    uart_init(&uarts[PORT_ELRS], 420000, true);
    uart_init(&uarts[PORT_LIDAR], 115200, false); /* LDS01RR, XV-11 protocol */
    uart_init(&uarts[PORT_GNSS], 9600, true); /* the MAX-M10S default */
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0); /* 420 kbaud, the shortest byte time */
    HAL_NVIC_SetPriority(USART3_4_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    HAL_NVIC_EnableIRQ(USART3_4_IRQn);
}

/* ---- SPI1 to the RFM95W, polled, 6 MHz ---- */

static uint8_t spi_byte(uint8_t b)
{
    while (!(SPI1->SR & SPI_SR_TXE)) {
    }
    *(volatile uint8_t *)&SPI1->DR = b;
    while (!(SPI1->SR & SPI_SR_RXNE)) {
    }
    return *(volatile uint8_t *)&SPI1->DR;
}

static void spi_init(void)
{
    __HAL_RCC_SPI1_CLK_ENABLE();
    SPI1->CR1 = 0;
    SPI1->CR2 = SPI_CR2_FRXTH | (7u << SPI_CR2_DS_Pos); /* 8-bit, RXNE at one byte */
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | (2u << SPI_CR1_BR_Pos) | SPI_CR1_SPE; /* /8 */
}

static void radio_xfer(uint8_t addr, uint8_t *data, const uint8_t *out, int n)
{
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_RESET);
    spi_byte(addr);
    for (int i = 0; i < n; i++) {
        const uint8_t r = spi_byte(out ? out[i] : 0);
        if (data)
            data[i] = r;
    }
    while (SPI1->SR & SPI_SR_BSY) {
    }
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET);
}

static void radio_read(void *ctx, uint8_t reg, uint8_t *data, int n) { (void)ctx; radio_xfer(reg & 0x7F, data, NULL, n); }
static void radio_write(void *ctx, uint8_t reg, const uint8_t *data, int n) { (void)ctx; radio_xfer(reg | 0x80, NULL, data, n); }
static const struct sx1276_io radio_io = {.read = radio_read, .write = radio_write};

/* ---- ADC, VDDA from VREFINT ---- */

static void adc_init(void)
{
    __HAL_RCC_ADC_CLK_ENABLE();
    ADC1->CFGR2 = 2u << ADC_CFGR2_CKMODE_Pos; /* PCLK / 4 = 12 MHz */
    ADC1->SMPR = 7u << ADC_SMPR_SMP1_Pos;      /* 160.5 cycles: the 100k divider needs time */
    ADC1_COMMON->CCR |= ADC_CCR_VREFEN;
    ADC1->CR = ADC_CR_ADVREGEN;
    HAL_Delay(1);
    ADC1->CR |= ADC_CR_ADCAL;
    while (ADC1->CR & ADC_CR_ADCAL) {
    }
    ADC1->ISR = ADC_ISR_ADRDY;
    ADC1->CR |= ADC_CR_ADEN;
    while (!(ADC1->ISR & ADC_ISR_ADRDY)) {
    }
}

static uint32_t adc_raw(uint32_t ch)
{
    ADC1->CHSELR = 1u << ch;
    while (!(ADC1->ISR & ADC_ISR_CCRDY)) {
    }
    ADC1->ISR = ADC_ISR_CCRDY;
    uint32_t sum = 0;
    for (int i = 0; i < 8; i++) {
        ADC1->CR |= ADC_CR_ADSTART;
        while (!(ADC1->ISR & ADC_ISR_EOC)) {
        }
        sum += ADC1->DR;
    }
    return sum / 8;
}

/* ---- timers: TIM3_CH4 lidar motor PWM at 30 kHz, TIM14_CH1 buzzer from a 1 MHz count ---- */

#define LIDAR_PWM_ARR 1599u

static void timers_init(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM14_CLK_ENABLE();
    TIM3->PSC = 0;
    TIM3->ARR = LIDAR_PWM_ARR;
    TIM3->CCR4 = 0;
    TIM3->CCMR2 = (6u << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE; /* PWM mode 1 */
    TIM3->CCER = TIM_CCER_CC4E;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;

    TIM14->PSC = PCLK_HZ / 1000000u - 1;
    TIM14->ARR = 999;
    TIM14->CCR1 = 0;
    TIM14->CCMR1 = (6u << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
    TIM14->CCER = TIM_CCER_CC1E;
    TIM14->EGR = TIM_EGR_UG;
    TIM14->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
}

/* ---- io_hal ---- */

static IWDG_HandleTypeDef hiwdg;
static uint32_t vdda_mv = 3300;

static uint32_t hal_now(void *ctx) { (void)ctx; return HAL_GetTick(); }

static bool hal_write(void *ctx, enum io_port port, const uint8_t *d, int n)
{
    (void)ctx;
    return uart_write(&uarts[port], d, n);
}

static uint32_t hal_adc(void *ctx, enum io_adc ch)
{
    (void)ctx;
    static const uint8_t chan[ADCS] = {[ADC_VBAT] = ADC_CH_VBAT, [ADC_5V] = ADC_CH_5V, [ADC_NTC] = ADC_CH_NTC};
    const uint32_t ref = adc_raw(ADC_CH_VREFINT);
    if (ref)
        vdda_mv = (uint32_t)*VREFINT_CAL_ADDR * VREFINT_CAL_VREF / ref;
    return adc_raw(chan[ch]) * vdda_mv / 4095;
}

static void hal_out(void *ctx, enum io_out pin, bool on)
{
    (void)ctx;
    const GPIO_PinState s = on ? GPIO_PIN_SET : GPIO_PIN_RESET;
    switch (pin) {
    case OUT_LTE_EN: HAL_GPIO_WritePin(LTE_EN_PORT, LTE_EN_PIN, s); break;
    case OUT_LED: HAL_GPIO_WritePin(LED_PORT, LED_PIN, s); break;
    default: break;
    }
}

static bool hal_in(void *ctx, enum io_in pin)
{
    (void)ctx;
    return pin == IN_LTE_STATUS && HAL_GPIO_ReadPin(LTE_STATUS_PORT, LTE_STATUS_PIN) == GPIO_PIN_SET;
}

static void hal_lidar_pwm(void *ctx, uint16_t permille)
{
    (void)ctx;
    TIM3->CCR4 = (LIDAR_PWM_ARR + 1) * permille / 1000;
}

static void hal_buzzer(void *ctx, uint16_t hz)
{
    (void)ctx;
    if (hz < 100 || hz > 10000) {
        TIM14->CCR1 = 0;
        return;
    }
    TIM14->ARR = 1000000u / hz - 1;
    TIM14->CCR1 = 500000u / hz;
}

static void hal_bootloader(void *ctx)
{
    (void)ctx;
    BOOT_FLAG = BOOT_FLAG_ROM;
    HAL_Delay(5); /* let the UARTs finish */
    NVIC_SystemReset();
}

/* Straight after a reset asked for by MSG_BOOTLOADER, nothing set up yet: map the system
   memory at 0 and jump into it. It listens on USART1 (and USART2, where the receiver
   sends CRSF; stm32flash's 0x7F still wins on USART1). */
static void rom_bootloader(void)
{
    BOOT_FLAG = 0;
    const uint32_t *sys = (const uint32_t *)SYSMEM_ADDR;
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();
    SCB->VTOR = SYSMEM_ADDR;
    __set_MSP(sys[0]);
    ((void (*)(void))sys[1])();
}

void SysTick_Handler(void) { HAL_IncTick(); }

int main(void)
{
    if (BOOT_FLAG == BOOT_FLAG_ROM)
        rom_bootloader();
    const uint32_t reset_cause = mcu_reset_cause();
    HAL_Init();
    mcu_clocks();
    board_gpio();

    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_32; /* ~1 kHz from LSI */
    hiwdg.Init.Reload = 500;                  /* 0.5 s */
    hiwdg.Init.Window = IWDG_WINDOW_DISABLE;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
        mcu_fail();

    uarts_init();
    spi_init();
    adc_init();
    timers_init();
    HAL_Delay(1);
    HAL_GPIO_WritePin(LORA_NRST_PORT, LORA_NRST_PIN, GPIO_PIN_SET); /* released: the module pulls up */
    HAL_Delay(6);                                                   /* SX1276 ready 5 ms after reset */

    static const struct io_hal hal = {
        .now_ms = hal_now, .write = hal_write, .adc_mv = hal_adc, .out = hal_out, .in = hal_in,
        .lidar_pwm = hal_lidar_pwm, .buzzer = hal_buzzer, .bootloader = hal_bootloader,
        .radio = &radio_io, .version = FW_VERSION,
    };
    static struct io_hal h;
    h = hal;
    h.reset_cause = reset_cause;
    static struct io io;
    io_init(&io, &h);

    uint32_t last = HAL_GetTick();
    for (;;) {
        uint8_t buf[64];
        for (int p = 0; p < PORTS; p++) {
            int n;
            while ((n = uart_read(&uarts[p], buf, sizeof(buf))) > 0)
                io_rx(&io, p, buf, n);
        }
        const uint32_t t = HAL_GetTick();
        if (t != last) {
            last = t;
            io_poll(&io);
            HAL_IWDG_Refresh(&hiwdg);
        }
    }
}
