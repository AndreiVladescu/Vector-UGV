/* CAN bootloader for the leg node, 16 KB at the start of flash.

   After reset: if the application asked for it (BOOT_FLAG_ENTER) or there's no valid
   image, stay and take commands. Otherwise listen 200 ms for an ENTER, so a broken
   application can still be replaced, then reset into the application. Jumping only right
   after a reset means the application starts with every peripheral untouched. */

#include "board.h"
#include "boot.h"

#define LISTEN_MS 200

static FDCAN_HandleTypeDef hcan;

void SysTick_Handler(void) { HAL_IncTick(); }

static void send(void *ctx, const struct can_frame_t *f)
{
    (void)ctx;
    uint32_t t = HAL_GetTick();
    while (!board_can_send(&hcan, f) && HAL_GetTick() - t < 10) {
    }
}

/* If the application started the watchdog it may still be counting; refreshing a
   stopped one does nothing. */
static void kick(void) { IWDG->KR = 0xAAAA; }

static bool erase(void *ctx, uint32_t offset, uint32_t len)
{
    (void)ctx;
    FLASH_EraseInitTypeDef e = {.TypeErase = FLASH_TYPEERASE_PAGES, .NbPages = 1};
    uint32_t bad;
    bool ok = HAL_FLASH_Unlock() == HAL_OK;
    for (uint32_t a = offset; ok && a < offset + len; a += FLASH_PAGE_SIZE) {
        kick(); /* a page takes up to 40 ms */
        e.Page = (APP_ADDR + a - FLASH_BASE) / FLASH_PAGE_SIZE;
        ok = HAL_FLASHEx_Erase(&e, &bad) == HAL_OK;
    }
    HAL_FLASH_Lock();
    return ok;
}

static bool program(void *ctx, uint32_t offset, const uint8_t data[8])
{
    (void)ctx;
    uint64_t v;
    __builtin_memcpy(&v, data, 8);
    bool ok = HAL_FLASH_Unlock() == HAL_OK &&
              HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, APP_ADDR + offset, v) == HAL_OK;
    HAL_FLASH_Lock();
    return ok && *(const volatile uint64_t *)(APP_ADDR + offset) == v;
}

static const struct boot_io io = {
    .image = (const uint8_t *)APP_ADDR, .size = APP_SIZE, .page = FLASH_PAGE_SIZE,
    .sram_lo = SRAM_BASE, .sram_hi = SRAM_BASE + SRAM_SIZE_MAX, .image_addr = APP_ADDR,
    .erase = erase, .program = program, .send = send,
};

static void jump(void)
{
    const uint32_t *app = (const uint32_t *)APP_ADDR;
    SCB->VTOR = APP_ADDR;
    __set_MSP(app[0]);
    ((void (*)(void))app[1])();
}

static void restart(uint32_t flag)
{
    HAL_Delay(2); /* the last reply leaves the controller */
    BOOT_FLAG = flag;
    NVIC_SystemReset();
}

int main(void)
{
    uint32_t flag = BOOT_FLAG;
    BOOT_FLAG = 0;
    if (flag != BOOT_FLAG_RUN) /* the reset we do ourselves to start the application doesn't count */
        BOOT_RESET_CAUSE = board_reset_cause();
    if (flag == BOOT_FLAG_RUN && boot_image_valid(&io))
        jump();

    HAL_Init();
    board_clocks();
    board_gpio();
    board_can(&hcan, board_leg_id());

    struct boot b;
    boot_init(&b, &io, board_leg_id());
    bool stay = flag == BOOT_FLAG_ENTER || !boot_image_valid(&io);
    if (stay)
        boot_reply(&io, b.node, BOOT_ENTER, BOOT_OK,
                   BOOT_VERSION | (boot_image_valid(&io) ? 0x100 : 0) | (APP_SIZE / 1024) << 16);

    uint32_t t0 = HAL_GetTick();
    for (;;) {
        struct can_frame_t f;
        board_can_recover(&hcan);
        if (board_can_recv(&hcan, &f)) {
            if (!stay && boot_is_enter(&f, b.node))
                stay = true;
            if (stay && boot_frame(&b, &f))
                restart(BOOT_FLAG_RUN);
        }
        kick();
        uint32_t t = HAL_GetTick();
        if (!stay && t - t0 > LISTEN_MS)
            restart(BOOT_FLAG_RUN);
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, stay && t % 100 < 50 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}
