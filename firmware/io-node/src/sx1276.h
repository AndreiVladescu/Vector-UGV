#ifndef SX1276_H
#define SX1276_H

#include <stdbool.h>
#include <stdint.h>

/* SX1276 in LoRa mode (RFM95W-868S2, PA_BOOST): 869.525 MHz (g3 sub-band, 10 % duty
   cycle), SF9, 125 kHz, CR 4/5, explicit header, CRC on, sync word 0x12, 14 dBm. The radio
   sits in continuous receive and goes back there after every packet sent. */

#define LORA_FREQ_HZ 869525000u
#define LORA_SF 9
#define LORA_BW_HZ 125000u
#define LORA_PREAMBLE 8
#define LORA_POWER_DBM 14
#define LORA_MAX 64

/* register access over SPI: n bytes from or to reg (bit 7 of the address = write) */
struct sx1276_io {
    void *ctx;
    void (*read)(void *ctx, uint8_t reg, uint8_t *data, int n);
    void (*write)(void *ctx, uint8_t reg, const uint8_t *data, int n);
};

struct sx1276 {
    const struct sx1276_io *io;
    bool ok, tx;
};

/* false when the chip doesn't answer with version 0x12 */
bool sx1276_init(struct sx1276 *r, const struct sx1276_io *io);
bool sx1276_send(struct sx1276 *r, const uint8_t *data, int n);

enum sx1276_event { SX_NONE, SX_TX_DONE, SX_RX };
/* polls the IRQ flags; on SX_RX the packet is in data (n bytes), with RSSI in dBm and SNR x4 */
enum sx1276_event sx1276_poll(struct sx1276 *r, uint8_t *data, int *n, int16_t *rssi, int8_t *snr4);

/* time on air of an n-byte packet with the settings above, in ms (rounded up) */
uint32_t lora_airtime_ms(int n);

#endif
