#include "sx1276_sim.h"

#include <string.h>

enum { FIFO = 0x00, OP_MODE = 0x01, FIFO_ADDR_PTR = 0x0D, RX_CURRENT = 0x10, IRQ = 0x12, RX_NB = 0x13,
       SNR = 0x19, RSSI = 0x1A, PAYLOAD_LEN = 0x22, VERSION = 0x42 };

static void rd(void *ctx, uint8_t reg, uint8_t *data, int n)
{
    struct sx1276_sim *s = ctx;
    for (int i = 0; i < n; i++) {
        if (!s->present) {
            data[i] = 0;
        } else if (reg == FIFO) {
            data[i] = s->fifo[s->reg[FIFO_ADDR_PTR]++];
        } else {
            data[i] = s->reg[(reg + i) & 0x7F];
        }
    }
}

static void wr(void *ctx, uint8_t reg, const uint8_t *data, int n)
{
    struct sx1276_sim *s = ctx;
    if (!s->present)
        return;
    for (int i = 0; i < n; i++) {
        if (reg == FIFO)
            s->fifo[s->reg[FIFO_ADDR_PTR]++] = data[i];
        else if (reg == IRQ)
            s->reg[IRQ] &= (uint8_t)~data[i]; /* write 1 to clear */
        else
            s->reg[(reg + i) & 0x7F] = data[i];
    }
}

void sx1276_sim_init(struct sx1276_sim *s)
{
    memset(s, 0, sizeof(*s));
    s->present = true;
    s->reg[VERSION] = 0x12;
    s->reg[OP_MODE] = 0x09;
    s->io = (struct sx1276_io){.ctx = s, .read = rd, .write = wr};
}

void sx1276_sim_tick(struct sx1276_sim *s)
{
    if ((s->reg[OP_MODE] & 0x87) != 0x83)
        return;
    s->sent_len = s->reg[PAYLOAD_LEN];
    memcpy(s->sent, s->fifo, s->sent_len > LORA_MAX ? LORA_MAX : s->sent_len);
    s->sent_count++;
    s->reg[IRQ] |= 0x08;
    s->reg[OP_MODE] = 0x81; /* back to standby */
}

void sx1276_sim_rx(struct sx1276_sim *s, const uint8_t *data, int n, int rssi_dbm, int snr4, bool crc_ok)
{
    if ((s->reg[OP_MODE] & 0x87) != 0x85)
        return;
    const uint8_t at = 0x40;
    memcpy(s->fifo + at, data, n);
    s->reg[RX_CURRENT] = at;
    s->reg[RX_NB] = (uint8_t)n;
    s->reg[RSSI] = (uint8_t)(rssi_dbm + 157);
    s->reg[SNR] = (uint8_t)snr4;
    s->reg[IRQ] |= 0x40 | (crc_ok ? 0 : 0x20);
}

uint32_t sx1276_sim_freq(const struct sx1276_sim *s)
{
    const uint32_t frf = (uint32_t)s->reg[0x06] << 16 | s->reg[0x07] << 8 | s->reg[0x08];
    return (uint32_t)(((uint64_t)frf * 32000000u) >> 19);
}
