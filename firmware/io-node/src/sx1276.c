#include "sx1276.h"

enum {
    REG_FIFO = 0x00, REG_OP_MODE = 0x01, REG_FRF_MSB = 0x06, REG_PA_CONFIG = 0x09, REG_OCP = 0x0B,
    REG_LNA = 0x0C, REG_FIFO_ADDR_PTR = 0x0D, REG_FIFO_TX_BASE = 0x0E, REG_FIFO_RX_BASE = 0x0F,
    REG_FIFO_RX_CURRENT = 0x10, REG_IRQ_FLAGS = 0x12, REG_RX_NB_BYTES = 0x13, REG_PKT_SNR = 0x19,
    REG_PKT_RSSI = 0x1A, REG_MODEM_CONFIG1 = 0x1D, REG_MODEM_CONFIG2 = 0x1E, REG_PREAMBLE_MSB = 0x20,
    REG_PAYLOAD_LENGTH = 0x22, REG_MODEM_CONFIG3 = 0x26, REG_SYNC_WORD = 0x39, REG_DIO_MAPPING1 = 0x40,
    REG_VERSION = 0x42, REG_PA_DAC = 0x4D
};
enum { MODE_LORA = 0x80, MODE_SLEEP = 0, MODE_STDBY = 1, MODE_TX = 3, MODE_RX_CONT = 5 };
enum { IRQ_RX_DONE = 0x40, IRQ_CRC_ERR = 0x20, IRQ_TX_DONE = 0x08 };

static uint8_t rd(struct sx1276 *r, uint8_t reg)
{
    uint8_t v = 0;
    r->io->read(r->io->ctx, reg, &v, 1);
    return v;
}

static void wr(struct sx1276 *r, uint8_t reg, uint8_t v) { r->io->write(r->io->ctx, reg, &v, 1); }

static void receive(struct sx1276 *r)
{
    wr(r, REG_DIO_MAPPING1, 0x00); /* DIO0 = RxDone */
    wr(r, REG_FIFO_ADDR_PTR, 0);
    wr(r, REG_OP_MODE, MODE_LORA | MODE_RX_CONT);
    r->tx = false;
}

bool sx1276_init(struct sx1276 *r, const struct sx1276_io *io)
{
    r->io = io;
    r->ok = r->tx = false;
    if (rd(r, REG_VERSION) != 0x12)
        return false;
    wr(r, REG_OP_MODE, MODE_SLEEP); /* LongRangeMode only changes in sleep */
    wr(r, REG_OP_MODE, MODE_LORA | MODE_SLEEP);
    const uint64_t frf = ((uint64_t)LORA_FREQ_HZ << 19) / 32000000u;
    const uint8_t f[3] = {(uint8_t)(frf >> 16), (uint8_t)(frf >> 8), (uint8_t)frf};
    r->io->write(r->io->ctx, REG_FRF_MSB, f, 3);
    wr(r, REG_PA_CONFIG, 0x80 | (LORA_POWER_DBM - 2)); /* PA_BOOST: 2 + OutputPower dBm */
    wr(r, REG_PA_DAC, 0x84);                             /* default, up to 17 dBm */
    wr(r, REG_OCP, 0x20 | 11);                           /* 100 mA */
    wr(r, REG_LNA, 0x23);                                /* max gain, HF boost */
    wr(r, REG_FIFO_TX_BASE, 0);
    wr(r, REG_FIFO_RX_BASE, 0);
    wr(r, REG_MODEM_CONFIG1, 0x72);                  /* 125 kHz, 4/5, explicit header */
    wr(r, REG_MODEM_CONFIG2, LORA_SF << 4 | 0x04);   /* CRC on */
    wr(r, REG_MODEM_CONFIG3, 0x04);                  /* AGC; symbols < 16 ms, no LDRO */
    const uint8_t pre[2] = {0, LORA_PREAMBLE};
    r->io->write(r->io->ctx, REG_PREAMBLE_MSB, pre, 2);
    wr(r, REG_SYNC_WORD, 0x12);
    wr(r, REG_IRQ_FLAGS, 0xFF);
    wr(r, REG_OP_MODE, MODE_LORA | MODE_STDBY);
    receive(r);
    r->ok = true;
    return true;
}

bool sx1276_send(struct sx1276 *r, const uint8_t *data, int n)
{
    if (!r->ok || r->tx || n <= 0 || n > LORA_MAX)
        return false;
    wr(r, REG_OP_MODE, MODE_LORA | MODE_STDBY);
    wr(r, REG_FIFO_ADDR_PTR, 0);
    r->io->write(r->io->ctx, REG_FIFO, data, n);
    wr(r, REG_PAYLOAD_LENGTH, (uint8_t)n);
    wr(r, REG_DIO_MAPPING1, 0x40); /* DIO0 = TxDone */
    wr(r, REG_IRQ_FLAGS, 0xFF);
    wr(r, REG_OP_MODE, MODE_LORA | MODE_TX);
    r->tx = true;
    return true;
}

enum sx1276_event sx1276_poll(struct sx1276 *r, uint8_t *data, int *n, int16_t *rssi, int8_t *snr4)
{
    if (!r->ok)
        return SX_NONE;
    const uint8_t irq = rd(r, REG_IRQ_FLAGS);
    if (!irq)
        return SX_NONE;
    wr(r, REG_IRQ_FLAGS, irq);
    if (r->tx) {
        if (!(irq & IRQ_TX_DONE))
            return SX_NONE;
        receive(r);
        return SX_TX_DONE;
    }
    if (!(irq & IRQ_RX_DONE) || (irq & IRQ_CRC_ERR))
        return SX_NONE;
    int len = rd(r, REG_RX_NB_BYTES);
    if (len > LORA_MAX)
        len = LORA_MAX;
    wr(r, REG_FIFO_ADDR_PTR, rd(r, REG_FIFO_RX_CURRENT));
    r->io->read(r->io->ctx, REG_FIFO, data, len);
    *n = len;
    *snr4 = (int8_t)rd(r, REG_PKT_SNR);
    *rssi = (int16_t)(-157 + rd(r, REG_PKT_RSSI)); /* HF port */
    return SX_RX;
}

uint32_t lora_airtime_ms(int n)
{
    /* Semtech AN1200.13: symbols = 8 + max(ceil((8n - 4SF + 28 + 16CRC) / 4SF) (CR + 4), 0),
       plus 12.25 for the preamble */
    const int num = 8 * n - 4 * LORA_SF + 28 + 16, den = 4 * LORA_SF;
    int sym = num > 0 ? (num + den - 1) / den * 5 : 0;
    const uint32_t sym_us = (1000000u << LORA_SF) / LORA_BW_HZ;
    const uint32_t us = (uint32_t)(sym + 8) * sym_us + (LORA_PREAMBLE * 4 + 17) * sym_us / 4;
    return (us + 999) / 1000;
}
