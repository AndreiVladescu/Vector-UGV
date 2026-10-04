#ifndef SX1276_SIM_H
#define SX1276_SIM_H

#include <stdbool.h>
#include <stdint.h>

#include "sx1276.h"

/* An SX1276 as far as the driver uses it: registers, the FIFO and its pointer, TX and RX
   done flags. sx1276_sim_tick finishes a transmission; sx1276_sim_rx delivers a packet. */
struct sx1276_sim {
    uint8_t reg[0x80];
    uint8_t fifo[256];
    uint8_t sent[LORA_MAX];
    int sent_len, sent_count;
    bool present;
    struct sx1276_io io;
};

void sx1276_sim_init(struct sx1276_sim *s);
void sx1276_sim_tick(struct sx1276_sim *s);
void sx1276_sim_rx(struct sx1276_sim *s, const uint8_t *data, int n, int rssi_dbm, int snr4, bool crc_ok);
uint32_t sx1276_sim_freq(const struct sx1276_sim *s);

#endif
