#ifndef LINK_H
#define LINK_H

#include <stdbool.h>
#include <stdint.h>

/* IO MCU <-> CM5 on UART2 (CM5 GPIO4/5), 1 Mbaud 8N1. A message is type, payload and a
   CRC-16/CCITT-FALSE (little endian) over type and payload, COBS-encoded and ended by a
   zero byte. See firmware/io-node/README.md. */

#define LINK_VERSION 1
#define LINK_MAX_PAYLOAD 250
#define LINK_MAX_WIRE (LINK_MAX_PAYLOAD + 3 + (LINK_MAX_PAYLOAD + 3) / 254 + 2)
#define LINK_BOOT_MAGIC 0x746f6f62u /* "boot" */

enum link_type {
    /* IO MCU -> CM5 */
    MSG_STATUS = 0x01,  /* 10 Hz, struct below */
    MSG_CRSF = 0x02,    /* a CRSF frame from the receiver, sync to CRC */
    MSG_LIDAR = 0x03,   /* an LD19 packet, 47 bytes */
    MSG_NMEA = 0x04,    /* an NMEA sentence, '$' to the checksum, no CR LF */
    MSG_LORA_RX = 0x05, /* i16 RSSI dBm, i8 SNR x4, the packet */
    MSG_LORA_TX = 0x06, /* u8 0 sent, 1 radio busy, 2 duty cycle, 3 no radio */
    /* CM5 -> IO MCU */
    MSG_CRSF_OUT = 0x81,  /* a CRSF frame for the receiver (telemetry) */
    MSG_LIDAR_PWM = 0x82, /* u16 duty in 0.1 %, 0 = pin low, the LD19 runs its own 10 Hz */
    MSG_BEEP = 0x83,      /* u16 Hz, u16 on ms, u16 off ms, u8 count; count 0 stops */
    MSG_LTE_POWER = 0x84, /* u8 on */
    MSG_LORA_SEND = 0x85, /* the packet, up to 64 bytes */
    MSG_BEACON = 0x86,    /* u16 interval s, 0 = off */
    MSG_BOOTLOADER = 0x87 /* u32 LINK_BOOT_MAGIC: reset into ST's ROM bootloader on USART1 */
};

enum status_flag {
    ST_LTE_EN = 1 << 0,
    ST_LTE_STATUS = 1 << 1,
    ST_CRSF = 1 << 2,  /* RC channels in the last 500 ms */
    ST_LIDAR = 1 << 3, /* a good LD19 packet in the last 500 ms */
    ST_GNSS = 1 << 4,  /* a sentence in the last 2 s */
    ST_FIX = 1 << 5,   /* GGA fix quality > 0 */
    ST_LORA = 1 << 6,  /* the RFM95W answered at start-up */
    ST_HOST = 1 << 7   /* a message from the CM5 in the last 2 s */
};

/* MSG_STATUS payload, little endian, packed by link_put_* (no struct layout on the wire) */
struct io_status {
    uint32_t version;   /* FW_VERSION */
    uint32_t uptime_ms;
    uint8_t reset_cause; /* RESET_* */
    uint8_t flags;       /* status_flag */
    uint16_t vbat_mv;
    uint16_t v5_mv;
    int16_t temp_c10;    /* NTC on the 5 V buck, 0.1 degC */
    uint8_t sats;
    uint8_t fix;         /* GGA quality */
    uint16_t beacons;    /* LoRa beacons sent */
    uint16_t dropped;    /* messages to the CM5 lost to a full buffer */
    uint16_t bad_crsf, bad_lidar, bad_nmea, bad_link;
};
#define STATUS_LEN 30

/* COBS + CRC; returns the bytes written to out (at most LINK_MAX_WIRE), 0 if too long */
int link_encode(uint8_t type, const uint8_t *payload, int n, uint8_t *out);
uint16_t link_crc16(const uint8_t *p, int n);
void link_pack_status(const struct io_status *s, uint8_t out[STATUS_LEN]);

/* Byte-wise decoder. link_feed returns true when a whole good message is in d->msg
   (d->msg[0] = type, payload follows, d->len counts both). */
struct link_dec {
    uint8_t buf[LINK_MAX_WIRE];
    int n;
    bool overflow;
    uint8_t msg[LINK_MAX_PAYLOAD + 3];
    int len;
    uint32_t bad;
};

bool link_feed(struct link_dec *d, uint8_t b);

static inline uint16_t get_u16(const uint8_t *p) { return (uint16_t)(p[0] | p[1] << 8); }
static inline uint32_t get_u32(const uint8_t *p) { return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24; }
static inline uint8_t *put_u16(uint8_t *p, uint16_t v) { p[0] = v; p[1] = v >> 8; return p + 2; }
static inline uint8_t *put_u32(uint8_t *p, uint32_t v)
{
    p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24;
    return p + 4;
}

#endif
