#ifndef IO_H
#define IO_H

#include <stdbool.h>
#include <stdint.h>

#include "frames.h"
#include "link.h"
#include "sx1276.h"

/* The IO co-processor on the CM5 carrier: ELRS (CRSF), LD19 lidar, GNSS, the RFM95W, ADC
   housekeeping, buzzer and the LTE supply, all behind one UART to the CM5. */

enum io_port { PORT_HOST, PORT_ELRS, PORT_LIDAR, PORT_GNSS, PORTS };
enum io_adc { ADC_VBAT, ADC_5V, ADC_NTC, ADCS };
enum io_out { OUT_LTE_EN, OUT_LED, OUTS };
enum io_in { IN_LTE_STATUS, INS };

struct io_hal {
    void *ctx;
    uint32_t (*now_ms)(void *ctx);
    /* queue bytes for a UART; false (and nothing queued) when they don't fit */
    bool (*write)(void *ctx, enum io_port port, const uint8_t *data, int n);
    uint32_t (*adc_mv)(void *ctx, enum io_adc ch); /* at the pin */
    void (*out)(void *ctx, enum io_out pin, bool on);
    bool (*in)(void *ctx, enum io_in pin);
    void (*lidar_pwm)(void *ctx, uint16_t permille);
    void (*buzzer)(void *ctx, uint16_t hz); /* 0 = off */
    void (*bootloader)(void *ctx);          /* doesn't return on the board */
    const struct sx1276_io *radio;
    uint32_t version, reset_cause;
};

#define BEACON_LEN 16
#define BEACON_DEFAULT_S 30
#define LORA_DUTY 10 /* percent, g3 sub-band */

struct io {
    const struct io_hal *hal;
    struct link_dec dec;
    struct crsf crsf;
    struct ld19 ld19;
    struct nmea nmea;
    struct sx1276 radio;
    struct gga gga;        /* the last GGA */
    struct gga last_fix;   /* the last GGA with a fix, for the beacon */
    uint32_t t_crsf, t_lidar, t_gnss, t_host, t_status, t_beacon, t_led;
    uint32_t lora_free_at; /* end of the duty-cycle off time after the last packet */
    uint16_t beacon_s, beacons;
    uint8_t beacon_seq;
    bool lte_en, led, host_seen, crsf_seen, lidar_seen, gnss_seen;
    /* buzzer pattern */
    uint16_t beep_hz, beep_on, beep_off;
    uint8_t beep_left;
    bool beep_sounding;
    uint32_t beep_t;
    uint32_t dropped;
};

void io_init(struct io *io, const struct io_hal *hal);
/* bytes from a UART */
void io_rx(struct io *io, enum io_port port, const uint8_t *data, int n);
/* call every millisecond or so */
void io_poll(struct io *io);

/* the beacon packet: 'V', seq, fix quality, sats, lat e7, lon e7 (i32), alt m (i16), vbat 10 mV (u16) */
int io_beacon(const struct io *io, uint8_t out[BEACON_LEN]);
void io_status(const struct io *io, struct io_status *s);

#endif
