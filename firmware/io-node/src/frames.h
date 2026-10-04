#ifndef FRAMES_H
#define FRAMES_H

#include <stdbool.h>
#include <stdint.h>

/* Framers for the three serial devices: each takes one byte at a time and says when a
   whole frame with a good checksum is in its buffer. The IO MCU passes the frames to the
   CM5 untouched; it only reads what it needs itself (CRSF link, GNSS position). */

/* CRSF from the ELRS receiver: sync 0xC8, length (type..CRC), type, payload, CRC-8 0xD5 */
#define CRSF_SYNC 0xC8
#define CRSF_RC_CHANNELS 0x16
#define CRSF_MAX 64

struct crsf {
    uint8_t buf[CRSF_MAX];
    int n;
    uint32_t bad;
};

bool crsf_feed(struct crsf *c, uint8_t b); /* frame in buf[0 .. buf[1] + 1] */
uint8_t crsf_crc8(const uint8_t *p, int n);

/* LD19 lidar: 0x54 0x2C, speed, start angle, 12 points, end angle, timestamp, CRC-8 0x4D */
#define LD19_LEN 47

struct ld19 {
    uint8_t buf[LD19_LEN];
    int n;
    uint32_t bad;
};

bool ld19_feed(struct ld19 *l, uint8_t b);
uint8_t ld19_crc8(const uint8_t *p, int n);

/* NMEA 0183: '$' ... '*' two hex digits, CR LF */
#define NMEA_MAX 96

struct nmea {
    char buf[NMEA_MAX];
    int n, len;
    uint32_t bad;
};

bool nmea_feed(struct nmea *m, uint8_t b); /* sentence in buf, len chars, '$' to the checksum */

struct gga {
    int32_t lat_e7, lon_e7; /* degrees x 1e7 */
    int32_t alt_dm;         /* metres above MSL x 10 */
    uint8_t quality, sats;
};

/* true for a GGA sentence (any talker); a sentence with no fix gives quality 0 */
bool nmea_gga(const char *s, int n, struct gga *g);

#endif
