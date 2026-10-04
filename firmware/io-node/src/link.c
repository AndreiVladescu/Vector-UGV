#include "link.h"

#include <string.h>

uint16_t link_crc16(const uint8_t *p, int n)
{
    uint16_t crc = 0xFFFF;
    while (n--) {
        crc ^= (uint16_t)*p++ << 8;
        for (int i = 0; i < 8; i++)
            crc = crc & 0x8000 ? (uint16_t)(crc << 1 ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

int link_encode(uint8_t type, const uint8_t *payload, int n, uint8_t *out)
{
    if (n < 0 || n > LINK_MAX_PAYLOAD)
        return 0;
    uint8_t raw[LINK_MAX_PAYLOAD + 3];
    raw[0] = type;
    memcpy(raw + 1, payload, n);
    put_u16(raw + 1 + n, link_crc16(raw, n + 1));
    const int len = n + 3;

    int code_at = 0, o = 1;
    uint8_t code = 1;
    for (int i = 0; i < len; i++) {
        if (raw[i]) {
            out[o++] = raw[i];
            code++;
        }
        if (!raw[i] || code == 0xFF) {
            out[code_at] = code;
            code_at = o++;
            code = 1;
        }
    }
    out[code_at] = code;
    out[o++] = 0;
    return o;
}

static int cobs_decode(const uint8_t *in, int n, uint8_t *out, int max)
{
    int o = 0;
    for (int i = 0; i < n;) {
        const uint8_t code = in[i++];
        if (!code || i + code - 1 > n)
            return -1;
        for (int k = 1; k < code; k++) {
            if (o >= max)
                return -1;
            out[o++] = in[i++];
        }
        if (code != 0xFF && i < n) {
            if (o >= max)
                return -1;
            out[o++] = 0;
        }
    }
    return o;
}

bool link_feed(struct link_dec *d, uint8_t b)
{
    if (b) {
        if (d->n < (int)sizeof(d->buf))
            d->buf[d->n++] = b;
        else
            d->overflow = true;
        return false;
    }
    const int n = d->n;
    const bool overflow = d->overflow;
    d->n = 0;
    d->overflow = false;
    if (!n)
        return false; /* back-to-back delimiters: resync, not an error */
    const int len = overflow ? -1 : cobs_decode(d->buf, n, d->msg, sizeof(d->msg));
    if (len < 3 || link_crc16(d->msg, len - 2) != get_u16(d->msg + len - 2)) {
        d->bad++;
        return false;
    }
    d->len = len - 2;
    return true;
}

void link_pack_status(const struct io_status *s, uint8_t out[STATUS_LEN])
{
    uint8_t *p = out;
    p = put_u32(p, s->version);
    p = put_u32(p, s->uptime_ms);
    *p++ = s->reset_cause;
    *p++ = s->flags;
    p = put_u16(p, s->vbat_mv);
    p = put_u16(p, s->v5_mv);
    p = put_u16(p, (uint16_t)s->temp_c10);
    *p++ = s->sats;
    *p++ = s->fix;
    p = put_u16(p, s->beacons);
    p = put_u16(p, s->dropped);
    p = put_u16(p, s->bad_crsf);
    p = put_u16(p, s->bad_lidar);
    p = put_u16(p, s->bad_nmea);
    put_u16(p, s->bad_link);
}
