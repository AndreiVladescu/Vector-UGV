#include "frames.h"

#include <string.h>

static uint8_t crc8(uint8_t poly, const uint8_t *p, int n)
{
    uint8_t crc = 0;
    while (n--) {
        crc ^= *p++;
        for (int i = 0; i < 8; i++)
            crc = crc & 0x80 ? (uint8_t)(crc << 1 ^ poly) : (uint8_t)(crc << 1);
    }
    return crc;
}

uint8_t crsf_crc8(const uint8_t *p, int n) { return crc8(0xD5, p, n); }

uint16_t xv11_checksum(const uint8_t *p)
{
    uint32_t c = 0;
    for (int i = 0; i < 10; i++)
        c = (c << 1) + (uint32_t)(p[2 * i] | p[2 * i + 1] << 8);
    return (uint16_t)(((c & 0x7FFF) + (c >> 15)) & 0x7FFF);
}

/* drop the first byte and look for the next start in what is left */
static int resync(uint8_t *buf, int n, uint8_t start)
{
    for (int i = 1; i < n; i++)
        if (buf[i] == start) {
            memmove(buf, buf + i, n - i);
            return n - i;
        }
    return 0;
}

bool crsf_feed(struct crsf *c, uint8_t b)
{
    if (c->n == 0 && b != CRSF_SYNC)
        return false;
    c->buf[c->n++] = b;
    for (;;) {
        if (c->n < 2)
            return false;
        const int len = c->buf[1];
        if (len < 2 || len > CRSF_MAX - 2) {
            c->n = resync(c->buf, c->n, CRSF_SYNC);
            continue;
        }
        if (c->n < len + 2)
            return false;
        if (crsf_crc8(c->buf + 2, len - 1) == c->buf[len + 1]) {
            c->n = 0;
            return true;
        }
        c->bad++;
        c->n = resync(c->buf, c->n, CRSF_SYNC);
    }
}

bool xv11_feed(struct xv11 *l, uint8_t b)
{
    if ((l->n == 0 && b != 0xFA) || (l->n == 1 && (b < 0xA0 || b > 0xF9))) {
        l->n = b == 0xFA;
        if (l->n)
            l->buf[0] = b;
        return false;
    }
    l->buf[l->n++] = b;
    if (l->n < XV11_LEN)
        return false;
    l->n = 0;
    if (xv11_checksum(l->buf) == (uint16_t)(l->buf[20] | l->buf[21] << 8))
        return true;
    l->bad++;
    return false;
}

static int hex(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    return -1;
}

bool nmea_feed(struct nmea *m, uint8_t b)
{
    if (b == '$') {
        m->n = 0;
        m->buf[m->n++] = '$';
        return false;
    }
    if (m->n == 0)
        return false;
    if (b == '\r' || b == '\n') {
        const int n = m->n;
        m->n = 0;
        if (n < 4 || m->buf[n - 3] != '*') {
            m->bad++;
            return false;
        }
        uint8_t sum = 0;
        for (int i = 1; i < n - 3; i++)
            sum ^= (uint8_t)m->buf[i];
        const int hi = hex(m->buf[n - 2]), lo = hex(m->buf[n - 1]);
        if (hi < 0 || lo < 0 || sum != (hi << 4 | lo)) {
            m->bad++;
            return false;
        }
        m->len = n; /* stays in buf until the next '$' */
        m->buf[n] = 0;
        return true;
    }
    if (m->n >= NMEA_MAX - 1) {
        m->n = 0;
        m->bad++;
        return false;
    }
    m->buf[m->n++] = (char)b;
    return false;
}

/* field k (0 = the address) of a sentence; its length, or -1 when there are fewer fields */
static int field(const char *s, int n, int k, const char **out)
{
    int i = 0;
    for (; k > 0 && i < n; i++)
        if (s[i] == ',')
            k--;
    if (k)
        return -1;
    int j = i;
    while (j < n && s[j] != ',' && s[j] != '*')
        j++;
    *out = s + i;
    return j - i;
}

/* a decimal like "4426.1234" -> value x 10^scale, digits past the scale cut off */
static bool decimal(const char *f, int n, int scale, int64_t *v)
{
    int64_t x = 0;
    int frac = -1;
    bool neg = false, any = false;
    for (int i = 0; i < n; i++) {
        if (i == 0 && f[i] == '-') {
            neg = true;
        } else if (f[i] == '.' && frac < 0) {
            frac = 0;
        } else if (f[i] >= '0' && f[i] <= '9') {
            any = true;
            if (frac >= scale)
                continue;
            x = x * 10 + (f[i] - '0');
            if (frac >= 0)
                frac++;
        } else {
            return false;
        }
    }
    for (int k = frac < 0 ? 0 : frac; k < scale; k++)
        x *= 10;
    *v = neg ? -x : x;
    return any;
}

/* ddmm.mmmm (or dddmm.mmmm) and a hemisphere -> degrees x 1e7 */
static bool coord(const char *f, int n, const char *h, int hn, int32_t *out)
{
    int64_t v; /* minutes field as ddmm x 1e7 */
    if (hn != 1 || !decimal(f, n, 7, &v) || v < 0)
        return false;
    const int64_t deg = v / 1000000000LL, min_e7 = v % 1000000000LL;
    int64_t e7 = deg * 10000000LL + min_e7 / 60;
    if (*h == 'S' || *h == 'W')
        e7 = -e7;
    else if (*h != 'N' && *h != 'E')
        return false;
    *out = (int32_t)e7;
    return true;
}

bool nmea_gga(const char *s, int n, struct gga *g)
{
    const char *f, *h;
    int fn = field(s, n, 0, &f);
    if (fn != 6 || memcmp(f + 3, "GGA", 3))
        return false;
    memset(g, 0, sizeof(*g));
    int64_t v;
    if ((fn = field(s, n, 6, &f)) > 0 && decimal(f, fn, 0, &v))
        g->quality = (uint8_t)v;
    if ((fn = field(s, n, 7, &f)) > 0 && decimal(f, fn, 0, &v))
        g->sats = (uint8_t)v;
    if (!g->quality)
        return true;
    int hn;
    fn = field(s, n, 2, &f);
    hn = field(s, n, 3, &h);
    if (fn <= 0 || !coord(f, fn, h, hn, &g->lat_e7))
        g->quality = 0;
    fn = field(s, n, 4, &f);
    hn = field(s, n, 5, &h);
    if (fn <= 0 || !coord(f, fn, h, hn, &g->lon_e7))
        g->quality = 0;
    if ((fn = field(s, n, 9, &f)) > 0 && decimal(f, fn, 1, &v))
        g->alt_dm = (int32_t)v;
    return true;
}
