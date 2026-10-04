#include "io.h"

#include <math.h>
#include <string.h>

#define STATUS_MS 100
#define CRSF_TIMEOUT_MS 500
#define LIDAR_TIMEOUT_MS 500
#define GNSS_TIMEOUT_MS 2000
#define HOST_TIMEOUT_MS 2000
#define BUZZER_HZ 2700 /* start-up chirp, near the buzzer's resonance */
/* lidar motor: about 0.36 rpm per permille from 5 V (240 rpm at 3.3 V); PI on the packets' speed */
#define MOTOR_MS 50
#define MOTOR_START 600   /* open-loop duty until packets arrive */
#define MOTOR_KP 2.0f     /* permille per rpm */
#define MOTOR_KI 8.0f     /* permille per rpm and second */

static uint32_t now(const struct io *io) { return io->hal->now_ms(io->hal->ctx); }
static bool recent(bool seen, uint32_t t, uint32_t now, uint32_t ms) { return seen && now - t < ms; }

static void send(struct io *io, uint8_t type, const uint8_t *p, int n)
{
    uint8_t wire[LINK_MAX_WIRE];
    const int len = link_encode(type, p, n, wire);
    if (!len || !io->hal->write(io->hal->ctx, PORT_HOST, wire, len))
        io->dropped++;
}

static void beep(struct io *io, uint16_t hz, uint16_t on, uint16_t off, uint8_t count)
{
    io->beep_hz = hz;
    io->beep_on = on;
    io->beep_off = off;
    io->beep_left = hz && on ? count : 0;
    io->beep_sounding = io->beep_left > 0;
    io->beep_t = now(io);
    io->hal->buzzer(io->hal->ctx, io->beep_sounding ? hz : 0);
}

static void beep_poll(struct io *io, uint32_t t)
{
    if (io->beep_sounding && t - io->beep_t >= io->beep_on) {
        io->beep_sounding = false;
        io->beep_left--;
        io->beep_t = t;
        io->hal->buzzer(io->hal->ctx, 0);
    } else if (!io->beep_sounding && io->beep_left && t - io->beep_t >= io->beep_off) {
        io->beep_sounding = true;
        io->beep_t = t;
        io->hal->buzzer(io->hal->ctx, io->beep_hz);
    }
}

enum { LORA_SENT, LORA_BUSY, LORA_DUTY_WAIT, LORA_NO_RADIO };

static int lora_send(struct io *io, const uint8_t *p, int n)
{
    const uint32_t t = now(io);
    if (!io->radio.ok)
        return LORA_NO_RADIO;
    if ((int32_t)(t - io->lora_free_at) < 0)
        return LORA_DUTY_WAIT;
    if (!sx1276_send(&io->radio, p, n))
        return LORA_BUSY;
    io->lora_free_at = t + lora_airtime_ms(n) * (100 / LORA_DUTY);
    return LORA_SENT;
}

void io_init(struct io *io, const struct io_hal *hal)
{
    memset(io, 0, sizeof(*io));
    io->hal = hal;
    io->beacon_s = BEACON_DEFAULT_S;
    const uint32_t t = now(io);
    io->t_beacon = t;
    io->lora_free_at = t;
    hal->out(hal->ctx, OUT_LTE_EN, false);
    hal->lidar_pwm(hal->ctx, 0);
    if (hal->radio)
        sx1276_init(&io->radio, hal->radio);
    beep(io, BUZZER_HZ, 60, 0, 1);
}

static void host_msg(struct io *io, const uint8_t *m, int len)
{
    const uint8_t type = m[0], *p = m + 1;
    const int n = len - 1;
    io->host_seen = true;
    io->t_host = now(io);
    switch (type) {
    case MSG_CRSF_OUT:
        if (n >= 4 && p[0] == CRSF_SYNC && p[1] == n - 2)
            io->hal->write(io->hal->ctx, PORT_ELRS, p, n);
        break;
    case MSG_LIDAR_RPM:
        if (n >= 2) {
            const uint16_t r = get_u16(p);
            io->lidar_rpm = r > 400 ? 400 : r;
        }
        break;
    case MSG_BEEP:
        if (n >= 7)
            beep(io, get_u16(p), get_u16(p + 2), get_u16(p + 4), p[6]);
        break;
    case MSG_LTE_POWER:
        if (n >= 1) {
            io->lte_en = p[0] != 0;
            io->hal->out(io->hal->ctx, OUT_LTE_EN, io->lte_en);
        }
        break;
    case MSG_LORA_SEND:
        if (n >= 1 && n <= LORA_MAX) {
            const uint8_t r = (uint8_t)lora_send(io, p, n);
            send(io, MSG_LORA_TX, &r, 1);
        }
        break;
    case MSG_BEACON:
        if (n >= 2)
            io->beacon_s = get_u16(p);
        break;
    case MSG_BOOTLOADER:
        if (n >= 4 && get_u32(p) == LINK_BOOT_MAGIC) {
            io->hal->out(io->hal->ctx, OUT_LTE_EN, false);
            io->hal->bootloader(io->hal->ctx);
        }
        break;
    default:
        break;
    }
}

void io_rx(struct io *io, enum io_port port, const uint8_t *data, int n)
{
    const uint32_t t = now(io);
    for (int i = 0; i < n; i++) {
        const uint8_t b = data[i];
        switch (port) {
        case PORT_HOST:
            if (link_feed(&io->dec, b))
                host_msg(io, io->dec.msg, io->dec.len);
            break;
        case PORT_ELRS:
            if (crsf_feed(&io->crsf, b)) {
                if (io->crsf.buf[2] == CRSF_RC_CHANNELS) {
                    io->crsf_seen = true;
                    io->t_crsf = t;
                }
                send(io, MSG_CRSF, io->crsf.buf, io->crsf.buf[1] + 2);
            }
            break;
        case PORT_LIDAR:
            if (xv11_feed(&io->xv11, b)) {
                io->lidar_seen = true;
                io->t_lidar = t;
                io->lidar_meas64 = xv11_rpm64(io->xv11.buf);
                send(io, MSG_LIDAR, io->xv11.buf, XV11_LEN);
            }
            break;
        case PORT_GNSS:
            if (nmea_feed(&io->nmea, b)) {
                io->gnss_seen = true;
                io->t_gnss = t;
                if (nmea_gga(io->nmea.buf, io->nmea.len, &io->gga) && io->gga.quality)
                    io->last_fix = io->gga;
                send(io, MSG_NMEA, (const uint8_t *)io->nmea.buf, io->nmea.len);
            }
            break;
        default:
            break;
        }
    }
}

static uint16_t vbat_mv(const struct io *io)
{
    return (uint16_t)(io->hal->adc_mv(io->hal->ctx, ADC_VBAT) * 122 / 22); /* 100k / 22k */
}

void io_status(const struct io *io, struct io_status *s)
{
    const struct io_hal *h = io->hal;
    const uint32_t t = h->now_ms(h->ctx);
    memset(s, 0, sizeof(*s));
    s->version = h->version;
    s->uptime_ms = t;
    s->reset_cause = (uint8_t)h->reset_cause;
    s->vbat_mv = vbat_mv(io);
    s->v5_mv = (uint16_t)(h->adc_mv(h->ctx, ADC_5V) * 2); /* 10k / 10k */
    /* NTC (10k, B 3380) to ground under a 10k pull-up to 3V3 */
    const uint32_t v = h->adc_mv(h->ctx, ADC_NTC);
    if (v > 10 && v < 3290) {
        const float r = 10000.0f * v / (3300 - v);
        const float k = 1.0f / (1.0f / 298.15f + logf(r / 10000.0f) / 3380.0f);
        s->temp_c10 = (int16_t)lroundf((k - 273.15f) * 10);
    } else {
        s->temp_c10 = INT16_MIN; /* open or shorted */
    }
    const bool gnss = recent(io->gnss_seen, io->t_gnss, t, GNSS_TIMEOUT_MS);
    s->flags = (io->lte_en ? ST_LTE_EN : 0) | (h->in(h->ctx, IN_LTE_STATUS) ? ST_LTE_STATUS : 0) |
               (recent(io->crsf_seen, io->t_crsf, t, CRSF_TIMEOUT_MS) ? ST_CRSF : 0) |
               (recent(io->lidar_seen, io->t_lidar, t, LIDAR_TIMEOUT_MS) ? ST_LIDAR : 0) |
               (gnss ? ST_GNSS : 0) | (gnss && io->gga.quality ? ST_FIX : 0) | (io->radio.ok ? ST_LORA : 0) |
               (recent(io->host_seen, io->t_host, t, HOST_TIMEOUT_MS) ? ST_HOST : 0);
    s->sats = gnss ? io->gga.sats : 0;
    s->fix = gnss ? io->gga.quality : 0;
    s->beacons = io->beacons;
    s->dropped = (uint16_t)io->dropped;
    s->bad_crsf = (uint16_t)io->crsf.bad;
    s->bad_lidar = (uint16_t)io->xv11.bad;
    s->bad_nmea = (uint16_t)io->nmea.bad;
    s->bad_link = (uint16_t)io->dec.bad;
}

int io_beacon(const struct io *io, uint8_t out[BEACON_LEN])
{
    const uint32_t t = io->hal->now_ms(io->hal->ctx);
    const bool fix = recent(io->gnss_seen, io->t_gnss, t, GNSS_TIMEOUT_MS) && io->gga.quality;
    const int32_t alt = io->last_fix.alt_dm / 10;
    uint8_t *p = out;
    *p++ = 'V';
    *p++ = io->beacon_seq;
    *p++ = fix ? io->gga.quality : 0; /* 0: the position is the last known one */
    *p++ = fix ? io->gga.sats : 0;
    p = put_u32(p, (uint32_t)io->last_fix.lat_e7);
    p = put_u32(p, (uint32_t)io->last_fix.lon_e7);
    p = put_u16(p, (uint16_t)(int16_t)(alt < INT16_MIN ? INT16_MIN : alt > INT16_MAX ? INT16_MAX : alt));
    put_u16(p, (uint16_t)(vbat_mv(io) / 10));
    return BEACON_LEN;
}

/* The LDS01RR's motor is ours to drive: open loop until packets come in, then PI on their speed */
static void motor_poll(struct io *io, uint32_t t)
{
    float duty = 0;
    if (!io->lidar_rpm) {
        io->lidar_i = 0;
    } else if (!recent(io->lidar_seen, io->t_lidar, t, 300)) {
        io->lidar_i = duty = MOTOR_START;
    } else {
        const float err = io->lidar_rpm - io->lidar_meas64 / 64.0f;
        io->lidar_i += MOTOR_KI * err * MOTOR_MS / 1000.0f;
        io->lidar_i = io->lidar_i < 0 ? 0 : io->lidar_i > 1000 ? 1000 : io->lidar_i;
        duty = io->lidar_i + MOTOR_KP * err;
    }
    io->lidar_duty = (uint16_t)(duty < 0 ? 0 : duty > 1000 ? 1000 : duty);
    io->hal->lidar_pwm(io->hal->ctx, io->lidar_duty);
}

void io_poll(struct io *io)
{
    const uint32_t t = now(io);
    const struct io_hal *h = io->hal;

    uint8_t pkt[LORA_MAX + 3];
    int n = 0;
    int16_t rssi = 0;
    int8_t snr = 0;
    switch (sx1276_poll(&io->radio, pkt + 3, &n, &rssi, &snr)) {
    case SX_RX:
        put_u16(pkt, (uint16_t)rssi);
        pkt[2] = (uint8_t)snr;
        send(io, MSG_LORA_RX, pkt, n + 3);
        break;
    default:
        break;
    }

    if (io->beacon_s && t - io->t_beacon >= io->beacon_s * 1000u) {
        uint8_t b[BEACON_LEN];
        io_beacon(io, b);
        if (lora_send(io, b, BEACON_LEN) == LORA_SENT) {
            io->beacon_seq++;
            io->beacons++;
            io->t_beacon = t;
        }
    }

    if (t - io->t_status >= STATUS_MS) {
        io->t_status = t;
        struct io_status s;
        uint8_t p[STATUS_LEN];
        io_status(io, &s);
        link_pack_status(&s, p);
        send(io, MSG_STATUS, p, STATUS_LEN);
    }

    beep_poll(io, t);

    if (t - io->t_motor >= MOTOR_MS) {
        io->t_motor = t;
        motor_poll(io, t);
    }

    /* LED: slow blink with the CM5 talking, fast without */
    const uint32_t half = recent(io->host_seen, io->t_host, t, HOST_TIMEOUT_MS) ? 500 : 125;
    if (t - io->t_led >= half) {
        io->t_led = t;
        io->led = !io->led;
        h->out(h->ctx, OUT_LED, io->led);
    }
}
