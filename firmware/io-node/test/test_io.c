#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "io.h"
#include "sx1276_sim.h"

static int failures, checks;

#define CHECK(cond) do { \
    checks++; \
    if (!(cond)) { failures++; printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } \
} while (0)

#define CHECK_EQ(a, b) do { \
    checks++; \
    long long _a = (a), _b = (b); \
    if (_a != _b) { failures++; printf("  FAIL %s:%d: %s = %lld, want %lld\n", __FILE__, __LINE__, #a, _a, _b); } \
} while (0)

/* ---- a board in memory ---- */

#define MAX_MSGS 4096
struct msg { uint8_t type; uint8_t p[LINK_MAX_PAYLOAD]; int n; };

static struct {
    uint32_t t;
    struct link_dec dec;
    struct msg msgs[MAX_MSGS];
    int nmsgs;
    uint8_t elrs[256];
    int nelrs;
    uint32_t adc[ADCS];
    bool out[OUTS], in[INS];
    uint16_t pwm, hz;
    int bootloader;
    bool host_full;
    struct sx1276_sim radio;
} b;

static uint32_t h_now(void *c) { (void)c; return b.t; }

static bool h_write(void *c, enum io_port port, const uint8_t *d, int n)
{
    (void)c;
    if (port == PORT_HOST) {
        if (b.host_full)
            return false;
        for (int i = 0; i < n; i++)
            if (link_feed(&b.dec, d[i]) && b.nmsgs < MAX_MSGS) {
                struct msg *m = &b.msgs[b.nmsgs++];
                m->type = b.dec.msg[0];
                m->n = b.dec.len - 1;
                memcpy(m->p, b.dec.msg + 1, m->n);
            }
    } else if (port == PORT_ELRS && b.nelrs + n <= (int)sizeof(b.elrs)) {
        memcpy(b.elrs + b.nelrs, d, n);
        b.nelrs += n;
    }
    return true;
}

static uint32_t h_adc(void *c, enum io_adc ch) { (void)c; return b.adc[ch]; }
static void h_out(void *c, enum io_out p, bool on) { (void)c; b.out[p] = on; }
static bool h_in(void *c, enum io_in p) { (void)c; return b.in[p]; }
static void h_pwm(void *c, uint16_t d) { (void)c; b.pwm = d; }
static void h_buzzer(void *c, uint16_t hz) { (void)c; b.hz = hz; }
static void h_boot(void *c) { (void)c; b.bootloader++; }

static struct io_hal hal = {
    .now_ms = h_now, .write = h_write, .adc_mv = h_adc, .out = h_out, .in = h_in, .lidar_pwm = h_pwm,
    .buzzer = h_buzzer, .bootloader = h_boot, .version = 0x1234567, .reset_cause = 2,
};
static struct io io;

static void start(bool radio)
{
    memset(&b, 0, sizeof(b));
    b.t = 1000;
    sx1276_sim_init(&b.radio);
    b.radio.present = radio;
    hal.radio = &b.radio.io;
    io_init(&io, &hal);
}

static void run(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++) {
        b.t++;
        io_poll(&io);
    }
}

static const struct msg *last(uint8_t type)
{
    for (int i = b.nmsgs - 1; i >= 0; i--)
        if (b.msgs[i].type == type)
            return &b.msgs[i];
    return NULL;
}

static int count(uint8_t type)
{
    int n = 0;
    for (int i = 0; i < b.nmsgs; i++)
        n += b.msgs[i].type == type;
    return n;
}

static void host(uint8_t type, const uint8_t *p, int n)
{
    uint8_t wire[LINK_MAX_WIRE];
    const int len = link_encode(type, p, n, wire);
    io_rx(&io, PORT_HOST, wire, len);
}

static struct io_status status(void)
{
    struct io_status s;
    io_status(&io, &s);
    return s;
}

/* ---- frame builders ---- */

static int crsf_frame(uint8_t type, const uint8_t *p, int n, uint8_t *out)
{
    out[0] = CRSF_SYNC;
    out[1] = (uint8_t)(n + 2);
    out[2] = type;
    memcpy(out + 3, p, n);
    out[n + 3] = crsf_crc8(out + 2, n + 1);
    return n + 4;
}

static void ld19_packet(uint8_t *p, uint16_t start_deg100)
{
    memset(p, 0, LD19_LEN);
    p[0] = 0x54;
    p[1] = 0x2C;
    p[2] = 0x10; p[3] = 0x0E; /* 3600 deg/s */
    p[4] = start_deg100 & 0xFF; p[5] = start_deg100 >> 8;
    for (int i = 0; i < 12; i++) {
        p[6 + 3 * i] = 0xE8; p[7 + 3 * i] = 0x03; p[8 + 3 * i] = 200; /* 1000 mm */
    }
    p[42] = (start_deg100 + 800) & 0xFF; p[43] = (start_deg100 + 800) >> 8;
    p[46] = ld19_crc8(p, LD19_LEN - 1);
}

/* "$<body>*HH\r\n" with a good checksum */
static int nmea(const char *body, char *out)
{
    uint8_t sum = 0;
    for (const char *c = body; *c; c++)
        sum ^= (uint8_t)*c;
    return sprintf(out, "$%s*%02X\r\n", body, sum);
}

static void gnss(const char *body)
{
    char s[128];
    const int n = nmea(body, s);
    io_rx(&io, PORT_GNSS, (const uint8_t *)s, n);
}

/* ---- tests ---- */

static void test_link(void)
{
    printf("link framing\n");
    struct link_dec d = {0};
    uint8_t p[LINK_MAX_PAYLOAD], wire[LINK_MAX_WIRE];
    srand(1);
    for (int round = 0; round < 300; round++) {
        const int n = round < 2 ? round * LINK_MAX_PAYLOAD : rand() % (LINK_MAX_PAYLOAD + 1);
        for (int i = 0; i < n; i++)
            p[i] = round == 1 ? 0xAA : round % 3 ? (uint8_t)rand() : (uint8_t)(rand() % 3); /* zero-heavy */
        const int len = link_encode(0x42, p, n, wire);
        CHECK(len > 0 && len <= LINK_MAX_WIRE);
        int zeros = 0;
        for (int i = 0; i < len; i++)
            zeros += !wire[i];
        CHECK_EQ(zeros, 1);
        CHECK_EQ(wire[len - 1], 0);
        bool got = false;
        for (int i = 0; i < len; i++)
            got = link_feed(&d, wire[i]);
        CHECK(got);
        CHECK_EQ(d.len, n + 1);
        CHECK(d.msg[0] == 0x42 && !memcmp(d.msg + 1, p, n));
    }
    CHECK_EQ(link_encode(1, p, LINK_MAX_PAYLOAD + 1, wire), 0);

    /* a flipped bit is dropped and counted, and the next message still gets through */
    const uint8_t q[] = {1, 2, 0, 3};
    int len = link_encode(7, q, 4, wire);
    wire[2] ^= 0x10;
    bool got = false;
    for (int i = 0; i < len; i++)
        got |= link_feed(&d, wire[i]);
    CHECK(!got);
    CHECK_EQ(d.bad, 1);
    len = link_encode(7, q, 4, wire);
    for (int i = 0; i < len; i++)
        got |= link_feed(&d, wire[i]);
    CHECK(got && d.len == 5);

    /* the same bytes as vector_io/link.py (test_link.py) */
    const uint8_t v[3] = {0x11, 0x00, 0x22}, want[8] = {0x03, 0x01, 0x11, 0x04, 0x22, 0x07, 0x82, 0x00};
    CHECK(link_encode(0x01, v, 3, wire) == 8 && !memcmp(wire, want, 8));

    /* CRC-16/CCITT-FALSE check value */
    CHECK_EQ(link_crc16((const uint8_t *)"123456789", 9), 0x29B1);
}

static void test_crsf(void)
{
    printf("CRSF from the receiver\n");
    start(true);
    uint8_t ch[22], f[64], stream[128];
    for (int i = 0; i < 22; i++)
        ch[i] = (uint8_t)(i * 37);
    const int n = crsf_frame(CRSF_RC_CHANNELS, ch, 22, f);
    CHECK_EQ(n, 26);
    /* noise, a frame split in two, a corrupted frame, a good one */
    int k = 0;
    stream[k++] = 0x13; stream[k++] = 0xC8; stream[k++] = 0x99;
    memcpy(stream + k, f, n); k += n;
    memcpy(stream + k, f, n); stream[k + 10] ^= 1; k += n;
    memcpy(stream + k, f, n); k += n;
    io_rx(&io, PORT_ELRS, stream, 20);
    io_rx(&io, PORT_ELRS, stream + 20, k - 20);
    CHECK_EQ(count(MSG_CRSF), 2);
    const struct msg *m = last(MSG_CRSF);
    CHECK(m && m->n == n && !memcmp(m->p, f, n));
    CHECK(io.crsf.bad >= 1);
    CHECK(status().flags & ST_CRSF);
    run(600);
    CHECK(!(status().flags & ST_CRSF));

    /* link statistics pass through but don't count as a link */
    start(true);
    uint8_t ls[10] = {50, 60, 100, 8, 0, 4, 2, 70, 100, 6};
    const int ln = crsf_frame(0x14, ls, 10, f);
    io_rx(&io, PORT_ELRS, f, ln);
    CHECK_EQ(count(MSG_CRSF), 1);
    CHECK(!(status().flags & ST_CRSF));

    /* telemetry from the CM5 goes out to the receiver as it is; junk doesn't */
    const uint8_t bat[8] = {0, 160, 0, 12, 0, 1, 0, 80};
    const int bn = crsf_frame(0x08, bat, 8, f);
    host(MSG_CRSF_OUT, f, bn);
    CHECK(b.nelrs == bn && !memcmp(b.elrs, f, bn));
    host(MSG_CRSF_OUT, (const uint8_t *)"\x01\x02\x03\x04", 4);
    CHECK_EQ(b.nelrs, bn);
}

static void test_lidar(void)
{
    printf("LD19 packets\n");
    start(true);
    uint8_t p[LD19_LEN], stream[4 * LD19_LEN + 8];
    int k = 0;
    stream[k++] = 0x54; stream[k++] = 0x54; stream[k++] = 0x00;
    for (int i = 0; i < 4; i++) {
        ld19_packet(p, (uint16_t)(i * 900));
        if (i == 2)
            p[20] ^= 0x40;
        memcpy(stream + k, p, LD19_LEN);
        k += LD19_LEN;
    }
    for (int i = 0; i < k; i += 5)
        io_rx(&io, PORT_LIDAR, stream + i, k - i < 5 ? k - i : 5);
    CHECK_EQ(count(MSG_LIDAR), 3);
    CHECK_EQ(io.ld19.bad, 1);
    ld19_packet(p, 2700);
    const struct msg *m = last(MSG_LIDAR);
    CHECK(m && m->n == LD19_LEN && !memcmp(m->p, p, LD19_LEN));
    CHECK(status().flags & ST_LIDAR);

    host(MSG_LIDAR_PWM, (const uint8_t *)"\x2c\x01", 2);
    CHECK_EQ(b.pwm, 300);
    host(MSG_LIDAR_PWM, (const uint8_t *)"\xff\xff", 2);
    CHECK_EQ(b.pwm, 1000);
}

static void test_gnss(void)
{
    printf("GNSS\n");
    struct gga g;
    const char *s = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
    CHECK(nmea_gga(s, (int)strlen(s), &g));
    CHECK_EQ(g.quality, 1);
    CHECK_EQ(g.sats, 8);
    CHECK_EQ(g.lat_e7, 481173000);
    CHECK_EQ(g.lon_e7, 115166666);
    CHECK_EQ(g.alt_dm, 5454);
    s = "$GNGGA,101010.00,3352.12345,S,15112.54321,W,2,12,0.8,-12.3,M,22.0,M,,*00";
    CHECK(nmea_gga(s, (int)strlen(s), &g));
    CHECK_EQ(g.lat_e7, -338687241);
    CHECK_EQ(g.lon_e7, -1512090535);
    CHECK_EQ(g.alt_dm, -123);
    s = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A";
    CHECK(!nmea_gga(s, (int)strlen(s), &g));

    start(true);
    gnss("GNGGA,,,,,,0,00,99.99,,,,,,");
    CHECK_EQ(count(MSG_NMEA), 1);
    struct io_status st = status();
    CHECK((st.flags & ST_GNSS) && !(st.flags & ST_FIX));
    gnss("GNGGA,101010.00,4426.16140,N,02606.16800,E,1,09,1.0,80.5,M,36.0,M,,");
    gnss("GNRMC,101010.00,A,4426.16140,N,02606.16800,E,0.012,,041026,,,A");
    CHECK_EQ(count(MSG_NMEA), 3);
    const struct msg *m = last(MSG_NMEA);
    CHECK(m && m->p[0] == '$' && m->p[m->n - 3] == '*' && memchr(m->p, '\r', m->n) == NULL);
    st = status();
    CHECK((st.flags & ST_FIX) && st.sats == 9 && st.fix == 1);
    CHECK_EQ(io.last_fix.lat_e7, 444360233);

    /* a bad checksum, an over-long line, a sentence with no '*' */
    const char *bad = "$GNGGA,1,2,3*00\r\n";
    io_rx(&io, PORT_GNSS, (const uint8_t *)bad, (int)strlen(bad));
    char longline[200] = "$";
    memset(longline + 1, 'A', 150);
    strcpy(longline + 151, "\r\n");
    io_rx(&io, PORT_GNSS, (const uint8_t *)longline, (int)strlen(longline));
    io_rx(&io, PORT_GNSS, (const uint8_t *)"$GNTXT,1\r\n", 10);
    CHECK_EQ(count(MSG_NMEA), 3);
    CHECK_EQ(io.nmea.bad, 3);
    run(2100);
    CHECK(!(status().flags & (ST_GNSS | ST_FIX)));
}

static void test_status(void)
{
    printf("status and housekeeping\n");
    start(true);
    b.adc[ADC_VBAT] = 2200;
    b.adc[ADC_5V] = 2510;
    b.adc[ADC_NTC] = 1650;
    b.in[IN_LTE_STATUS] = true;
    run(1000);
    CHECK(count(MSG_STATUS) >= 9 && count(MSG_STATUS) <= 11);
    const struct msg *m = last(MSG_STATUS);
    CHECK(m && m->n == STATUS_LEN);
    if (!m)
        return;
    CHECK_EQ(get_u32(m->p), 0x1234567);
    CHECK(b.t - get_u32(m->p + 4) < 100);
    CHECK_EQ(m->p[8], 2);
    CHECK_EQ(get_u16(m->p + 10), 12200);
    CHECK_EQ(get_u16(m->p + 12), 5020);
    CHECK_EQ((int16_t)get_u16(m->p + 14), 250);
    CHECK(m->p[9] & ST_LTE_STATUS);
    CHECK(m->p[9] & ST_LORA);
    CHECK(!(m->p[9] & ST_HOST));
    b.adc[ADC_NTC] = 3299; /* open */
    CHECK_EQ(status().temp_c10, INT16_MIN);
    b.adc[ADC_NTC] = 900; /* hot: about 53 degC */
    CHECK(status().temp_c10 > 520 && status().temp_c10 < 540);

    /* the LED blinks slower once the CM5 talks */
    int toggles = 0;
    bool led = b.out[OUT_LED];
    for (int i = 0; i < 1000; i++) {
        run(1);
        toggles += b.out[OUT_LED] != led;
        led = b.out[OUT_LED];
    }
    CHECK(toggles >= 7 && toggles <= 9);
    host(MSG_BEACON, (const uint8_t *)"\x1e\x00", 2);
    CHECK(status().flags & ST_HOST);
    toggles = 0;
    for (int i = 0; i < 1000; i++) {
        run(1);
        toggles += b.out[OUT_LED] != led;
        led = b.out[OUT_LED];
    }
    CHECK(toggles >= 1 && toggles <= 3);

    /* a full buffer to the CM5 loses messages and counts them */
    b.host_full = true;
    run(300);
    b.host_full = false;
    CHECK(status().dropped >= 3);
}

static void test_outputs(void)
{
    printf("LTE, buzzer, bootloader\n");
    start(true);
    CHECK_EQ(b.hz, 2700); /* start-up chirp */
    run(100);
    CHECK_EQ(b.hz, 0);
    CHECK(!b.out[OUT_LTE_EN]);
    host(MSG_LTE_POWER, (const uint8_t *)"\x01", 1);
    CHECK(b.out[OUT_LTE_EN]);
    CHECK(status().flags & ST_LTE_EN);

    /* 3 beeps of 100 ms, 50 ms apart */
    const uint8_t beep[7] = {0xD0, 0x07, 100, 0, 50, 0, 3};
    host(MSG_BEEP, beep, 7);
    int on = 0, edges = 0;
    bool was = b.hz != 0;
    CHECK(was && b.hz == 2000);
    for (int i = 0; i < 600; i++) {
        run(1);
        on += b.hz != 0;
        edges += (b.hz != 0) != was;
        was = b.hz != 0;
    }
    CHECK(on >= 297 && on <= 303);
    CHECK_EQ(edges, 5);
    CHECK_EQ(b.hz, 0);
    host(MSG_BEEP, beep, 7);
    host(MSG_BEEP, (const uint8_t *)"\0\0\0\0\0\0\0", 7);
    CHECK_EQ(b.hz, 0);

    host(MSG_BOOTLOADER, (const uint8_t *)"boox", 4);
    CHECK_EQ(b.bootloader, 0);
    uint8_t magic[4];
    put_u32(magic, LINK_BOOT_MAGIC);
    host(MSG_BOOTLOADER, magic, 4);
    CHECK_EQ(b.bootloader, 1);
    CHECK(!b.out[OUT_LTE_EN]); /* the module isn't left powered by a dead MCU */
}

static void test_lora(void)
{
    printf("LoRa\n");
    CHECK_EQ(lora_airtime_ms(16), 165);
    CHECK_EQ(lora_airtime_ms(1), 104);
    CHECK_EQ(lora_airtime_ms(64), 391);

    start(true);
    CHECK(io.radio.ok);
    CHECK(labs((long)sx1276_sim_freq(&b.radio) - (long)LORA_FREQ_HZ) < 62);
    CHECK_EQ(b.radio.reg[0x01], 0x85); /* LoRa, receiving */

    /* the beacon every 30 s, with the last fix */
    b.adc[ADC_VBAT] = 2200;
    gnss("GNGGA,101010.00,4426.16140,N,02606.16800,E,1,09,1.0,80.5,M,36.0,M,,");
    run(29990);
    CHECK_EQ(b.radio.sent_count, 0);
    gnss("GNGGA,101040.00,4426.16140,N,02606.16800,E,1,09,1.0,80.5,M,36.0,M,,");
    run(20);
    sx1276_sim_tick(&b.radio);
    CHECK_EQ(b.radio.sent_count, 1);
    CHECK_EQ(b.radio.sent_len, BEACON_LEN);
    const uint8_t *s = b.radio.sent;
    CHECK(s[0] == 'V' && s[1] == 0 && s[2] == 1 && s[3] == 9);
    CHECK_EQ((int32_t)get_u32(s + 4), 444360233);
    CHECK_EQ((int32_t)get_u32(s + 8), 261028000);
    CHECK_EQ((int16_t)get_u16(s + 12), 80);
    CHECK_EQ(get_u16(s + 14), 1220);
    run(5);
    CHECK_EQ(b.radio.reg[0x01], 0x85); /* back to receive after TxDone */

    /* duty cycle: a packet from the CM5 right after the beacon waits out 10x its air time */
    host(MSG_LORA_SEND, (const uint8_t *)"hello", 5);
    const struct msg *m = last(MSG_LORA_TX);
    CHECK(m && m->p[0] == 2);
    run(1650);
    host(MSG_LORA_SEND, (const uint8_t *)"hello", 5);
    m = last(MSG_LORA_TX);
    CHECK(m && m->p[0] == 0);
    sx1276_sim_tick(&b.radio);
    CHECK(b.radio.sent_count == 2 && b.radio.sent_len == 5 && !memcmp(b.radio.sent, "hello", 5));
    run(5);

    /* the next beacon with no fix: the last position, quality 0 */
    run(31000);
    sx1276_sim_tick(&b.radio);
    CHECK_EQ(b.radio.sent_count, 3);
    CHECK(b.radio.sent[1] == 1 && b.radio.sent[2] == 0 && (int32_t)get_u32(b.radio.sent + 4) == 444360233);
    CHECK_EQ(status().beacons, 2);
    run(5);

    /* received packets go to the CM5; CRC errors don't */
    sx1276_sim_rx(&b.radio, (const uint8_t *)"ping", 4, -97, -22, true);
    run(2);
    m = last(MSG_LORA_RX);
    CHECK(m && m->n == 7 && (int16_t)get_u16(m->p) == -97 && (int8_t)m->p[2] == -22 && !memcmp(m->p + 3, "ping", 4));
    sx1276_sim_rx(&b.radio, (const uint8_t *)"pong", 4, -97, -22, false);
    run(2);
    CHECK_EQ(count(MSG_LORA_RX), 1);

    /* beacon off */
    host(MSG_BEACON, (const uint8_t *)"\0\0", 2);
    run(40000);
    sx1276_sim_tick(&b.radio);
    CHECK_EQ(b.radio.sent_count, 3);

    /* no radio fitted */
    start(false);
    CHECK(!io.radio.ok && !(status().flags & ST_LORA));
    host(MSG_LORA_SEND, (const uint8_t *)"x", 1);
    m = last(MSG_LORA_TX);
    CHECK(m && m->p[0] == 3);
    run(31000);
}

int main(void)
{
    test_link();
    test_crsf();
    test_lidar();
    test_gnss();
    test_status();
    test_outputs();
    test_lora();
    printf("%d checks, %d failures\n", checks, failures);
    return failures != 0;
}
