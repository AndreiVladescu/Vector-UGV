#include "bq76942.h"

#include <string.h>

enum { U1 = 1, U2 = 2 };

/* Data memory settings for the 4S pack, 1 mΩ shunt (CC Gain stays at its 1 mΩ default).
   Vcell Mode, TS1 and DA Configuration come from bq76942.cell_mask and .cell_ntc. */
static const struct setting {
    uint16_t addr;
    uint8_t size;
    int16_t value;
} settings[] = {
    {0x9234, U2, 0x2882}, /* Power Config: default with SLEEP off, the MCU is awake anyway */
    {0x9237, U1, 0x01},   /* REG0 Config: pre-regulator on */
    {0x9236, U1, 0x0D},   /* REG12 Config: REG1 on at 3.3 V (feeds the MCU buck enable) */
    {0x92FE, U1, 0x00},   /* TS2: wake input only, never a thermistor */
    {0x92FF, U1, 0x0F},   /* TS3: FET thermistor */
    {0x9300, U1, 0x00},   /* HDQ pin: unused */
    {0x9308, U1, 0x3D},   /* FET Options: series FETs, host control, predischarge, FETs wait for us */
    {0x930E, U1, 20},     /* Predischarge Timeout, 10 ms: 200 ms */
    {0x9343, U2, 0x0010}, /* Mfg Status Init: FET_EN (normal FET control), permanent fail off */
    {0x9261, U1, 0xBC},   /* Enabled Protections A: SCD OCD1 OCC COV CUV */
    {0x9262, U1, 0xF3},   /* Enabled Protections B: OTF OTINT OTD OTC UTD UTC */
    {0x9275, U1, 55},     /* CUV 2.78 V (50.6 mV steps) */
    {0x9278, U1, 84},     /* COV 4.25 V */
    {0x9280, U1, 4},      /* OCC 8 mV: 8 A charge */
    {0x9282, U1, 15},     /* OCD1 30 mV: 30 A */
    {0x9283, U1, 30},     /* OCD1 delay 3.3 ms steps: 100 ms */
    {0x9284, U1, 20},     /* OCD2 40 mV: 40 A */
    {0x9285, U1, 3},      /* OCD2 delay 10 ms */
    {0x9286, U1, 4},      /* SCD 80 mV: 80 A */
    {0x929A, U1, 45},     /* OTC 45 °C */
    {0x929C, U1, 40},     /* OTC recovery */
    {0x92A9, U1, -20},    /* UTD -20 °C: the robot works outdoors in winter */
    {0x92AB, U1, -15},    /* UTD recovery */
    {0x9335, U1, 0x03},   /* Balancing Configuration: while charging and at rest */
    {0x923F, U2, 2700},   /* Shutdown Cell Voltage: the BMS switches itself off below 2.7 V */
};
#define SETTINGS (sizeof(settings) / sizeof(settings[0]))
#define VCELL_MODE 0x9304
#define TS1_CONFIG 0x92FD
#define DA_CONFIG 0x9303

uint8_t bq_crc8(uint8_t crc, const uint8_t *p, int n)
{
    while (n--) {
        crc ^= *p++;
        for (int i = 0; i < 8; i++)
            crc = crc & 0x80 ? (uint8_t)(crc << 1) ^ 0x07 : (uint8_t)(crc << 1);
    }
    return crc;
}

static bool fail(struct bq76942 *b)
{
    b->errors++;
    return false;
}

static bool reg_write(struct bq76942 *b, uint8_t reg, const uint8_t *data, int n)
{
    uint8_t buf[1 + 2 * 36];
    int len = 0;
    buf[len++] = reg;
    for (int i = 0; i < n; i++) {
        buf[len++] = data[i];
        if (b->crc) {
            uint8_t head[3] = {BQ76942_ADDR << 1, reg, data[i]};
            buf[len++] = i == 0 ? bq_crc8(0, head, 3) : bq_crc8(0, &data[i], 1);
        }
    }
    return b->hal->i2c_write(b->hal->ctx, BQ76942_ADDR, buf, len) || fail(b);
}

static bool reg_read(struct bq76942 *b, uint8_t reg, uint8_t *data, int n)
{
    if (!b->crc)
        return b->hal->i2c_read(b->hal->ctx, BQ76942_ADDR, reg, data, n) || fail(b);
    uint8_t buf[2 * 36];
    if (n > 36 || !b->hal->i2c_read(b->hal->ctx, BQ76942_ADDR, reg, buf, 2 * n))
        return fail(b);
    for (int i = 0; i < n; i++) {
        uint8_t head[4] = {BQ76942_ADDR << 1, reg, (BQ76942_ADDR << 1) | 1, buf[2 * i]};
        uint8_t crc = i == 0 ? bq_crc8(0, head, 4) : bq_crc8(0, &buf[2 * i], 1);
        if (crc != buf[2 * i + 1])
            return fail(b);
        data[i] = buf[2 * i];
    }
    return true;
}

bool bq_direct_read(struct bq76942 *b, uint8_t cmd, uint8_t *data, int n)
{
    return reg_read(b, cmd, data, n);
}

static bool read16(struct bq76942 *b, uint8_t cmd, int16_t *v)
{
    uint8_t d[2];
    if (!reg_read(b, cmd, d, 2))
        return false;
    *v = (int16_t)(d[0] | d[1] << 8);
    return true;
}

/* 0x3E reads back 0xFFFF until a subcommand has been handled */
static bool wait_done(struct bq76942 *b, uint16_t cmd)
{
    for (int i = 0; i < 10; i++) {
        uint8_t d[2];
        if (reg_read(b, BQ_SUBCMD, d, 2) && (d[0] | d[1] << 8) == cmd)
            return true;
        b->hal->delay_ms(b->hal->ctx, 1);
    }
    return fail(b);
}

bool bq_command(struct bq76942 *b, uint16_t cmd)
{
    uint8_t d[2] = {cmd & 0xff, cmd >> 8};
    return reg_write(b, BQ_SUBCMD, d, 2) && wait_done(b, cmd);
}

/* subcommands with data and data memory reads share the 32-byte transfer buffer */
bool bq_subcmd_read(struct bq76942 *b, uint16_t cmd, uint8_t *data, int n)
{
    uint8_t d[2] = {cmd & 0xff, cmd >> 8}, sum[2], buf[32];
    if (n > 32 || !reg_write(b, BQ_SUBCMD, d, 2) || !wait_done(b, cmd) || !reg_read(b, BQ_CHECKSUM, sum, 2))
        return false;
    int len = sum[1] - 4;
    if (len < n || len > 32 || !reg_read(b, BQ_BUFFER, buf, len))
        return fail(b);
    uint8_t s = d[0] + d[1];
    for (int i = 0; i < len; i++)
        s += buf[i];
    if ((uint8_t)~s != sum[0])
        return fail(b);
    memcpy(data, buf, n);
    return true;
}

bool bq_mem_read(struct bq76942 *b, uint16_t addr, uint8_t *data, int n)
{
    return bq_subcmd_read(b, addr, data, n);
}

bool bq_mem_write(struct bq76942 *b, uint16_t addr, const uint8_t *data, int n)
{
    uint8_t buf[2 + 32];
    if (n > 32)
        return false;
    buf[0] = addr & 0xff;
    buf[1] = addr >> 8;
    memcpy(buf + 2, data, n);
    uint8_t s = 0;
    for (int i = 0; i < n + 2; i++)
        s += buf[i];
    uint8_t tail[2] = {(uint8_t)~s, (uint8_t)(n + 4)};
    bool ok = reg_write(b, BQ_SUBCMD, buf, n + 2) && reg_write(b, BQ_CHECKSUM, tail, 2);
    b->hal->delay_ms(b->hal->ctx, 2);
    return ok;
}

static void encode(const struct setting *s, uint8_t *d)
{
    d[0] = (uint16_t)s->value & 0xff;
    d[1] = (uint16_t)s->value >> 8;
}

static bool matches(struct bq76942 *b, const struct setting *s)
{
    uint8_t want[2], got[2];
    encode(s, want);
    return bq_mem_read(b, s->addr, got, s->size) && memcmp(want, got, s->size) == 0;
}

static bool status_bit(struct bq76942 *b, uint16_t bit, bool want)
{
    for (int i = 0; i < 10; i++) {
        int16_t st;
        if (read16(b, BQ_BATTERY_STATUS, &st) && ((st & bit) != 0) == want)
            return true;
        b->hal->delay_ms(b->hal->ctx, 1);
    }
    return fail(b);
}

static bool fet_control_on(struct bq76942 *b)
{
    uint8_t m[2];
    if (!bq_subcmd_read(b, BQ_MANUFACTURING_STATUS, m, 2))
        return false;
    return (m[0] & 0x10) || bq_command(b, BQ_FET_ENABLE); /* FET_EN toggles */
}

bool bq_configure(struct bq76942 *b, bool *changed)
{
    enum { N = SETTINGS + 3 };
    struct setting all[N];
    memcpy(all, settings, sizeof(settings));
    all[SETTINGS] = (struct setting){VCELL_MODE, U2, (int16_t)b->cell_mask};
    /* TS1: cell thermistor (18k model) or unused; DA: mV and mA, plus TINT_EN (the die stands in
       for the cell temperature) without a thermistor */
    all[SETTINGS + 1] = (struct setting){TS1_CONFIG, U1, b->cell_ntc ? 0x07 : 0x00};
    all[SETTINGS + 2] = (struct setting){DA_CONFIG, U1, b->cell_ntc ? 0x01 : 0x09};

    bool diff[N], any = false;
    for (unsigned i = 0; i < N; i++) {
        diff[i] = !matches(b, &all[i]);
        any |= diff[i];
    }
    *changed = any;
    if (any) {
        if (!bq_command(b, BQ_SET_CFGUPDATE) || !status_bit(b, BQ_ST_CFGUPDATE, true))
            return false;
        for (unsigned i = 0; i < N; i++) {
            uint8_t d[2];
            encode(&all[i], d);
            if (diff[i] && !bq_mem_write(b, all[i].addr, d, all[i].size))
                return false;
        }
        if (!bq_command(b, BQ_EXIT_CFGUPDATE) || !status_bit(b, BQ_ST_CFGUPDATE, false))
            return false;
        for (unsigned i = 0; i < N; i++)
            if (!matches(b, &all[i]))
                return fail(b);
        bq_command(b, BQ_RESET_PASSQ);
    }
    return fet_control_on(b);
}

static int16_t kelvin10_to_c(int16_t v)
{
    int d = v - 2732;
    return (int16_t)((d + (d >= 0 ? 5 : -5)) / 10);
}

bool bq_read(struct bq76942 *b, struct bq_reading *r)
{
    int16_t v;
    int cell = 0;
    for (int i = 0; i < 10 && cell < BQ_CELLS; i++) {
        if (!(b->cell_mask & (1 << i)))
            continue;
        if (!read16(b, BQ_CELL1 + 2 * i, &v))
            return false;
        r->cell_mv[cell++] = v < 0 ? 0 : (uint16_t)v;
    }
    static const uint8_t temps[3] = {BQ_TS1_TEMP, BQ_TS3_TEMP, BQ_INT_TEMP};
    int16_t t[3];
    for (int i = 0; i < 3; i++) {
        if (!read16(b, temps[i], &v))
            return false;
        t[i] = kelvin10_to_c(v);
    }
    r->cell_temp_c = b->cell_ntc ? t[0] : t[2];
    r->fet_temp_c = t[1];
    r->int_temp_c = t[2];

    int16_t stack, pack, ld, st;
    if (!read16(b, BQ_STACK, &stack) || !read16(b, BQ_PACK, &pack) || !read16(b, BQ_LD, &ld) ||
        !read16(b, BQ_CC2_CURRENT, &r->current_ma) || !read16(b, BQ_BATTERY_STATUS, &st) ||
        !reg_read(b, BQ_SAFETY_STATUS_A, &r->safety_a, 1) || !reg_read(b, BQ_SAFETY_STATUS_B, &r->safety_b, 1) ||
        !reg_read(b, BQ_SAFETY_STATUS_C, &r->safety_c, 1) || !reg_read(b, BQ_FET_STATUS, &r->fets, 1))
        return false;
    r->stack_mv = (uint16_t)stack;
    r->pack_mv = (uint16_t)pack;
    r->ld_mv = (uint16_t)ld;
    r->battery_status = (uint16_t)st;
    return true;
}

bool bq_passed_charge(struct bq76942 *b, float *mah)
{
    uint8_t d[12];
    if (!bq_subcmd_read(b, BQ_DASTATUS6, d, 12))
        return false;
    int32_t whole = (int32_t)(d[0] | d[1] << 8 | d[2] << 16 | (uint32_t)d[3] << 24);
    uint32_t frac = d[4] | d[5] << 8 | d[6] << 16 | (uint32_t)d[7] << 24;
    *mah = (float)whole + (float)frac / 4294967296.0f;
    return true;
}

bool bq_fets(struct bq76942 *b, bool on)
{
    return bq_command(b, on ? BQ_ALL_FETS_ON : BQ_ALL_FETS_OFF);
}

/* once is enough while unsealed; REG1 and with it this MCU go away */
bool bq_shutdown(struct bq76942 *b)
{
    uint8_t d[2] = {BQ_SHUTDOWN & 0xff, BQ_SHUTDOWN >> 8};
    return reg_write(b, BQ_SUBCMD, d, 2);
}
