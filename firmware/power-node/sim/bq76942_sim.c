#include "bq76942_sim.h"

#include <math.h>
#include <string.h>

#include "bq76942.h"

#define ADDR_W (BQ76942_ADDR << 1)
#define ADDR_R ((BQ76942_ADDR << 1) | 1)

static uint8_t *mem(struct bq76942_sim *s, uint16_t addr) { return &s->mem[addr - BQS_MEM_BASE]; }

uint16_t bq76942_sim_mem16(const struct bq76942_sim *s, uint16_t addr)
{
    return s->mem[addr - BQS_MEM_BASE] | s->mem[addr - BQS_MEM_BASE + 1] << 8;
}

static void set16(struct bq76942_sim *s, uint16_t addr, uint16_t v)
{
    mem(s, addr)[0] = v & 0xff;
    mem(s, addr)[1] = v >> 8;
}

/* datasheet defaults of the settings the firmware touches, BQ7694202 flavour (REG1 on) */
void bq76942_sim_init(struct bq76942_sim *s, bool crc)
{
    memset(s, 0, sizeof(*s));
    s->awake = true;
    s->crc = crc;
    set16(s, 0x9234, 0x2982);
    *mem(s, 0x9236) = 0x0D;
    *mem(s, 0x9237) = 0x01;
    *mem(s, 0x92FD) = 0x07;
    *mem(s, 0x9303) = 0x05;
    *mem(s, 0x9308) = 0x0D;
    *mem(s, 0x930E) = 5;
    set16(s, 0x9343, 0x0040);
    *mem(s, 0x9261) = 0x88;
    *mem(s, 0x9275) = 50;
    *mem(s, 0x9278) = 86;
    *mem(s, 0x9280) = 2;
    *mem(s, 0x9282) = 4;
    *mem(s, 0x9283) = 1;
    *mem(s, 0x9284) = 3;
    *mem(s, 0x9285) = 7;
    *mem(s, 0x929A) = 55;
    *mem(s, 0x929C) = 50;
    set16(s, 0x923F, 0);
    s->host_fets = true;
    s->ts1_c = s->ts3_c = s->int_c = 25;
    bq76942_sim_cells(s, 3800);
}

void bq76942_sim_cells(struct bq76942_sim *s, uint16_t mv)
{
    memset(s->vc_mv, 0, sizeof(s->vc_mv));
    s->vc_mv[0] = s->vc_mv[1] = s->vc_mv[2] = s->vc_mv[9] = mv;
}

static uint8_t crc8(uint8_t crc, const uint8_t *p, int n) { return bq_crc8(crc, p, n); }

uint8_t bq76942_sim_fets(const struct bq76942_sim *s)
{
    if (!s->awake || s->cfgupdate || !s->fet_en || !s->host_fets)
        return 0;
    uint8_t f = BQ_FET_CHG | BQ_FET_DSG;
    if (s->safety_a & 0x04) /* CUV */
        f &= ~BQ_FET_DSG;
    return f;
}

static void subcommand(struct bq76942_sim *s, uint16_t cmd)
{
    s->subcmd = cmd;
    s->busy_once = true;
    s->buf_len = 0;
    if (cmd >= BQS_MEM_BASE && cmd < BQS_MEM_BASE + BQS_MEM_SIZE - 32) {
        memcpy(s->buf, mem(s, cmd), 32);
        s->buf_len = 32;
        return;
    }
    switch (cmd) {
    case BQ_DEVICE_NUMBER:
        s->buf[0] = 0x42;
        s->buf[1] = 0x76;
        s->buf_len = 2;
        break;
    case BQ_MANUFACTURING_STATUS:
        s->buf[0] = s->fet_en ? 0x10 : 0;
        s->buf[1] = 0;
        s->buf_len = 2;
        break;
    case BQ_DASTATUS6: {
        double whole = floor(s->accum_mah);
        int32_t w = (int32_t)whole;
        uint32_t frac = (uint32_t)((s->accum_mah - whole) * 4294967296.0);
        uint32_t secs = s->accum_ms / 1000;
        for (int i = 0; i < 4; i++) {
            s->buf[i] = (uint8_t)((uint32_t)w >> (8 * i));
            s->buf[4 + i] = (uint8_t)(frac >> (8 * i));
            s->buf[8 + i] = (uint8_t)(secs >> (8 * i));
        }
        s->buf_len = 32;
        break;
    }
    case BQ_SET_CFGUPDATE: s->cfgupdate = true; s->cfgupdates++; break;
    case BQ_EXIT_CFGUPDATE: s->cfgupdate = false; break;
    case BQ_FET_ENABLE: s->fet_en = !s->fet_en; break;
    case BQ_ALL_FETS_ON: s->host_fets = true; break;
    case BQ_ALL_FETS_OFF: s->host_fets = false; break;
    case BQ_RESET_PASSQ: s->accum_mah = 0; s->accum_ms = 0; break;
    case BQ_SHUTDOWN:
        s->shutdown = true;
        if (!s->ld_high)
            s->awake = false;
        break;
    }
    /* FET_INIT_OFF: after leaving CONFIG_UPDATE the FETs wait for the host */
    if (cmd == BQ_EXIT_CFGUPDATE && (*mem(s, 0x9308) & 0x20))
        s->host_fets = false;
}

static void commit(struct bq76942_sim *s, uint8_t sum, uint8_t len)
{
    uint8_t chk = (s->pending_addr & 0xff) + (s->pending_addr >> 8);
    for (int i = 0; i < s->pending_len; i++)
        chk += s->pending[i];
    if ((uint8_t)~chk != sum || len != s->pending_len + 4 || !s->cfgupdate) {
        s->rejected_writes++;
    } else {
        memcpy(mem(s, s->pending_addr), s->pending, s->pending_len);
        s->mem_writes++;
    }
    s->pending_len = 0;
}

bool bq76942_sim_write(struct bq76942_sim *s, const uint8_t *d, int n)
{
    if (!s->awake || s->fail_next > 0) {
        if (s->fail_next > 0)
            s->fail_next--;
        s->nacks++;
        return false;
    }
    uint8_t reg = d[0], data[40];
    int len = 0;
    for (int i = 1; i < n; i += s->crc ? 2 : 1) {
        if (s->crc) {
            uint8_t head[3] = {ADDR_W, reg, d[i]};
            uint8_t c = i == 1 ? crc8(0, head, 3) : crc8(0, &d[i], 1);
            if (i + 1 >= n || d[i + 1] != c) {
                s->crc_errors++;
                return false;
            }
        }
        data[len++] = d[i];
    }
    if (reg == BQ_SUBCMD && len >= 2) {
        uint16_t cmd = data[0] | data[1] << 8;
        if (len > 2) {
            s->pending_addr = cmd;
            s->pending_len = (uint8_t)(len - 2);
            memcpy(s->pending, data + 2, len - 2);
        } else {
            subcommand(s, cmd);
        }
    } else if (reg == BQ_CHECKSUM && len == 2 && s->pending_len) {
        commit(s, data[0], data[1]);
    }
    return true;
}

static int16_t kelvin10(int16_t c) { return (int16_t)(c * 10 + 2732); }

static void direct(struct bq76942_sim *s, uint8_t *regs)
{
    memset(regs, 0, 0x80);
    uint16_t st = s->cfgupdate ? BQ_ST_CFGUPDATE : 0;
    if (s->safety_a || s->safety_b || s->safety_c)
        st |= BQ_ST_SS;
    regs[0x03] = s->safety_a;
    regs[0x05] = s->safety_b;
    regs[0x07] = s->safety_c;
    regs[0x12] = st & 0xff;
    regs[0x13] = st >> 8;
    uint32_t stack = 0;
    for (int i = 0; i < 10; i++) {
        regs[0x14 + 2 * i] = s->vc_mv[i] & 0xff;
        regs[0x15 + 2 * i] = s->vc_mv[i] >> 8;
        stack += s->vc_mv[i];
    }
    if (*mem(s, 0x9303) & 0x04) /* USER_VOLTS_CV: centivolts */
        stack /= 10;
    uint8_t fets = bq76942_sim_fets(s);
    uint16_t pack = (fets & BQ_FET_DSG) ? (uint16_t)stack : 0;
    regs[0x34] = stack & 0xff;
    regs[0x35] = (uint8_t)(stack >> 8);
    regs[0x36] = regs[0x38] = pack & 0xff;
    regs[0x37] = regs[0x39] = pack >> 8;
    regs[0x3A] = (uint16_t)s->current_ma & 0xff;
    regs[0x3B] = (uint16_t)s->current_ma >> 8;
    const int16_t temps[][2] = {{0x68, s->int_c}, {0x70, s->ts1_c}, {0x74, s->ts3_c}};
    for (int i = 0; i < 3; i++) {
        uint16_t k = (uint16_t)kelvin10(temps[i][1]);
        regs[temps[i][0]] = k & 0xff;
        regs[temps[i][0] + 1] = k >> 8;
    }
    regs[0x7F] = fets;
}

bool bq76942_sim_read(struct bq76942_sim *s, uint8_t reg, uint8_t *out, int n)
{
    if (!s->awake || s->fail_next > 0) {
        if (s->fail_next > 0)
            s->fail_next--;
        s->nacks++;
        return false;
    }
    int count = s->crc ? n / 2 : n;
    uint8_t v[40];
    if (count > 40)
        return false;
    for (int i = 0; i < count; i++) {
        uint8_t r = (uint8_t)(reg + i);
        uint8_t regs[0x80];
        if (r == BQ_SUBCMD || r == BQ_SUBCMD + 1) {
            uint16_t back = s->busy_once ? 0xffff : s->subcmd;
            v[i] = r == BQ_SUBCMD ? back & 0xff : back >> 8;
            if (r == BQ_SUBCMD + 1 || count == 1)
                s->busy_once = false;
        } else if (r >= BQ_BUFFER && r < BQ_BUFFER + 32) {
            v[i] = s->buf[r - BQ_BUFFER];
        } else if (r == BQ_CHECKSUM) {
            uint8_t sum = (s->subcmd & 0xff) + (s->subcmd >> 8);
            for (int k = 0; k < s->buf_len; k++)
                sum += s->buf[k];
            v[i] = (uint8_t)~sum;
        } else if (r == BQ_CHECKSUM + 1) {
            v[i] = s->buf_len + 4;
        } else {
            direct(s, regs);
            v[i] = r < 0x80 ? regs[r] : 0;
        }
    }
    if (!s->crc) {
        memcpy(out, v, count);
        return true;
    }
    for (int i = 0; i < count; i++) {
        uint8_t head[4] = {ADDR_W, reg, ADDR_R, v[i]};
        out[2 * i] = v[i];
        out[2 * i + 1] = i == 0 ? crc8(0, head, 4) : crc8(0, &v[i], 1);
    }
    if (s->corrupt_next > 0) {
        s->corrupt_next--;
        out[1] ^= 0x5a;
    }
    return true;
}

void bq76942_sim_step(struct bq76942_sim *s, uint32_t ms)
{
    if (!s->awake)
        return;
    uint8_t fets = bq76942_sim_fets(s);
    bool flows = s->current_ma > 0 ? (fets & BQ_FET_CHG) : (fets & BQ_FET_DSG);
    if (flows) {
        s->accum_mah += s->current_ma * (double)ms / 3600000.0;
        s->accum_ms += ms;
    }
    /* CUV when enabled: threshold in 50.6 mV steps */
    if (*mem(s, 0x9261) & 0x04) {
        uint16_t cuv = (uint16_t)(*mem(s, 0x9275) * 50.6);
        bool under = false;
        uint16_t mode = bq76942_sim_mem16(s, 0x9304);
        for (int i = 0; i < 10; i++)
            if ((mode ? mode & (1 << i) : 1) && s->vc_mv[i] < cuv)
                under = true;
        if (under)
            s->safety_a |= 0x04;
        else
            s->safety_a &= ~0x04;
    }
}
