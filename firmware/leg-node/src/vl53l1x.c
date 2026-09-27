/*
 * VL53L1X driver. The default configuration and the timing tables come from ST's
 * VL53L1X ultra lite driver:
 *
 * COPYRIGHT(c) 2018 STMicroelectronics
 * Redistribution and use in source and binary forms, with or without modification, are
 * permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright notice, this list of
 *     conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright notice, this list
 *     of conditions and the following disclaimer in the documentation and/or other
 *     materials provided with the distribution.
 *  3. Neither the name of STMicroelectronics nor the names of its contributors may be used
 *     to endorse or promote products derived from this software without specific prior
 *     written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "vl53l1x.h"

#define REG_VHV_LOOP_BOUND 0x0008
#define REG_VHV_INIT 0x000B
#define REG_GPIO_HV_MUX 0x0030
#define REG_GPIO_STATUS 0x0031
#define REG_PHASECAL_TIMEOUT 0x004B
#define REG_TIMEOUT_A 0x005E
#define REG_VCSEL_A 0x0060
#define REG_TIMEOUT_B 0x0061
#define REG_VCSEL_B 0x0063
#define REG_VALID_PHASE_HIGH 0x0069
#define REG_INTERMEASUREMENT 0x006C
#define REG_WOI_SD0 0x0078
#define REG_INITIAL_PHASE_SD0 0x007A
#define REG_INT_CLEAR 0x0086
#define REG_MODE_START 0x0087
#define REG_RESULT 0x0089 /* range status; the distance sits at 0x0096 */
#define REG_OSC_CALIBRATE 0x00DE
#define REG_BOOT_STATE 0x00E5
#define REG_MODEL_ID 0x010F

#define MODEL_ID 0xEACC
#define CONFIG_FIRST 0x2D

static const uint8_t default_config[] = {
    0x00, 0x01, 0x01, 0x01, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08, 0x10, 0x01, 0x01, 0x00, 0x00, 0x00,
    0x00, 0xff, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0b, 0x00, 0x00, 0x02, 0x0a, 0x21,
    0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x38, 0xff, 0x01, 0x00, 0x08, 0x00,
    0x00, 0x01, 0xdb, 0x0f, 0x01, 0xf1, 0x0d, 0x01, 0x68, 0x00, 0x80, 0x08, 0xb8, 0x00, 0x00, 0x00,
    0x00, 0x0f, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0f, 0x0d, 0x0e, 0x0e, 0x00,
    0x00, 0x02, 0xc7, 0xff, 0x9B, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
};
_Static_assert(sizeof(default_config) == 0x87 - CONFIG_FIRST + 1, "config covers 0x2D-0x87");

struct budget { uint16_t ms, a, b; };

static const struct budget short_budgets[] = {
    {15, 0x001D, 0x0027}, {20, 0x0051, 0x006E}, {33, 0x00D6, 0x006E}, {50, 0x01AE, 0x01E8},
    {100, 0x02E1, 0x0388}, {200, 0x03E1, 0x0496}, {500, 0x0591, 0x05C1},
};
static const struct budget long_budgets[] = {
    {20, 0x001E, 0x0022}, {33, 0x0060, 0x006E}, {50, 0x00AD, 0x00C6},
    {100, 0x01CC, 0x01EA}, {200, 0x02D9, 0x02F8}, {500, 0x048F, 0x04A4},
};

static bool wr8(struct vl53l1x *d, uint16_t reg, uint8_t v) { return d->write(d->ctx, reg, &v, 1); }

static bool wr16(struct vl53l1x *d, uint16_t reg, uint16_t v)
{
    uint8_t b[2] = {v >> 8, v & 0xff};
    return d->write(d->ctx, reg, b, 2);
}

static bool wr32(struct vl53l1x *d, uint16_t reg, uint32_t v)
{
    uint8_t b[4] = {v >> 24, v >> 16, v >> 8, v};
    return d->write(d->ctx, reg, b, 4);
}

static bool rd8(struct vl53l1x *d, uint16_t reg, uint8_t *v) { return d->read(d->ctx, reg, v, 1); }

static bool rd16(struct vl53l1x *d, uint16_t reg, uint16_t *v)
{
    uint8_t b[2];
    if (!d->read(d->ctx, reg, b, 2))
        return false;
    *v = (uint16_t)(b[0] << 8 | b[1]);
    return true;
}

static int current_mode(struct vl53l1x *d)
{
    uint8_t v;
    if (!rd8(d, REG_PHASECAL_TIMEOUT, &v))
        return -1;
    return v == 0x14 ? VL53L1X_SHORT : v == 0x0A ? VL53L1X_LONG : 0;
}

static bool data_ready(struct vl53l1x *d, bool *ready)
{
    uint8_t v;
    if (!rd8(d, REG_GPIO_STATUS, &v))
        return false;
    *ready = (v & 1) == d->ready_level;
    return true;
}

bool vl53l1x_start(struct vl53l1x *d) { return wr8(d, REG_INT_CLEAR, 0x01) && wr8(d, REG_MODE_START, 0x40); }
bool vl53l1x_stop(struct vl53l1x *d) { return wr8(d, REG_MODE_START, 0x00); }

bool vl53l1x_set_budget(struct vl53l1x *d, uint16_t ms)
{
    int mode = current_mode(d);
    const struct budget *t = mode == VL53L1X_SHORT ? short_budgets : long_budgets;
    int n = mode == VL53L1X_SHORT ? (int)(sizeof(short_budgets) / sizeof(*t)) : (int)(sizeof(long_budgets) / sizeof(*t));
    if (mode <= 0)
        return false;
    for (int i = 0; i < n; i++)
        if (t[i].ms == ms)
            return wr16(d, REG_TIMEOUT_A, t[i].a) && wr16(d, REG_TIMEOUT_B, t[i].b);
    return false;
}

static int current_budget(struct vl53l1x *d)
{
    int mode = current_mode(d);
    uint16_t a;
    if (mode <= 0 || !rd16(d, REG_TIMEOUT_A, &a))
        return -1;
    const struct budget *t = mode == VL53L1X_SHORT ? short_budgets : long_budgets;
    int n = mode == VL53L1X_SHORT ? (int)(sizeof(short_budgets) / sizeof(*t)) : (int)(sizeof(long_budgets) / sizeof(*t));
    for (int i = 0; i < n; i++)
        if (t[i].a == a)
            return t[i].ms;
    return 100;
}

bool vl53l1x_set_mode(struct vl53l1x *d, enum vl53l1x_mode mode)
{
    /* the budget registers depend on the mode, so carry the budget over */
    int budget = current_budget(d);
    if (budget < 0)
        return false;
    bool s = mode == VL53L1X_SHORT;
    if (!(wr8(d, REG_PHASECAL_TIMEOUT, s ? 0x14 : 0x0A) && wr8(d, REG_VCSEL_A, s ? 0x07 : 0x0F) &&
          wr8(d, REG_VCSEL_B, s ? 0x05 : 0x0D) && wr8(d, REG_VALID_PHASE_HIGH, s ? 0x38 : 0xB8) &&
          wr16(d, REG_WOI_SD0, s ? 0x0705 : 0x0F0D) && wr16(d, REG_INITIAL_PHASE_SD0, s ? 0x0606 : 0x0E0E)))
        return false;
    if (budget == 15 && !s)
        budget = 20;
    return vl53l1x_set_budget(d, (uint16_t)budget);
}

bool vl53l1x_set_period(struct vl53l1x *d, uint16_t ms)
{
    uint16_t pll;
    if (!rd16(d, REG_OSC_CALIBRATE, &pll))
        return false;
    return wr32(d, REG_INTERMEASUREMENT, (uint32_t)((pll & 0x3FF) * ms * 1.075f));
}

bool vl53l1x_init(struct vl53l1x *d, enum vl53l1x_mode mode, uint16_t budget_ms, uint16_t period_ms)
{
    uint16_t id;
    uint8_t v = 0;
    for (int i = 0; i < 10 && !(v & 1); i++) {
        if (i)
            d->delay_ms(d->ctx, 1);
        if (!rd8(d, REG_BOOT_STATE, &v))
            v = 0;
    }
    if (!(v & 1) || !rd16(d, REG_MODEL_ID, &id) || id != MODEL_ID)
        return false;

    if (!d->write(d->ctx, CONFIG_FIRST, default_config, sizeof(default_config)) ||
        !rd8(d, REG_GPIO_HV_MUX, &v))
        return false;
    d->ready_level = !(v & 0x10);

    /* one throwaway measurement runs the VHV calibration; the default period is ~100 ms */
    bool ready = false;
    if (!vl53l1x_start(d))
        return false;
    for (int t = 0; !ready; t++) {
        if (t > 150 || !data_ready(d, &ready))
            return false;
        if (!ready)
            d->delay_ms(d->ctx, 1);
    }
    if (!wr8(d, REG_INT_CLEAR, 0x01) || !vl53l1x_stop(d) || !wr8(d, REG_VHV_LOOP_BOUND, 0x09) ||
        !wr8(d, REG_VHV_INIT, 0))
        return false;

    return vl53l1x_set_mode(d, mode) && vl53l1x_set_budget(d, budget_ms) &&
           vl53l1x_set_period(d, period_ms < budget_ms ? budget_ms : period_ms) && vl53l1x_start(d);
}

int vl53l1x_poll(struct vl53l1x *d, uint16_t *mm, uint8_t *status)
{
    static const uint8_t st_map[24] = {
        255, 255, 255, 5, 2, 4, 1, 7, 3, 0, 255, 255, 9, 13, 255, 255, 255, 255, 10, 6, 255, 255, 11, 12,
    };
    bool ready;
    uint8_t r[15];
    if (!data_ready(d, &ready))
        return -1;
    if (!ready)
        return 0;
    if (!d->read(d->ctx, REG_RESULT, r, sizeof(r)) || !wr8(d, REG_INT_CLEAR, 0x01))
        return -1;
    uint8_t raw = r[0] & 0x1F;
    *status = raw < sizeof(st_map) ? st_map[raw] : 255;
    *mm = *status == 0 ? (uint16_t)(r[13] << 8 | r[14]) : VL53L1X_NO_TARGET;
    return 1;
}
