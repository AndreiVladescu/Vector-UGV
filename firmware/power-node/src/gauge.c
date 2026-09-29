#include "gauge.h"

static const struct { uint16_t mv; uint8_t soc; } ocv[] = {
    {3000, 0}, {3300, 3}, {3450, 8}, {3550, 15}, {3600, 22}, {3650, 30}, {3700, 40}, {3750, 50},
    {3800, 58}, {3850, 65}, {3900, 72}, {3950, 78}, {4000, 84}, {4050, 89}, {4100, 94}, {4150, 98}, {4200, 100},
};
#define OCV_POINTS (sizeof(ocv) / sizeof(ocv[0]))

static float clamp(float v) { return v < 0 ? 0 : v > 100 ? 100 : v; }

float gauge_ocv_soc(uint16_t mv)
{
    if (mv <= ocv[0].mv)
        return 0;
    for (unsigned i = 1; i < OCV_POINTS; i++)
        if (mv < ocv[i].mv)
            return ocv[i - 1].soc + (float)(ocv[i].soc - ocv[i - 1].soc) * (mv - ocv[i - 1].mv) / (ocv[i].mv - ocv[i - 1].mv);
    return 100;
}

void gauge_start(struct gauge *g, float capacity_mah, float saved, uint16_t rest_cell_mv)
{
    float est = gauge_ocv_soc(rest_cell_mv);
    g->capacity_mah = capacity_mah;
    g->soc = saved >= 0 && saved - est < 15 && est - saved < 15 ? saved : est;
    g->have_q = false;
}

void gauge_update(struct gauge *g, float passed_mah)
{
    if (g->have_q)
        g->soc = clamp(g->soc + (passed_mah - g->last_q) * 100.0f / g->capacity_mah);
    g->last_q = passed_mah;
    g->have_q = true;
}

void gauge_set(struct gauge *g, float soc) { g->soc = clamp(soc); }
