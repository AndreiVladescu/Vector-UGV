#ifndef GAUGE_H
#define GAUGE_H

/* State of charge: the BQ76942's coulomb counter between two anchors, the resting cell
   voltage at power-up and "charge done" from the charger. */

#include <stdbool.h>
#include <stdint.h>

struct gauge {
    float capacity_mah;
    float soc;    /* 0..100 */
    float last_q; /* coulomb counter at the previous update, mAh */
    bool have_q;
};

/* resting voltage of one NMC cell to %, for the power-up estimate */
float gauge_ocv_soc(uint16_t cell_mv);
/* saved: the value kept in flash at the last power-off, < 0 if none. It wins unless the
   resting voltage says otherwise by more than 15 % (pack charged elsewhere, or self-discharge). */
void gauge_start(struct gauge *g, float capacity_mah, float saved, uint16_t rest_cell_mv);
void gauge_update(struct gauge *g, float passed_mah);
void gauge_set(struct gauge *g, float soc);

#endif
