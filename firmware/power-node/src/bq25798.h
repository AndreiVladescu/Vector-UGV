#ifndef BQ25798_H
#define BQ25798_H

/* BQ25798 buck-boost charger (datasheet SLUSDV2C). 16-bit registers are big-endian. Its
   watchdog puts the charge settings back to the PROG defaults (4S, 1 A) when we stop
   talking to it, so bq25798_poll re-applies them whenever that happened. */

#include "hal.h"

#define BQ25798_ADDR 0x6B

enum {
    CHG_VREG = 0x01, CHG_ICHG = 0x03, CHG_IINDPM = 0x06, CHG_CTRL0 = 0x0F, CHG_CTRL1 = 0x10,
    CHG_NTC1 = 0x18, CHG_STATUS0 = 0x1B, CHG_STATUS1 = 0x1C, CHG_FAULT0 = 0x20, CHG_FAULT1 = 0x21,
    CHG_ADC_CTRL = 0x2E, CHG_IBUS_ADC = 0x31, CHG_PART = 0x48,
};

/* CHG_STAT */
enum { CHG_NOT_CHARGING, CHG_TRICKLE, CHG_PRE, CHG_FAST, CHG_TAPER, CHG_TOPOFF = 6, CHG_DONE };

struct charge_settings {
    uint16_t charge_mv, charge_ma, input_ma;
    bool enable;
};

struct charger_reading {
    uint8_t status0, chg_stat, fault0, fault1;
    bool vbus, power_good; /* single input: VAC1 and VAC2 are tied to VBUS */
    int16_t ibus_ma, ibat_ma;
    uint16_t vbus_mv, vbat_mv;
};

struct bq25798 {
    const struct power_hal *hal;
    struct charge_settings set;
    bool applied;
    uint16_t errors, resets; /* resets: times the watchdog or a reset lost our settings */
};

bool bq25798_apply(struct bq25798 *c, const struct charge_settings *s);
/* call about once a second: feeds the watchdog, re-applies lost settings, reads status */
bool bq25798_poll(struct bq25798 *c, struct charger_reading *r);

#endif
