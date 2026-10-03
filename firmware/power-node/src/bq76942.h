#ifndef BQ76942_H
#define BQ76942_H

/* BQ76942 battery monitor over I2C. Register and data memory addresses from the BQ76942
   Technical Reference Manual (SLUUBY1B). The BQ7694202 variant boots with REG1 at 3.3 V
   (it powers our MCU) and with I2C CRC on; the plain BQ76942 has REG1 off. */

#include "hal.h"

#define BQ76942_ADDR 0x08
#define BQ_CELLS 4

/* direct commands */
enum {
    BQ_SAFETY_STATUS_A = 0x03, BQ_SAFETY_STATUS_B = 0x05, BQ_SAFETY_STATUS_C = 0x07,
    BQ_BATTERY_STATUS = 0x12, BQ_CELL1 = 0x14, BQ_STACK = 0x34, BQ_PACK = 0x36, BQ_LD = 0x38,
    BQ_CC2_CURRENT = 0x3A, BQ_INT_TEMP = 0x68, BQ_TS1_TEMP = 0x70, BQ_TS3_TEMP = 0x74,
    BQ_FET_STATUS = 0x7F,
    BQ_SUBCMD = 0x3E, BQ_BUFFER = 0x40, BQ_CHECKSUM = 0x60,
};

/* subcommands */
enum {
    BQ_DEVICE_NUMBER = 0x0001, BQ_SHUTDOWN = 0x0010, BQ_RESET = 0x0012, BQ_FET_ENABLE = 0x0022,
    BQ_MANUFACTURING_STATUS = 0x0057, BQ_DASTATUS6 = 0x0076, BQ_RESET_PASSQ = 0x0082,
    BQ_SET_CFGUPDATE = 0x0090, BQ_EXIT_CFGUPDATE = 0x0092, BQ_ALL_FETS_OFF = 0x0095,
    BQ_ALL_FETS_ON = 0x0096,
};

/* Battery Status bits */
enum { BQ_ST_CFGUPDATE = 1 << 0, BQ_ST_SS = 1 << 11, BQ_ST_SD_CMD = 1 << 13 };
/* FET Status bits */
enum { BQ_FET_CHG = 1 << 0, BQ_FET_PCHG = 1 << 1, BQ_FET_DSG = 1 << 2, BQ_FET_PDSG = 1 << 3 };

struct bq76942 {
    const struct power_hal *hal;
    bool crc;          /* I2C CRC, on for the BQ7694202 */
    uint16_t cell_mask; /* Vcell Mode: which VC inputs are cells, bottom to top */
    bool cell_ntc;      /* thermistor on TS1; without it the die temperature is the cell temperature */
    uint16_t errors;    /* failed transfers, for diagnostics */
};

struct bq_reading {
    uint16_t cell_mv[BQ_CELLS];
    uint16_t stack_mv, pack_mv, ld_mv;
    int16_t current_ma; /* positive = charging */
    int16_t cell_temp_c, fet_temp_c, int_temp_c;
    uint16_t battery_status;
    uint8_t safety_a, safety_b, safety_c, fets;
};

/* true when every setting already matches: nothing is written and the FETs stay as they
   are (an MCU reset must not glitch the pack). Otherwise CONFIG_UPDATE (which opens the
   FETs), write, verify. */
bool bq_configure(struct bq76942 *b, bool *changed);
bool bq_read(struct bq76942 *b, struct bq_reading *r);
/* charge passed through the sense resistor since the last RESET_PASSQ, mAh */
bool bq_passed_charge(struct bq76942 *b, float *mah);
bool bq_fets(struct bq76942 *b, bool on);
bool bq_shutdown(struct bq76942 *b);

bool bq_command(struct bq76942 *b, uint16_t cmd);
bool bq_subcmd_read(struct bq76942 *b, uint16_t cmd, uint8_t *data, int n);
bool bq_mem_read(struct bq76942 *b, uint16_t addr, uint8_t *data, int n);
bool bq_mem_write(struct bq76942 *b, uint16_t addr, const uint8_t *data, int n);
bool bq_direct_read(struct bq76942 *b, uint8_t cmd, uint8_t *data, int n);

uint8_t bq_crc8(uint8_t crc, const uint8_t *p, int n);

#endif
