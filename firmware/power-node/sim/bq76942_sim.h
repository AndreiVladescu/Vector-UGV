#ifndef BQ76942_SIM_H
#define BQ76942_SIM_H

/* Register-level BQ76942: direct commands, the subcommand buffer with its checksum, data
   memory that only CONFIG_UPDATE mode can change, FET control, CUV, the coulomb counter,
   SHUTDOWN, and the I2C CRC of the BQ7694202. */

#include <stdbool.h>
#include <stdint.h>

#define BQS_MEM_BASE 0x9180
#define BQS_MEM_SIZE 0x300

struct bq76942_sim {
    bool awake, crc;
    uint8_t mem[BQS_MEM_SIZE];
    bool cfgupdate, fet_en, host_fets;
    uint16_t subcmd;
    bool busy_once; /* the first readback after a subcommand says 0xFFFF */
    uint8_t buf[32], buf_len;
    uint16_t pending_addr;
    uint8_t pending[32], pending_len;

    uint16_t vc_mv[10];  /* per VC input; unused ones 0 */
    int16_t current_ma;  /* positive = charging */
    int16_t ts1_c, hdq_c, ts3_c, int_c;
    bool ld_high;        /* something (a charger) keeps LD up: SHUTDOWN waits */
    double accum_mah;
    uint32_t accum_ms;
    uint8_t safety_a, safety_b, safety_c;

    int cfgupdates, mem_writes, rejected_writes, crc_errors, nacks;
    int fail_next;       /* NACK this many transfers */
    int corrupt_next;    /* return this many reads with a bad CRC byte */
    bool shutdown;
};

void bq76942_sim_init(struct bq76942_sim *s, bool crc);
/* the 4 cells on VC1, VC2, VC3 and VC10, as TI asks for fewer than 10 cells */
void bq76942_sim_cells(struct bq76942_sim *s, uint16_t mv);
bool bq76942_sim_write(struct bq76942_sim *s, const uint8_t *d, int n);
bool bq76942_sim_read(struct bq76942_sim *s, uint8_t reg, uint8_t *out, int n);
void bq76942_sim_step(struct bq76942_sim *s, uint32_t ms);
uint8_t bq76942_sim_fets(const struct bq76942_sim *s);
uint16_t bq76942_sim_mem16(const struct bq76942_sim *s, uint16_t addr);

#endif
