#ifndef BOARD_H
#define BOARD_H

/* Leg cell analog front end, see hardware/side-board. */

#define SHUNT_MOHM 10.0f
#define INA_GAIN 20.0f      /* INA181A1 */
#define VBAT_RATIO 7.667f   /* 100k / 15k */
#define V6_RATIO 2.47f      /* 10k / 6.8k */
#define NTC_R25 10000.0f
#define NTC_BETA 3380.0f    /* NCP18XH103F03RB */
#define NTC_PULLUP 10000.0f /* to 3V3, NTC to GND */
#define VDDA_MV 3300.0f

#endif
