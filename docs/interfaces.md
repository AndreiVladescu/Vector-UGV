# Interfaces (board-to-board connectors & harnesses)

Draft. Freeze everything marked TBD before starting schematics.
Both ends of a cable use the same footprint and pin order.

## Overview

| ID | From → To | Connector | Carries |
|---|---|---|---|
| J-BAT | Pack → Power board | XT60 | VBAT (≈20 A peak) |
| J-BAL | Pack → Power board | JST-XH 5-pin | Cell taps C0–C4 |
| J-NTC | Pack → Power board | JST-PH 2×2-pin *TBD* | 2 cell NTCs |
| J-DCIN | GX-12 → Power board | GX-12 2-pin + JST-XH | 12–24 V charge input |
| J-USBC | Panel → Power board | USB-C receptacle (on board or panel pigtail *TBD*) | PD charge input |
| J-LEGPWR-L / -R | Power board → Side board | XT30 (PCB: XT30PW-M) | VBAT_L / VBAT_R (~13 A peak each) |
| J-SYSPWR | Power board → Carrier | Molex Micro-Fit 3.0 2×3 *TBD* | 5 V (6 A), VBAT_SYS, GND |
| J-SYSCTL | Power board → Carrier | JST-GH 6-pin *TBD* | PWR_EN, ESTOP_N, PGOOD, spare |
| J-CAN (×n) | Bus hops | JST-GH 4-pin | CANH, CANL, GND, ESTOP_N / spare |
| J-SERVO (×9 per side) | Side board → servo | 4-pin latched (JST-XH or 2.54 mm latching *TBD*) | V+ (6 V), GND, PWM, POT |
| J-TOF (×3 per side) | Side board → ToF | JST-GH 6-pin (latching; the cable crosses the coxa joint) | 3V3, GND, SDA, SCL, XSHUT, INT |
| J-SWD (per MCU) | Debug | Tag-Connect TC2030 or 1.27 mm 2×5 *TBD* | SWDIO, SWCLK, NRST, 3V3, GND |
| J-SWDREC | Carrier → any MCU | 1.27 mm 2×5 | CM5 GPIO SWD recovery |
| J-ELRS | Carrier → receiver | JST-GH 4-pin | 5 V, GND, TX, RX |
| J-LIDAR | Carrier → LD19 | JST-GH 4-pin | 5 V, GND, TX, RX |
| J-GIMBAL (×2) | Carrier → SG90 | 2.54 mm 3-pin | 5 V, GND, PWM |

## CAN bus

```
[120Ω] L3 ─ L2 ─ L1 ── J-CAN ── CARRIER (pass-through) ── J-CAN ── R1 ─ R2 ─ R3 [120Ω]
                                     │ stub < 30 cm
                                POWER BOARD
```

J-CAN pinout (JST-GH 4-pin):

| Pin | Signal |
|---|---|
| 1 | CANH |
| 2 | CANL |
| 3 | GND |
| 4 | ESTOP_N (open-drain, pulled up on the power board) *TBD* |

## J-SERVO pinout

| Pin | Signal | Wire |
|---|---|---|
| 1 | GND | brown |
| 2 | V+ 6.0 V | red |
| 3 | PWM | orange |
| 4 | POT wiper | added wire (pot mod) |

## J-TOF pinout

JST-GH 6-pin. The pin order matches the Pololu #3415 VL53L1X carrier.

| Pin | Signal | Side board end |
|---|---|---|
| 1 | 3V3 | 100 nF at the connector |
| 2 | GND | |
| 3 | SDA | 2.2k pull-up to 3V3, DNP if the breakout has its own |
| 4 | SCL | 2.2k pull-up to 3V3, DNP if the breakout has its own |
| 5 | XSHUT | MCU GPIO output |
| 6 | INT (GPIO1) | MCU GPIO input, 10k pull-up to 3V3 |

## Leg ID resistor divider

One ADC pin per leg cell reads a divider (3V3 → R_top → ADC → R_bot → GND). Values *TBD* (E24, ≥ 150 mV spacing):

| Node | ID | CAN node id |
|---|---|---|
| L1 | 1 | 0x1 |
| L2 | 2 | 0x2 |
| L3 | 3 | 0x3 |
| R1 | 4 | 0x4 |
| R2 | 5 | 0x5 |
| R3 | 6 | 0x6 |
| Power board | 7 | 0x7 (fixed in firmware) |
