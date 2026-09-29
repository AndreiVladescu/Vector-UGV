# Interfaces (board-to-board connectors & harnesses)

Draft. Freeze everything marked TBD before starting schematics.
Both ends of a cable use the same footprint and pin order.

## Overview

| ID | From → To | Connector | Carries |
|---|---|---|---|
| J-BAT | Pack → Power board | XT60 | VBAT (≈20 A peak) |
| J-BAL | Pack → Power board | JST-XH 5-pin | Cell taps C0–C4 |
| J-NTC | Pack → Power board | JST-PH 2×2-pin *TBD* | 2 cell NTCs |
| J-DCIN | GX-12 → Power board | GX-12 2-pin + JST-XH | 12–20 V charge input |
| J-USBC | Panel → Power board | USB-C receptacle (on board or panel pigtail *TBD*) | PD charge input, 20 V |
| J-PWRBTN | Panel → Power board | JST-PH 2-pin | momentary power button |
| J-ESTOP | Panel → Power board | JST-PH 2-pin | latching e-stop, normally closed |
| J-LEGPWR-L / -R | Power board → Side board | XT30 (PCB: XT30PW-M) | VBAT_L / VBAT_R (~13 A peak each) |
| J-SYSPWR | Power board → Carrier | Molex Micro-Fit 3.0 2×3 *TBD* | 5 V (6 A), VBAT_SYS, GND |
| J-SYSCTL | Power board ↔ Carrier | JST-GH 6-pin | see below |
| J-CAN (×n) | Bus hops | JST-GH 4-pin | CANH, CANL, GND, ESTOP_N / spare |
| J-SERVO (×9 per side) | Side board → servo | 4-pin latched (JST-XH or 2.54 mm latching *TBD*) | V+ (6 V), GND, PWM, POT |
| J-TOF (×3 per side) | Side board → ToF | JST-XH 6-pin | 3V3, GND, SDA, SCL, INT, XSHUT |
| J-SWD (per MCU) | Debug | Tag-Connect TC2030 or 1.27 mm 2×5 *TBD* | SWDIO, SWCLK, NRST, 3V3, GND |
| J-SWDREC | Carrier → any MCU | 1.27 mm 2×5 | CM5 GPIO SWD recovery |
| J-ELRS | Carrier → receiver | JST-GH 4-pin | 5 V, GND, TX, RX |
| J-LIDAR | Carrier → LD19 | JST-GH 4-pin | 5 V, GND, TX, RX |
| J-GIMBAL (×2) | Carrier → SG90 | 2.54 mm 3-pin | 5 V, GND, PWM |
| J-NOSE | Carrier → nose board | JST-GH 8-pin | see below |
| J-CAM | Carrier → nose board camera | Pi camera FFC (22-pin on the carrier) | CSI, camera screwed to the nose board |

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
| 4 | ESTOP_N: pulled up on every leg cell, only ever driven low (by the power board's e-stop buffer) |

## J-SYSCTL pinout

| Pin | Signal | Direction | CM5 side |
|---|---|---|---|
| 1 | GND | | |
| 2 | SHUTDOWN_REQ_N | power → CM5 | `gpio-shutdown` overlay: low = shut down |
| 3 | HALTED | CM5 → power | `gpio-poweroff` overlay: high once halted |
| 4 | ESTOP_N | power → CM5 | input, so ROS sees the hardware stop |
| 5 | 5V_PG | power → CM5 | 5 V rail good, optional |
| 6 | spare | | |

## J-NOSE pinout

| Pin | Signal | Nose board side |
|---|---|---|
| 1 | 3V3 | VL53L8CX AVDD, 1.8 V LDO in, compass |
| 2 | GND | |
| 3 | SDA | 3.3 V side of the PCA9306; the compass sits on this side |
| 4 | SCL | as SDA |
| 5 | TOF_LPN | CM5 GPIO → BSS138 → LPn (sensor restart) |
| 6 | TOF_INT | INT → BSS138 → CM5 GPIO |
| 7 | MAG_INT | compass data ready, optional |
| 8 | LED | status LED, optional |

Keep the cable under ~30 cm for 1 MHz I2C.

## J-SERVO pinout

| Pin | Signal | Wire |
|---|---|---|
| 1 | GND | brown |
| 2 | V+ 6.0 V | red |
| 3 | PWM | orange |
| 4 | POT wiper | added wire (pot mod) |

## J-TOF pinout

JST-XH 6-pin (board: B6B-XH-A). The pin order matches the VL53L1X breakout in use.

| Pin | Signal | Side board end |
|---|---|---|
| 1 | 3V3 | 100 nF at the connector |
| 2 | GND | |
| 3 | SDA | 2.2k pull-up to 3V3, DNP if the breakout has its own |
| 4 | SCL | 2.2k pull-up to 3V3, DNP if the breakout has its own |
| 5 | INT (GPIO1) | MCU GPIO input, 10k pull-up to 3V3 |
| 6 | XSHUT | MCU GPIO output |

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
