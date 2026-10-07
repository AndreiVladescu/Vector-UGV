# Interfaces (board-to-board connectors & harnesses)

Draft. Freeze everything marked TBD before starting schematics.
Both ends of a cable use the same footprint and pin order.

## Overview

| ID | From → To | Connector | Carries |
|---|---|---|---|
| J-BAT | Pack → Power board | XT60 | VBAT (≈20 A peak) |
| J-BAL | Pack → Power board | JST-XH 5-pin | Cell taps C0–C4 |
| J-NTC | Pack → Power board | JST-PH 2-pin | cell NTC (DIY pack only) |
| J-USBC | Panel → Power board | USB-C receptacle (on board or panel pigtail *TBD*) | PD charge input, 20 V, the only charge input |
| J-PWRBTN | Panel → Power board | JST-PH 2-pin | momentary power button |
| J-LEGPWR-L / -R | Power board → Side board | XT30 (PCB: XT30PW-M) | VBAT (~13 A peak each) |
| J-SYSPWR | Power board → Carrier | XT30 (PCB: XT30PW-M) | VBAT; the carrier makes 5 V itself |
| J-USBDEV | Carrier → laptop | USB-C receptacle | CM5 USB 2.0 for rpiboot, data only |
| J-SYSCTL | Power board ↔ Carrier | JST-XH 6-pin | see below |
| J-CAN (×n) | Bus hops | JST-XH 4-pin | CANH, CANL, GND, ESTOP_N / spare |
| J-SERVO (×9 per side) | Side board → servo | 2.54 mm header 1×04, three per leg as a 3×4 block | GND, V+ (6 V), PWM, POT |
| J-TOF (×3 per side) | Side board → ToF | 2.54 mm header 1×04 | 3V3, GND, SDA, SCL |
| J-SWD (per MCU) | Debug | side board: 1 mm pads for PCBite probes (SWDIO, SWCLK, NRST) plus one shared 2.54 mm 1×02 (3V3, GND) per board; carrier and power board: TC2050 | SWDIO, SWCLK, NRST, 3V3, GND |
| J-SWDREC | Carrier → any MCU | 1.27 mm 2×5 | CM5 GPIO SWD recovery |
| J-ELRS | Carrier → receiver | JST-GH 4-pin | 5 V, GND, TX, RX |
| J-LIDAR | Carrier (IO MCU) → LDS01RR lidar | JST-XH 4-pin | 1 TX (lidar → MCU, 3.3 V), 2 RX (not used), 3 GND, 4 5 V; the lidar end is JST-PH 2.0 |
| J-LIDARMOT | Carrier → LDS01RR motor | JST-XH 2-pin | M+ (5 V), M− (low-side FET, PWM from the IO MCU) |
| J-LTE | Carrier → A7670E board | 1x07 2.54 mm | GND, RXD, TXD, PWRKEY, VCC, GND, SLEEP; the alternative to the soldered-down A7670E |
| J-CONSOLE | Carrier → USB-UART | 1x03 2.54 mm | GND, TX, RX (CM5 UART0) |
| J-NOSE | Carrier → nose board | JST-GH 8-pin | see below |
| J-CAM0, J-CAM1 | Carrier → cameras | Pi camera FFC (22-pin on the carrier) | CSI; CAM0 is the camera screwed to the nose board, CAM1 is spare |

Board-to-board cables are JST-XH (2.5 mm, horizontal headers). JST-GH stays only where the other end dictates it (ELRS receiver) and on J-NOSE.

## CAN bus

```
[120Ω] L3 ─ L2 ─ L1 ── J-CAN ── CARRIER (pass-through) ── J-CAN ── R1 ─ R2 ─ R3 [120Ω]
                                     │ stub < 30 cm
                                POWER BOARD
```

J-CAN pinout (JST-XH 4-pin):

| Pin | Signal |
|---|---|
| 1 | CANH |
| 2 | CANL |
| 3 | GND |
| 4 | ESTOP_N: pulled up on every leg cell, only ever driven low (by the power MCU's open-drain pin); a diode on each leg cell holds its servo buck's EN low while it's low |

## J-SYSCTL pinout

| Pin | Signal | Direction | CM5 side |
|---|---|---|---|
| 1 | GND | | |
| 2 | SHUTDOWN_REQ_N | power → CM5 | `gpio-shutdown` overlay: low = shut down |
| 3 | HALTED | CM5 → power | `gpio-poweroff` overlay: high once halted |
| 4 | ESTOP_N | power → CM5 | input, so ROS sees the hardware stop |
| 5 | 5V_EN | power → carrier | the carrier's 5 V buck EN: pulled up on the carrier, pulled low by the power board to keep the CM5 off |
| 6 | 5V_PG | carrier → power | the 5 V buck's PG |

J-SYSCTL and the power board's J-CAN may merge into one 10-pin JST-XH (one cable to the carrier); not decided.

## J-NOSE pinout

The same numbering on the carrier and on the nose board's J1, joined by a straight 1:1 JST-GH cable.

| Pin | Signal | Nose board side |
|---|---|---|
| 1 | LED | status LED, optional; not connected on the carrier |
| 2 | MAG_INT | compass data ready, optional |
| 3 | TOF_INT | INT → BSS138 → CM5 GPIO |
| 4 | TOF_LPN | CM5 GPIO → BSS138 → LPn (sensor restart) |
| 5 | SCL | 3.3 V side of the PCA9306; the compass sits on this side |
| 6 | SDA | as SCL |
| 7 | GND | |
| 8 | 3V3 | VL53L8CX AVDD, 1.8 V LDO in, compass |

The nose board has two footprints on the same pads: J1 on the back (the one fitted) and J2 on the front, for a cable from the front instead. Only one is ever fitted. J2 is the same part flipped, so its pin 1 sits where J1's pin 8 is: it needs a reversed cable (pin 1 ↔ 8), never the straight one.

Keep the cable under ~30 cm for 1 MHz I2C.

## J-SERVO pinout

| Pin | Signal | Wire |
|---|---|---|
| 1 | GND | brown |
| 2 | V+ 6.0 V | red |
| 3 | PWM | orange |
| 4 | POT wiper | added wire (pot mod) |

## J-TOF pinout

2.54 mm header 1×04 (footprint `vector:PinHeader_1x04_P2.54mm_Vertical_Tile`), a 4-way Dupont housing on the lead, glued once it works. The pin order matches the VL53L1X breakout in use; its own pull-ups hold XSHUT high, and the firmware polls instead of using INT. PB4 and PB5 on the MCU are spare.

| Pin | Signal | Side board end |
|---|---|---|
| 1 | 3V3 | 100 nF at the connector |
| 2 | GND | |
| 3 | SDA | 2.2k pull-up to 3V3, DNP if the breakout has its own |
| 4 | SCL | 2.2k pull-up to 3V3, DNP if the breakout has its own |

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

## CM5 GPIO allocation

| GPIO | Use |
|---|---|
| 0, 1 | ID_SD, ID_SC: CAM1 I2C |
| 2, 3 | I2C1 SDA, SCL: IMU, J-NOSE |
| 4, 5 | UART2 TX, RX: IO MCU, 1 Mbaud (framing in `firmware/io-node/README.md`) |
| 6 | IMU_INT |
| 7 | GNSS_PPS (time pulse) |
| 8 | SWD recovery NRST |
| 9 | MAG_INT (J-NOSE); UART3 RX if UART3 is ever needed |
| 10, 11 | SWD recovery SWDIO, SWCLK |
| 12, 13 | UART4 TX, RX: LTE |
| 14, 15 | UART0 TX, RX: console |
| 16 | CAN_INT (MCP251863) |
| 17 | LTE_PWRKEY |
| 18, 19, 20, 21 | SPI1 CE0, MISO, MOSI, SCLK: MCP251863 (SPI0 shares pins with UART3) |
| 22 | SHUTDOWN_REQ_N (J-SYSCTL, in) |
| 23 | HALTED (J-SYSCTL, out) |
| 24 | ESTOP_N (J-SYSCTL, in) |
| 25 | TOF_INT (J-NOSE) |
| 26 | TOF_LPN (J-NOSE) |
| 27 | LTE_SLEEP |

ELRS, the lidar, the GNSS and the LoRa radio sit on the IO MCU. The nose LED on J-NOSE stays unconnected.

