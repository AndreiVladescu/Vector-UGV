# Interfaces (board-to-board connectors & harnesses)

Draft. Freeze everything marked TBD before starting schematics.
Both ends of a cable use the same footprint and pin order.

## Overview

| ID | From → To | Connector | Carries |
|---|---|---|---|
| J-BAT | Pack → Power board | XT60 | VBAT (≈20 A peak) |
| J-BAL | Pack → Power board | JST-XH 5-pin | Cell taps C0–C4 |
| J-NTC | Pack → Power board | JST-PH 2-pin | cell NTC (DIY pack only) |
| J-USBC | Panel → Power board | panel-mount USB-C receptacle with a pigtail to a JST-XH 6-pin on the board | PD charge input, 20 V, the only charge input; see below |
| J-PWRBTN | Panel → Power board | JST-PH 2-pin | momentary power button |
| J-LEGPWR-L / -R | Power board → Side board | XT30 (PCB: XT30PW-M) | VBAT (~13 A peak each) |
| J-SYSPWR | Power board → Carrier | XT30 (PCB: XT30PW-M) | VBAT; the carrier makes 5 V itself |
| J-USBDEV | Carrier → laptop | USB-C receptacle | CM5 USB 2.0 for rpiboot, data only |
| J-SYSCTL | Power board ↔ Carrier | JST-XH 10-pin | the power board's CAN stub and its control lines in one cable; see below |
| J-CAN (×2) | Carrier ↔ side boards | JST-XH 4-pin | CANH, CANL, GND, ESTOP_N |
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
                                     │ J-SYSCTL pins 1–4, stub < 30 cm
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

JST-XH 10-pin (board: S10B-XH-A-1, horizontal) on the power board and the carrier, straight 1:1 cable. Pins 1–4 are J-CAN's, in the same order: the power board's stub off the CAN bus, plus the e-stop line. Twist CANH with CANL.

| Pin | Signal | Direction | Notes |
|---|---|---|---|
| 1 | CANH | bus | |
| 2 | CANL | bus | |
| 3 | GND | | |
| 4 | ESTOP_N | power → all | open drain from the power MCU, pulled up on every leg cell; the carrier passes it to both J-CAN pin 4 and to CM5 GPIO24, so ROS sees the hardware stop |
| 5 | SHUTDOWN_REQ_N | power → CM5 | `gpio-shutdown` overlay: low = shut down |
| 6 | HALTED | CM5 → power | `gpio-poweroff` overlay: high once halted; 100k pull-down on the power board |
| 7 | 5V_EN | power → carrier | the carrier's 5 V buck EN: pulled up on the carrier, pulled low by the power board to keep the CM5 off |
| 8 | 5V_PG | carrier → power | the 5 V buck's PG |
| 9 | GND | | |
| 10 | spare | | not connected on either board |

## J-USBC pinout

A panel-mount USB-C receptacle (power only) with a short pigtail to a JST-XH 6-pin on the power board (S6B-XH-A-1). The CYPD3177 on the board needs both CC lines, so the panel part has to bring out VBUS, GND, CC1 and CC2; a two-wire (VBUS/GND only) USB-C socket can't negotiate 20 V. VBUS and GND get two pins each (3 A, the XH limit per pin).

| Pin | Signal |
|---|---|
| 1 | VBUS |
| 2 | VBUS |
| 3 | CC1 |
| 4 | CC2 |
| 5 | GND |
| 6 | GND |

Keep the pigtail short (≤ 15 cm). The VBUS TVS sits on the board at the connector.

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

## Leg ID jumpers

Each leg cell reads three solder jumpers to 3V3 as GPIOs with the internal pull-downs (bridged = 1), so the node ID comes from the board, not the firmware:

| Bit | MCU pin | Jumpers | Meaning |
|---|---|---|---|
| POS0, POS1 | PB3, PB8 | two per cell, set once on the root sheet | the cell's position: 01 = cell A (next to the CAN-in connector), 10 = B, 11 = C (outer end); 00 = not set |
| SIDE | PB9 | JP107, one per board, shared by its three cells | open = left board, bridged = right board |

The right board is the same PCB turned 180°, so its cell A sits at the other end of the body and the order runs backwards (`leg_node_from_straps()` in `firmware/leg-node/src/leg_id.h`):

| Board | Cell A | Cell B | Cell C |
|---|---|---|---|
| Left (JP107 open) | L1, node 0x1 | L2, node 0x2 | L3, node 0x3 |
| Right (JP107 bridged) | R3, node 0x6 | R2, node 0x5 | R1, node 0x4 |

The power board is node 0x7, fixed in its firmware. Position 00 keeps a leg off with a config fault; `leg_config.py read` shows the raw jumpers.

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

