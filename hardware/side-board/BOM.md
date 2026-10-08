# Side board BOM

Per board = 3 leg cells + shared parts. Small resistors and capacitors are 0402; test points are 1 mm pads. Two boards per robot. Prices are single-unit (DigiKey/Mouser, 2026-09), rounded; passives are rough.

## Per leg cell (×3 per board)

| Qty | Part | Value / MPN | Package | ~€ | Notes |
|---|---|---|---|---|---|
| 1 | MCU | STM32C092KCT6 | LQFP32 | 2.10 | |
| 1 | Crystal | KYX K2B400001210: 40 MHz, CL 12 pF, ±10 ppm, ESR ≤ 40 Ω | 2016 | 0.15 | at worst-case ESR its gm_crit (~1.7–2 mA/V) is over the C0's 1.5 mA/V start-up limit: check start-up on the first boards |
| 2 | Crystal load caps | 18 pF C0G, 2 × (12 − 3) pF | 0805 | 0.02 | |
| 2 | MCU decoupling | 100 nF + 4.7 µF on VDD/VDDA | 0402 / 0805 | 0.05 | |
| 1 | NRST cap | 100 nF | 0402 | 0.01 | |
| 1 | CAN transceiver | TCAN332DR | SOIC-8 | 2.15 | 3.3 V-only, which is what the board has (TJA1051T/3 needs 5 V) |
| 1 | Servo buck | TPS56A37RPAR | VQFN-HR 3×3 | 2.70 | 4.5–28 V in, 10 A, 32 V abs max |
| 1 | Buck inductor | SRP7050WA-4R7M (Bourns): 4.7 µH, Isat 13.3 A, Irms 10.5 A, 20.5 mΩ max | 7.9×7.3 mm | 1.20 | WEBENCH in `docs/`: 8.8 A peak at 8 A, 520 kHz. Not the Sunlord MWSA0804S: only ~20 % saturation margin over a three-servo stall |
| 2 | Buck input caps | 15 µF + 1 µF, 35 V X7R | 1206 / 0805 | 0.25 | after the cell fuse |
| 2 | Buck output caps | 15 µF 16 V X7R | 1206 | 0.25 | plus 100 pF feed-forward across the top feedback resistor |
| 1 | Bootstrap cap | 100 nF | 0402 | 0.01 | |
| 1 | Soft-start cap | 22 nF | 0402 | 0.01 | slows the inrush when a leg wakes |
| 2 | Buck feedback | 90.9k / 10k 1 % (6.0 V from 0.6 V) | 0402 | 0.02 | |
| 1 | EN pull-down | 100k | 0402 | 0.01 | R217: buck off while the MCU is in reset |
| 1 | E-stop diode | BAT54J | SOD-323F | 0.10 | anode BUCK_EN, cathode ESTOP_N: the hardware e-stop holds the servo buck off |
| 1 | BUCK_EN series | 1k | 0402 | 0.01 | between PB0 and BUCK_EN, so the diode wins over the MCU pin |
| 1 | PG pull-up | 100k | 0402 | 0.01 | R218 |
| 1 | Current amp | INA181A1IDBVR (×20) | SOT-23-6 | 0.40 | + 100 nF |
| 1 | Shunt | 10 mΩ 1 %, ≥ 1 W | 2512 | 0.30 | 7.5 A stall → 0.56 W |
| 3 | Pot series R | 10k | 0402 | 0.01 | wiper → 10k → ADC |
| 3 | Pot divider bottom | 10k, DNP | 0402 | – | no divider needed, see docs/servos.md |
| 3 | Pot RC | 0 Ω link, 100 nF at the ADC | 0402 | 0.02 | with the 10k series R: 160 Hz |
| 3 | PWM series | 220 Ω | 0402 | 0.01 | |
| 3 | PWM pull-down | 10k | 0402 | 0.01 | servo sees no pulses while the MCU boots |
| 3 | Servo connector | 2.54 mm pin header 1×04 (or one 3×4 header per leg) | TH, vertical | 0.05 | ~3 A/pin; GND, V+, PWM, POT: the servo's own plug on pins 1–3, the pot wire on a 1-pin Dupont on pin 4; glued |
| 1 | NTC | NCP18XH103F03RB | 0603 | 0.10 | next to the inductor |
| 1 | NTC pull-up | 10k 1 % | 0402 | 0.01 | |
| 2 | VBAT divider | 100k / 15k 1 % + 100 nF | 0402 | 0.03 | PA5 |
| 2 | 6V0 divider | 15k / 10k 1 % + 100 nF | 0402 | 0.03 | PA6 |
| 2 | LED + resistor | any 0805 LED, 1k | 0805 / 0402 | 0.05 | PA15 (green), 6V0 present (red) |
| 1 | ToF connector | 2.54 mm pin header 1×04 | TH, vertical | 0.05 | 3V3, GND, SDA, SCL |
| 2 | I2C pull-ups | 2.2k, DNP by default | 0402 | – | most VL53L1X breakouts have their own |
| 1 | Cell fuse | 5 A fast, Littelfuse 0466005.NR | 1206 | 0.30 | in the buck's VIN: a shorted leg can't take the side down; 32 V, 11 mΩ |
| – | Leg ID | 3 solder jumpers | PCB | – | |
| – | SWD | 1 mm pads: SWDIO, SWCLK (PA14, net BOOT0), NRST | PCB | – | for PCBite probes; 3V3 and GND from the shared J104 |

About **€10 per leg cell**, ICs included.

## Shared on each board

| Qty | Part | Value / MPN | Package | ~€ | Notes |
|---|---|---|---|---|---|
| 1 | Power in | XT30PW-M (Amass) | TH | 0.50 | not a Mouser part: LCSC/AliExpress |
| 1 | TVS | SMBJ18A | SMB | 0.30 | clamps ~29 V, under the buck's 32 V abs max |
| 1 | Bulk cap | 100 µF 35 V low-ESR (polymer or electrolytic) | SMD | 0.60 | at the XT30, before the three cells |
| 1 | 3.3 V buck | TPS62933DRLR | SOT-583 | 1.00 | 3.8–30 V in, 3 A; stays on in Sentinel |
| 1 | 3.3 V inductor | SRP4020TA-4R7M (Bourns): 4.7 µH, Isat 3.5 A, 105 mΩ | 4.4×4.2 mm | 1.00 | ~0.3 A load; same part as the power board. LCSC equivalents for a tenth of the price in `hardware/BOM_LCSC.csv` |
| 5 | 3.3 V caps | 10 µF 35 V + 100 nF in, 47 µF out, 100 nF boot, 33 nF SS | 0805/1206 | 0.30 | |
| 2 | 3.3 V feedback | 30.9k / 10k 1 % | 0402 | 0.02 | |
| 2 | CAN connectors | JST XH 4-pin S4B-XH-A-1 (in, out) | TH, horizontal | 0.40 | CANH, CANL, GND, ESTOP_N |
| 1 | CAN ESD | NUP2105LT1G | SOT-23 | 0.30 | both connectors are on the same bus |
| 1 | CAN termination | 120 Ω + solder jumper | 0402 | 0.01 | close the jumper only at the outer end of the bus |
| 1 | SWD power | 1×02 pin header, 2.54 mm (J104): 3V3, GND | TH | 0.05 | shared by the three cells, for jumper wires |
| 4 | Mounting holes | M3 | PCB | – | |

About **€7 shared**, so **~€37 per side board** in parts, **~€74 for both**, plus PCBs and stencil.

## Cheaper options

- **TCAN332DR** is the biggest single item after the MCU and buck. There's no cheaper drop-in 3.3 V-only transceiver on the same footprint that I'd trust more, so keep it; quantity breaks at 10+ help (six legs + carrier + power board).
- **TPS56A37RPAR** is also on LCSC from about €1.20 if you split the order.
- Passives and connectors: LCSC is usually half the Mouser price; ICs from Mouser in one order, as planned.
