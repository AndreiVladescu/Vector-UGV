# Side board BOM

Per board = 3 leg cells + shared parts. Two boards per robot. Prices are single-unit (DigiKey/Mouser, 2026-09), rounded; passives are rough.

Status: **sch** = in the schematic now, **add** = still to draw, **change** = in the schematic but needs a different value.

## Per leg cell (×3 per board)

| Qty | Part | Value / MPN | Package | ~€ | Status | Notes |
|---|---|---|---|---|---|---|
| 1 | MCU | STM32C092KCT6 | LQFP32 | 2.10 | sch | |
| 1 | Crystal | 40 MHz, CL 8–10 pF, ±20 ppm | 3225 | 0.40 | add | same part as the carrier's MCP251863; check drive level against the C0 HSE |
| 2 | Crystal load caps | per crystal CL (≈ 2×(CL − 3 pF)), C0G | 0402 | 0.02 | add | |
| 2 | MCU decoupling | 100 nF + 4.7 µF on VDD/VDDA | 0603 | 0.05 | change | schematic has 4× 100 nF; one of them becomes 4.7 µF |
| 1 | NRST cap | 100 nF | 0603 | 0.01 | add | |
| 1 | CAN transceiver | TCAN332DR | SOIC-8 | 2.15 | sch | 3.3 V-only, which is what the board has (TJA1051T/3 needs 5 V) |
| 1 | Servo buck | TPS56A37RPAR | VQFN-HR 3×3 | 2.70 | sch | 4.5–28 V in, 10 A, 32 V abs max |
| 1 | Buck inductor | ~2.2–3.3 µH, Isat ≥ 12 A, low DCR | 7×7 mm class | 1.00 | add | pick with the datasheet table / WEBENCH for 6.0 V out |
| 2 | Buck input caps | 10 µF 35 V X7R | 1206 | 0.25 | add | plus 100 nF next to VIN |
| 3 | Buck output caps | 22 µF 16 V X7R | 1206 | 0.25 | add | |
| 1 | Bootstrap cap | 100 nF | 0603 | 0.01 | add | |
| 1 | Soft-start cap | per datasheet (few nF) | 0603 | 0.01 | add | slows the inrush when a leg wakes |
| 2 | Buck feedback | 90.9k / 10k 1 % (6.0 V from 0.6 V) | 0603 | 0.02 | add | |
| 1 | EN pull-down | 100k | 0603 | 0.01 | sch | R217: buck off while the MCU is in reset |
| 1 | PG pull-up | 100k | 0603 | 0.01 | sch | R218 |
| 1 | Current amp | INA181A1IDBVR (×20) | SOT-23-6 | 0.40 | sch | + 100 nF |
| 1 | Shunt | 10 mΩ 1 %, ≥ 1 W | 2512 | 0.30 | add | 7.5 A stall → 0.56 W |
| 3 | Pot series R | 10k | 0603 | 0.01 | change | was 15k; wiper → 10k → ADC |
| 3 | Pot divider bottom | 10k | 0603 | – | change | mark DNP (no divider needed, see docs/servos.md) |
| 3 | Pot RC | 1k → 0 Ω link, 100 nF at the ADC | 0603 | 0.02 | change | gives 10k + 100 nF = 160 Hz |
| 3 | PWM series | 220 Ω | 0603 | 0.01 | sch | |
| 3 | PWM pull-down | 10k | 0603 | 0.01 | sch | servo sees no pulses while the MCU boots |
| 3 | Servo connector | JST XH 4-pin S4B-XH-A | TH, horizontal | 0.20 | sch | 3 A/pin; V+, GND, PWM, POT |
| 1 | NTC | NCP18XH103F03RB | 0603 | 0.10 | sch | next to the inductor |
| 1 | NTC pull-up | 10k 1 % | 0603 | 0.01 | add | |
| 2 | VBAT divider | 100k / 15k 1 % + 100 nF | 0603 | 0.03 | add | PA5 |
| 2 | 6V0 divider | 10k / 6.8k 1 % + 100 nF | 0603 | 0.03 | add | PA6 |
| 1 | LED + resistor | any 0603 LED, 1k | 0603 | 0.05 | add | PA15 |
| 1 | ToF connector | JST SH 6-pin SM06B-SRSS-TB | SMD | 0.50 | add | 3V3, GND, SDA, SCL, XSHUT, INT |
| 2 | I2C pull-ups | 4.7k, DNP by default | 0603 | – | add | most VL53L1X breakouts have their own |
| 1 | Cell fuse | 5 A fast, e.g. Littelfuse NANO2 0451005.MRL | 2410 | 0.60 | add | a shorted leg can't take the side down |
| – | Leg ID | 3 solder jumpers | PCB | – | sch | |
| – | SWD | Tag-Connect TC2030 footprint | PCB | – | add | SWDIO, SWCLK, NRST, 3V3, GND |

About **€10 per leg cell**, ICs included.

## Shared on each board

| Qty | Part | Value / MPN | Package | ~€ | Status | Notes |
|---|---|---|---|---|---|---|
| 1 | Power in | XT30PW-M (Amass) | TH | 0.50 | add | not a Mouser part: LCSC/AliExpress |
| 1 | TVS | SMBJ18A | SMB | 0.30 | sch | clamps ~29 V, under the buck's 32 V abs max |
| 1 | Bulk cap | 100 µF 35 V low-ESR (polymer or electrolytic) | SMD | 0.60 | add | at the XT30, before the three cells |
| 1 | 3.3 V buck | TPS62933DRLR | SOT-583 | 1.00 | sch | 3.8–30 V in, 3 A; stays on in Sentinel |
| 1 | 3.3 V inductor | ~4.7 µH, Isat ≥ 1.5 A | 4×4 mm class | 0.40 | add | |
| 4 | 3.3 V caps | 10 µF 35 V in, 2× 22 µF out, 100 nF boot | 0805/1206 | 0.30 | add | |
| 2 | 3.3 V feedback | per TPS62933 datasheet for 3.3 V | 0603 | 0.02 | add | |
| 2 | CAN connectors | JST GH 4-pin SM04B-GHS-TB (in, out) | SMD | 1.00 | add | CANH, CANL, GND, ESTOP_N |
| 2 | CAN ESD | NUP2105LT1G | SOT-23 | 0.30 | sch | one per connector |
| 1 | CAN termination | 120 Ω + solder jumper | 0603 | 0.01 | sch | fit only at the outer end of the bus |
| 4 | Mounting holes | M3 | PCB | – | add | |

About **€7 shared**, so **~€37 per side board** in parts, **~€74 for both**, plus PCBs and stencil.

## Cheaper options

- **TCAN332DR** is the biggest single item after the MCU and buck. There's no cheaper drop-in 3.3 V-only transceiver on the same footprint that I'd trust more, so keep it; quantity breaks at 10+ help (six legs + carrier + power board).
- **TPS56A37RPAR** is also on LCSC from about €1.20 if you split the order.
- Passives and connectors: LCSC is usually half the Mouser price; ICs from Mouser in one order, as planned.
