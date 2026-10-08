# Power board BOM

One board: BMS, USB-C charger, power MCU on CAN. Everything from Mouser except the XT60/XT30 (Amass, TME) and the panel button. Prices single-unit, rounded; passives rough. Start each block from TI's reference design (BQ76952EVM for the BMS, BQ25798EVM for the charger).

Parts shared with the side boards (the NTC, BAT54J) use the same MPN. Passives here are 0603 resistors and 0805 capacitors; the side boards use 0402.

## Battery management

| Qty | Part | Value / MPN | Package | ~€ | Notes |
|---|---|---|---|---|---|
| 1 | BMS | BQ7694202PFBR | TQFP-48 | 4.50 | the 02 variant boots with REG1 on at 3.3 V, which starts the MCU; I2C with CRC. Cells on VC1, VC2, VC3, VC10, VC4–VC9 shorted to VC3 |
| 2 | Pack FETs (CHG, DSG) | BSC010N04LSATMA1 | SuperSO8 | 4.00 | high side, back to back; 20 A peak. Any 40 V, ≤ 5 mΩ part would do, these have margin |
| 1 | Pre-discharge FET + resistor | per the EVM, ~100 Ω 2 W | SOT-23, 2512 | 0.50 | the only inrush limit for the side boards' input caps: size it for them |
| 1 | Coulomb counter shunt | CSS2H-2512R-1L00F (Bourns, 1 mΩ 1 %) | 2512 | 0.50 | Kelvin to SRP/SRN through 100 Ω each, 100 nF across |
| 5 | Cell tap filters | 20 Ω + 100 nF | 0603 | 0.10 | |
| 1 | REG0 pass transistor | BCP56-16 | SOT-223 | 0.20 | REG1 alone gives 45 mA, not enough for CAN |
| 2 | NTC | NCP18XH103F03RB | 0603 / on cable | 0.20 | cell (J-NTC, on TS1) and FETs (TS3); never on TS2. Same part as the side boards |

## Charger (USB-C only)

| Qty | Part | Value / MPN | Package | ~€ | Notes |
|---|---|---|---|---|---|
| 1 | Charger | BQ25798RQMR | VQFN-HR 4×4 | 4.50 | single input: VAC1/VAC2 tied to VBUS, ACDRV1/2 to GND (datasheet 7.3.5.2), no input FETs. PROG 17.4k: 1.5 MHz, 4S |
| 1 | Inductor | XAL5030-102MEC (1 µH) | 5×5 mm | 0.80 | |
| 1 | PD sink | CYPD3177-24LQXQ (Infineon) | QFN-24 | 1.90 | asks for 20 V, set by resistors; I2C left open (its address 0x08 is the BMS's) |
| 1 | J-USBC | S6B-XH-A-1 (JST) | TH, horizontal | 0.25 | pigtail to the panel USB-C: VBUS ×2, CC1, CC2, GND ×2 (`interfaces.md`) |
| 1 | Panel USB-C | panel-mount USB-C receptacle, power only, with VBUS, GND, CC1 and CC2 brought out | panel | 3.00 | not a two-wire VBUS/GND socket: the CYPD3177 needs CC |
| 1 | VBUS TVS | SMBJ20A | SMB | 0.30 | |
| 1 | Charger wake | BSS138-7-F | SOT-23 | 0.10 | VBUS present pulls TS2 low |
| – | Caps | per the EVM; VBUS and PMID 10 µF 50 V (20 V PD input) | 0805 / 1206 | 1.00 | |

20 V × 3 A ≈ 60 W, about 2.5 h for the pack. A plain 5 V USB-C source still charges, at about 15 W.

## Power control

| Qty | Part | Value / MPN | Package | ~€ | Notes |
|---|---|---|---|---|---|
| 1 | Power button | momentary, panel | panel | 0.50 | straight to the MCU (PA0), and through the diode to BMS TS2 (wake) |
| 1 | Button diode | BAT54J | SOD-323F | 0.10 | anode on TS2: the USB-C wake FET doesn't read as a press; same part as the leg cells' e-stop diodes |
| 1 | CM5 power cut | 2N7002 + 100k gate pull-down | SOT-23 | 0.10 | pulls the carrier's 5V_EN low; the pull-down keeps the CM5 on through an MCU reset |
| 1 | ESTOP_N series resistor | 100 Ω | 0603 | 0.01 | MCU open-drain pin straight onto ESTOP_N; no buffer, no mushroom button |

## 3.3 V, MCU and CAN

| Qty | Part | Value / MPN | Package | ~€ | Notes |
|---|---|---|---|---|---|
| 1 | 3.3 V buck | TPS62933DRLR + the side boards' inductor and caps | SOT-583 | 1.50 | EN from BMS REG1 |
| 1 | MCU | STM32C092KCT6 | LQFP32 | 2.10 | node 7 |
| 1 | Crystal + load caps | K2B400001210 (KYX, 40 MHz, CL 12 pF) + 2× 18 pF C0G | 2016 | 0.20 | FDCAN needs it; as the side boards |
| 1 | CAN transceiver | TCAN332DR | SOIC-8 | 2.15 | stub off the bus, no termination |
| 1 | CAN ESD | NUP2105LT1G | SOT-23 | 0.15 | |
| 2 | I2C pull-ups | 2.2k | 0603 | 0.02 | BMS and charger share the bus |
| 1 | HALTED pull-down | 100k | 0603 | 0.01 | |
| 1 | LED + resistor | as the side boards | 0805 / 0603 | 0.05 | |
| – | SWD | TC2050 footprint | PCB | – | |

## Protection and connectors

| Qty | Part | Value / MPN | Package | ~€ | Notes |
|---|---|---|---|---|---|
| 1 | Pack TVS | SMBJ18A | SMB | 0.30 | after the pack FETs; same as the side boards (TPS56A37 is 32 V abs max) |
| 1 | Main fuse | Littelfuse ATO 30 A + FHAC inline holder | cable | 3.00 | in the pack lead |
| 1 | Bulk cap | 100 µF 35 V, the side boards' part | SMD | 0.60 | |
| 1 | Pack | XT60PW-M (Amass) | TH | 0.70 | TME; the pack keeps the hooded half |
| 1 | Balance | B5B-XH-A (JST) | TH | 0.20 | one 5-pin plug with all taps, as on drone packs: GND, C1, C2, C3, C4 |
| 1 | Cell NTC | B2B-PH-K-S (JST) | TH | 0.10 | DIY pack only; drone packs have none |
| 1 | Power button | B2B-PH-K-S (JST) | TH | 0.10 | |
| 3 | Sides, carrier | XT30PW-M (Amass) | TH | 1.50 | VBAT to both side boards and the carrier; TME |
| 1 | J-SYSCTL | S10B-XH-A-1 (JST) | TH, horizontal | 0.35 | CAN stub, ESTOP_N and the CM5 control lines to the carrier in one cable (`interfaces.md`) |

About **€35** in parts, plus the PCB (4 layers, 2 oz).
