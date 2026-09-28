# Power board BOM

One board. Prices are single-unit (Mouser unless noted, 2026-09), rounded; passives are rough. Nothing is drawn yet. Start each block from TI's reference design (BQ76952EVM for the BMS, BQ25798EVM for the charger, the LM5069 design calculator, WEBENCH for the 5 V buck).

## Battery management

| Qty | Part | Value / MPN | Package | ~€ | Notes |
|---|---|---|---|---|---|
| 1 | BMS | BQ76942PFBR | TQFP-48 | 3.50 | 3–10 cells; short each unused cell input to its neighbour (datasheet 10.1.2) and list the used ones in Vcell Mode |
| 2 | Pack FETs (CHG, DSG) | BSC010N04LS (Infineon, 40 V, 1.0 mΩ) | SuperSO8 | 3.00 | high side, back to back; 20 A peak |
| 1 | Pre-discharge FET + resistor | small P/N-FET per the EVM, ~100 Ω 2 W | SOT-23, 2512 | 0.50 | PDSG: charges the 5 V buck and carrier input caps before DSG closes |
| 1 | Coulomb counter shunt | CSS2H-2512R-1L00F (Bourns, 1 mΩ 1 % 5 W) | 2512 | 0.50 | Kelvin-routed to SRP/SRN through 100 Ω each, 100 nF across |
| 5 | Cell tap filters | 20 Ω + 100 nF per tap, per datasheet | 0603 | 0.10 | |
| 1 | REG0 pass transistor | BCP56-16 (Nexperia NPN, 80 V) | SOT-223 | 0.20 | feeds REG1 (45 mA max, so not enough for the CAN transceiver); diode in its collector per datasheet |
| 3 | NTC | NCP18XH103F03RB (10k B3380) | 0603 / on cable | 0.30 | 2 on the cells (J-NTC), 1 at the FETs, on TS1/TS3; never on TS2, it would stop SHUTDOWN (datasheet 13.5) |

The BQ76942 has no fuel-gauge algorithm, only a coulomb counter: the MCU works out the state of charge.

## Charger

| Qty | Part | Value / MPN | Package | ~€ | Notes |
|---|---|---|---|---|---|
| 1 | Charger | BQ25798RQMR | VQFN-HR 4×4 | 4.50 | buck-boost, 1–4S, 5 A max; PROG = 17.4k: 1.5 MHz, 4S, and a reset leaves 16.8 V / 1 A |
| 1 | Inductor | XAL5030-102MEC (Coilcraft, 1 µH, Isat 11 A) | 5×5 mm | 0.80 | 1 µH is the only value for 1.5 MHz (2.2 µH for 750 kHz) |
| 4 | Input FETs (ACFET/RBFET) | CSD17578Q3A (TI, 30 V, ~6 mΩ) | SON 3×3 | 2.00 | back to back, one pair per input, GX-12 and USB-C |
| 1 | USB-PD sink | CH224K | ESSOP-10 | 0.30 | LCSC, asks for 20 V; STUSB4500 (~€2.50) if it has to come from Mouser |
| 1 | USB-C receptacle | USB4125-GF-A (GCT, 6-pin power-only, 3 A) | SMD | 0.50 | CC lines to the CH224K |
| 2 | Input TVS | SMBJ20A | SMB | 0.60 | the BQ25798 is rated 30 V absolute: this limits the DC input to 20 V (a laptop brick); VAC_OVP set to 22 V in firmware |
| – | Input, charge, battery caps | per the EVM | 0805/1206 | 1.00 | |

A 20 V source at 3 A gives about 60 W; with a weaker PD source the BQ25798's input voltage regulation (VINDPM) backs the current off on its own.

## Hot-swap and e-stop

| Qty | Part | Value / MPN | Package | ~€ | Notes |
|---|---|---|---|---|---|
| 2 | Hot-swap controller | LM5069MM-2/NOPB (auto-retry) | VSSOP-10 | 5.00 | -1 latches off instead; one per side |
| 2 | Hot-swap FET | BSC010N04LS, if the LM5069 calculator's SOA check passes | SuperSO8 | 3.00 | it runs linear during soft-start; otherwise take a 40 V enhanced-SOA part the calculator suggests |
| 2 | Sense resistor | CSS2H-2512K-3L00F (Bourns, 3 mΩ 1 %) | 2512 | 1.00 | ~55 mV threshold: about 18 A per side |
| 1 | E-stop buffer | SN74LVC3G07DCUR (3× open-drain) | VSSOP-8 | 0.30 | RUN → ESTOP_N, UVLO_L, UVLO_R |
| 2 | Side cut | small N-FET or open-drain GPIO | SOT-23 | 0.20 | MCU cuts one side alone |
| 1 | E-stop button | latching mushroom, NC contact | panel | 5.00 | AliExpress |
| 1 | Power button | momentary | panel | 0.50 | to BMS TS2 (wake) and through a diode to the MCU |
| 1 | Charger wake | small N-FET | SOT-23 | 0.10 | either charge input present pulls TS2 low |

## Supplies

| Qty | Part | Value / MPN | Package | ~€ | Notes |
|---|---|---|---|---|---|
| 1 | 5 V / 6 A buck | LM61460AASQRJRRQ1 (auto mode) | VQFN-HR 14 | 3.00 | 36 V class: a 28 V part sits too close to the TVS clamp; auto mode for light load in Sentinel; 400 kHz |
| 1 | 5 V inductor | XEL6060-472MEC (Coilcraft, 4.7 µH, Isat 8.5 A) | 6×6 mm | 1.00 | the datasheet's 5 V / 400 kHz design: 4× 22 µF out, 2× 4.7 µF + 2× 100 nF in |
| – | 5 V caps | per WEBENCH | 1206 | 1.00 | |
| 1 | 3.3 V logic buck | TPS62933DRLR | SOT-583 | 1.00 | same as the side board; EN from BMS REG1 |
| 1 | 3.3 V inductor | XGL4020-472MEC (Coilcraft, 4.7 µH, Isat 3 A) | 4×4 mm | 0.40 | |

## MCU and CAN

| Qty | Part | Value / MPN | Package | ~€ | Notes |
|---|---|---|---|---|---|
| 1 | MCU | STM32C092KCT6 | LQFP32 | 2.10 | same as the legs: shared firmware and bootloader, node 7 |
| 1 | Crystal + load caps | ABM8-40.000MHZ-10-1-U-T (CL 10 pF) + 2× 15 pF C0G | 3225 | 0.45 | same part for the side boards |
| 1 | CAN transceiver | TCAN332DR | SOIC-8 | 2.15 | stub off the bus, no termination |
| 1 | CAN ESD | NUP2105LT1G | SOT-23 | 0.15 | |
| 2 | Sense dividers | 5 V rail, VBAT (independent of the BMS) | 0603 | 0.05 | |
| 1 | LED + resistor | | 0603 | 0.05 | |
| – | SWD | TC2050 footprint, like the side board | PCB | – | |

## Protection and connectors

| Qty | Part | Value / MPN | Package | ~€ | Notes |
|---|---|---|---|---|---|
| 1 | Pack TVS | SMBJ18A | SMB | 0.30 | after the pack FETs |
| 1 | Main fuse | 30 A MINI or ATO blade, inline holder on the pack lead | cable | 2.00 | a car parts shop; also protects the lead itself |
| 1 | Pack | XT60 | TH | 0.60 | LCSC |
| 1 | Balance | B5B-XH-A (JST) | TH | 0.10 | cell taps |
| 2 | Cell NTCs | B2B-PH-K-S (JST) | TH | 0.20 | |
| 2 | Sides | XT30PW-M | TH | 1.00 | LCSC |
| 1 | Carrier power | 43045-0612 (Molex Micro-Fit 3.0, 2×3) | TH | 1.50 | 5 V, VBAT_SYS, GND |
| 1 | Carrier control | SM06B-GHS-TB (JST) | SMD | 0.50 | J-SYSCTL |
| 1 | CAN | SM04B-GHS-TB (JST) | SMD | 0.50 | |
| 1 | DC charge | GX-12 2-pin | panel | – | on hand |
| 2 | Buttons | B2B-PH-K-S (JST) | TH | 0.20 | power, e-stop |

About **€50** in parts, plus the PCB (4 layers, 2 oz).
