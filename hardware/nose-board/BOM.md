# Nose board BOM

Small 2-layer board in the nose cone: front ToF, compass, and the Pi camera screwed on next to the ToF. Design notes in `docs/spec.md` (Nose board), connector in `docs/interfaces.md` (J-NOSE). Prices single-unit, rounded.

| Qty | Part | Value / MPN | Package | ~€ | Notes |
|---|---|---|---|---|---|
| 1 | Front ToF | VL53L8CX | optical LGA16, 6.4 × 3.0 mm | 7.00 | keep the lens film on while soldering; thermal pad to GND with vias |
| 1 | 1.8 V LDO | TLV75518PDBVR (or AP2112K-1.8) | SOT-23-5 | 0.40 | CORE_1V8 + IOVDD, 50–80 mA while ranging |
| 1 | I2C level shifter | PCA9306 | VSSOP-8 | 0.60 | 3.3 V carrier ↔ 1.8 V sensor |
| 2 | Level shift FETs | BSS138 | SOT-23 | 0.20 | LPn (in), INT (out); gate on 1.8 V, 10k to 3.3 V on the carrier side |
| 1 | PCA9306 EN | 200k | 0603 | – | VREF2 and EN to 3.3 V (TI app circuit) |
| 2 | I2C pull-ups, sensor side | 2.2k | 0603 | – | to 1.8 V |
| 2 | I2C pull-ups, carrier side | 4.7k | 0603 | – | to 3.3 V, DNP if the carrier has them |
| 5 | Straps | 47k | 0603 | – | SPI_I2C_N, NCS to GND; INT, LPn, SYNC to 1.8 V |
| 6 | ToF decoupling | 4.7 µF + 100 nF each on AVDD, IOVDD, CORE_1V8 | 0805/0603 | 0.10 | at the pins |
| 2 | LDO caps | 1 µF in, 1 µF out | 0603 | 0.02 | |
| 1 | Compass | MMC5983MA | LGA-16 3×3 | 3.00 | 3.3 V, I2C 0x30; SPI_CS to 3.3 V, SDO open; keep copper and screws around it non-magnetic |
| 3 | Compass passives | 1 µF VDD, 10 µF CAP, 100k INT pull-down | 0603/0805 | 0.05 | INT is hi-Z until enabled |
| 1 | Status LED + resistor | any 0603 | 0603 | 0.05 | |
| 1 | Connector | SM08B-GHS-TB (JST-GH 8) | SMD | 0.60 | J-NOSE |
| 4 | Camera mount | M2 holes, Pi camera module pattern | PCB | – | camera axis parallel to the ToF |

About **€12** in parts, plus the PCB.
