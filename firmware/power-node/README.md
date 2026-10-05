# Power node

STM32C092 on the power board, CAN node 7. It sets up the BQ7694202 and the BQ25798 (their settings live in RAM and are gone after every power-off), keeps the state of charge, decides when the robot is on, switches the CM5's 5 V on the carrier, drives the e-stop line, and reports on CAN (`protocol/README.md`, power board section). Parts in `hardware/power-board/BOM.md`.

## States

| State | CM5 / legs | Leaves when |
|---|---|---|
| boot | as they were | BMS set up: button held or pack already on → on; USB-C in → charge; neither → off. BMS silent for 2 s → fault |
| charge | CM5 off; the side boards get VBAT but their servo bucks stay off | button press → on; no USB-C for 5 s → off |
| on | on | long press (2 s), key 40, low battery (a cell under 3.3 V or SoC under 2 % for 10 s) → halting; CM5 halted on its own → off / charge |
| halting | on, SHUTDOWN_REQ_N low | HALTED for 1 s, 30 s timeout, or a second long press → off / charge |
| off | FETs open, SoC saved | sends SHUTDOWN once the button is released (TS2 low would wake it straight back up); USB-C keeps the BMS awake → charge |
| fault | CM5 off | retries the BMS every 5 s |

An MCU reset (watchdog, CAN update) finds the BMS configured and the FETs closed and carries on without touching them; the CM5's 5V_EN is pulled on in hardware, so the CM5 doesn't notice.

E-stop: while ROS sets the e-stop flag in SYNC (on and halting), the MCU pulls ESTOP_N low, and a diode on every leg cell holds its servo buck off. A stale SYNC lets go: the legs crouch and power down on their own then.

The gauge starts from the resting cell voltage (or the SoC saved at power-off if the two agree within 15 %), then follows the BMS coulomb counter; "charge done" from the BQ25798 sets 100 %. Capacity is key 32 (default 8000 mAh). The charger is fed every second: if its 40 s watchdog runs out it falls back to the PROG defaults (4S, 1 A), and the next poll writes the settings back.

Cell temperature: with a thermistor on J-NTC (DIY pack) set key 37 to 1 and save; it applies at the next power-up, since reconfiguring the BMS opens the FETs. The default, 0, leaves TS1 unused and lets the BMS die temperature stand in: a drone pack has no thermistor, and an open TS1 reads as deep cold and trips the undertemperature protection.

## What the board has to match

- **BQ7694202PFBR, not BQ76942PFBR.** The plain part boots with REG1 off, and REG1 is what enables the MCU's 3.3 V buck. The 02 variant has REG1 on at 3.3 V and I2C CRC on (the driver does the CRC).
- The 3.3 V buck runs from the pack side of the FETs: the MCU has to be up before they close.
- Cells on VC1, VC2, VC3 and VC10 (TI: cells 1, 2 and 10 must be real), VC4–VC9 shorted to VC3. Vcell Mode 0x0207.
- Thermistors (10k, 18k pull-up model): TS1 to J-NTC, TS3 at the FETs. TS2 is for the wake button and the USB-C wake FET only. HDQ unused.
- BQ25798, single input: VAC1 and VAC2 tied to VBUS, ACDRV1/2 to GND. No NTC: TS gets a divider from REGN that reads as 25 °C, and the firmware sets TS_IGNORE. VAC_OVP stays at 26 V: the other steps are 18, 12 and 7 V.
- The carrier's 5 V buck EN is pulled up there; the 2N7002 on PB0 (100k gate pull-down) pulls it low.
- HALTED (J-SYSCTL pin 3) needs a 100k pull-down: the CM5 pin floats until Linux halts.

## Pins (LQFP32)

| Pin | Function | |
|---|---|---|
| PA11, PA12 | FDCAN1 | TCAN332 |
| PB6, PB7 | I2C1 | BQ7694202 (0x08) and BQ25798 (0x6B), 400 kHz, 2.2k pull-ups to 3V3 |
| PB0 | CM5_OFF | 2N7002 on the carrier's 5V_EN (J-SYSCTL pin 5) |
| PA8 | ESTOP_N, open drain | through 100 Ω to J-CAN pin 4 and J-SYSCTL pin 4; read back on the same pin |
| PA9 | SHUTDOWN_REQ_N, open drain | J-SYSCTL pin 2 |
| PA0 | BUTTON, pull-up | on the button; the button wakes TS2 through a BAT54J, so the USB-C wake FET never reads as a press |
| PA1 | HALTED | J-SYSCTL pin 3 |
| PA6 | 5V_PG | J-SYSCTL pin 6, the carrier buck's PG |
| PB1 | BMS_ALERT | wired, not used yet |
| PB2 | CHG_INT | wired, not used yet |
| PA15 | LED | |
| PA13, PA14, NRST | SWD | TC2050 |

LED: on = running (a short blink off every second on low battery), slow blink = charging, steady while charging is done, fast blink = halting, flicker = fault.

## Build and test

```sh
cmake -S firmware/power-node -B build/power-host && cmake --build build/power-host && build/power-host/test_power
cmake -S firmware/power-node/stm32 -B build/power-stm32 [-DBOARD=nucleo] && cmake --build build/power-stm32
```

On the Nucleo, `-DNUCLEO_CAN_PA11=ON` moves CAN to the power board's pins, PA11/PA12, wired to PD0/PD1 on the morpho header for the on-board transceiver.

`test_power` runs the real logic against register-level models of both TI chips (CONFIG_UPDATE, checksums, CRC, FET control, CUV, coulomb counter, SHUTDOWN, the charger watchdog). The register addresses come from the BQ76942 TRM (SLUUBY1B) and the BQ25798 datasheet (SLUSDV2C); on the bench, key 39 reads any BMS setting back to check it.
