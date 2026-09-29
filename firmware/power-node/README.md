# Power node

STM32C092 on the power board, CAN node 7. It sets up the BQ76942 (its settings live in RAM and are gone after every SHUTDOWN), runs the charger, keeps the state of charge, decides when the robot is on, and reports on CAN (`protocol/README.md`, power board section). The hot-swap controllers, the e-stop chain and the bucks work without it.

## States

| State | 5 V / legs | Leaves when |
|---|---|---|
| boot | as they were | BMS set up: button held or pack already on → on; charger in → charge; neither → off. BMS silent for 2 s → fault |
| charge | off (FETs on, so the charger reaches the cells) | button press → on; no charger for 5 s → off |
| on | on (sides per key 37) | long press (2 s), key 40, low battery (a cell under 3.3 V or SoC under 2 % for 10 s) → halting; CM5 halted on its own → off / charge |
| halting | on, SHUTDOWN_REQ_N low | CM5's halted line for 1 s, 30 s timeout, or a second long press → off / charge |
| off | off, FETs open, SoC saved | sends SHUTDOWN once the button is released (TS2 low would wake it straight back up); a charger keeps LD up and the BMS awake → charge |
| fault | off | retries the BMS every 5 s |

An MCU reset (watchdog, CAN update) finds the BMS configured and the FETs closed and carries on without touching them; the 5 V and side outputs are pulled to "on" in hardware, so the CM5 and the legs don't notice.

The gauge starts from the resting cell voltage (or the SoC saved at power-off if the two agree within 15 %), then follows the BQ76942's coulomb counter; "charge done" from the BQ25798 sets 100 %. Capacity is key 32 (default 8000 mAh). The charger is fed every second: if its 40 s watchdog ever runs out it falls back to the PROG defaults (4S, 1 A), and the next poll writes the settings back.

## What the board has to match

- **BQ7694202PFBR, not BQ76942PFBR.** The plain part boots with REG1 off, and REG1 is what enables the MCU's 3.3 V buck, so a fresh chip would never start the MCU. The 02 variant has REG1 on at 3.3 V and I2C CRC on (the driver does the CRC). Programming the plain part's OTP instead needs 10–12 V on BAT and an external 3.3 V for the MCU.
- Cells on VC1, VC2, VC3 and VC10 (TI: cells 1, 2 and 10 must be real), VC4–VC9 shorted to VC3. Vcell Mode 0x0207.
- Thermistors (10k, 18k pull-up model): TS1 and HDQ on the cells, TS3 at the FETs. TS2 stays free for the wake button and charger.
- 5 V buck EN pulled up, an N-FET from the MCU pulls it down. Same for each LM5069 UVLO. Gate pull-downs on all three, so a reset or the bootloader leaves power on.
- HALTED (J-SYSCTL pin 3) needs a pull-down: the CM5 pin floats until Linux halts.
- BQ25798: no NTC; TS is ignored in firmware (TS_IGNORE), the BMS watches the cell temperatures. VAC_OVP stays at 26 V: the other steps are 18, 12 and 7 V, all too low for a 20 V input.

## Pins (proposed, LQFP32)

| Pin | Function | |
|---|---|---|
| PA11, PA12 | FDCAN1 | TCAN332 |
| PB6, PB7 | I2C1 | BQ76942 (0x08) and BQ25798 (0x6B), 400 kHz, 2.2k pull-ups to 3V3 |
| PB0 | 5V_OFF | N-FET on the LM61460 EN |
| PB1, PB2 | SIDE_L_OFF, SIDE_R_OFF | N-FETs on the LM5069 UVLO pins |
| PA8 | RUN, open drain | pulls the e-stop line low |
| PA9 | SHUTDOWN_REQ_N, open drain | J-SYSCTL pin 2 |
| PA0 | BUTTON, pull-up | through a diode from the button |
| PA1 | HALTED | J-SYSCTL pin 3 |
| PA4 | RUN sense | low = e-stop |
| PA6 | 5V_PG | LM61460 PG |
| PA15 | LED | |
| PA13, PA14, NRST | SWD | TC2050 |

LED: on = running (a short blink off every second on low battery), slow blink = charging, steady while charging is done, fast blink = halting, flicker = fault.

## Build and test

```sh
cmake -S firmware/power-node -B build/power-host && cmake --build build/power-host && build/power-host/test_power
cmake -S firmware/power-node/stm32 -B build/power-stm32 [-DBOARD=nucleo] && cmake --build build/power-stm32
```

`test_power` runs the real logic against register-level models of both TI chips (CONFIG_UPDATE, checksums, CRC, FET control, CUV, coulomb counter, SHUTDOWN, the charger watchdog). The register addresses come from the BQ76942 TRM (SLUUBY1B) and the BQ25798 datasheet (SLUSDV2C); on the bench, key 39 reads any BMS setting back to check it.
