# CAN protocol

`vector.dbc` defines every message on the leg bus. Classic CAN, 1 Mbit/s, 11-bit IDs.

`ID = (function << 4) | node`. Nodes: 1–3 = L1–L3, 4–6 = R1–R3, 7 = power board, 0 = CM5 / broadcast.

| Function | Message | Direction | Rate |
|---|---|---|---|
| 0 | SYNC (counter, mode, e-stop) | CM5 → all | 200 Hz |
| 1 | LEG_CMD (3 targets, enable) | CM5 → leg | 200 Hz |
| 2 | LEG_STATE (3 positions, current) | leg → CM5 | 200 Hz |
| 3 | LEG_STATUS (ToF, VBAT, 6 V rail, temp, faults, state) | leg → CM5 | 50 Hz |
| 4 | POWER_STATE (pack V/I, SoC, temp, state) | power → CM5 | 10 Hz |
| 5 | LEG_CONFIG (joint, key, value, seq, op) | CM5 → leg | on demand |
| 6 | LEG_REPLY (joint, key, value, seq, status) | leg → CM5 | on demand |
| 7 | reserved for the bootloader | | |

Units: 0.01°, mV, mA, mm. VBAT and the 6 V rail travel in 10 mV steps.

Leg state: 0 off, 1 wake, 2 active, 3 crouch, 4 calibrate, 5 fault. Fault bits: watchdog, e-stop, wake failed, uncalibrated, buck, overtemp, calibration failed, config.

Config ops: 0 read, 1 write, 2 save to flash, 3 calibrate (value 0 full, 1 limits; joint 255 = all three), 4 defaults. Keys, per joint, integer on the wire:

| Key | | Scale |
|---|---|---|
| 1 | wiper mid, mV at 1500 µs | ×10 |
| 2 | wiper slope, mV/µs | ×10⁴ |
| 3 | centre, µs at joint angle 0 | ×10 |
| 4 | direction | ±1 |
| 5 | µs per degree | ×1000 |
| 6, 7 | min / max angle, deg | ×100 |
| 8 | fit error, mV (read only) | ×10 |
| 9 | calibrated (read only) | 0/1 |

Reply status: 0 ok, 1 bad key, 2 bad value, 3 busy, 4 calibration result (one per key: mid, slope, fit error), 5 calibration failed, 6 calibration done. `leg_config.py` in `vector_hw` wraps all of this.

```sh
cantools generate_c_source vector.dbc -o ../firmware/common/
candump can0 | cantools decode vector.dbc
```
