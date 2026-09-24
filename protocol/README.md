# CAN protocol

`vector.dbc` defines every message on the leg bus. Classic CAN, 1 Mbit/s, 11-bit IDs.

`ID = (function << 4) | node`. Nodes: 1–3 = L1–L3, 4–6 = R1–R3, 7 = power board, 0 = CM5 / broadcast.

| Function | Message | Direction | Rate |
|---|---|---|---|
| 0 | SYNC (counter, mode, e-stop) | CM5 → all | 200 Hz |
| 1 | LEG_CMD (3 targets, enable) | CM5 → leg | 200 Hz |
| 2 | LEG_STATE (3 positions, current) | leg → CM5 | 200 Hz |
| 3 | LEG_STATUS (ToF, VBAT, 6 V rail, temp, faults) | leg → CM5 | 50 Hz |
| 4 | POWER_STATE (pack V/I, SoC, temp, state) | power → CM5 | 10 Hz |
| 7 | reserved for the bootloader | | |

Units: 0.01°, mV, mA, mm.

```sh
cantools generate_c_source vector.dbc -o ../firmware/common/
candump can0 | cantools decode vector.dbc
```
