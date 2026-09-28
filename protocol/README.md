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
| 7 | BOOT (op, payload) | CM5 → leg | during an update |
| 8 | BOOT_REPLY (op, status, arg) | leg → CM5 | during an update |

Units: 0.01°, mV, mA, mm. VBAT and the 6 V rail travel in 10 mV steps. ToF: 0 = no sensor or no fresh reading, 65535 = nothing in range.

Leg state: 0 off, 1 wake, 2 active, 3 crouch, 4 calibrate, 5 fault, 6 self-test. Fault bits: watchdog, e-stop, wake failed, uncalibrated, buck (power-good lost), overload (stall current or hot board), calibration failed, config.

Config ops: 0 read, 1 write, 2 save to flash, 3 calibrate (value 0 full, 1 limits; joint 255 = all three), 4 defaults, 5 self-test. Keys, per joint (0–2) or for the whole leg (joint 255), integer on the wire:

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
| 16 | leg: firmware version, short git hash, bit 28 = built with local changes (read only) | |
| 17 | leg: last reset, bits power, pin, watchdog, software, other (read only) | |
| 18 | leg: CAN bus-off count << 16, dropped tx frames (read only) | |
| 19 | leg: uptime, s (read only) | |
| 20, 21 | leg: VBAT / 6V0 divider correction, 0.8–1.2 | ×10⁴ |
| 22 | leg: current amp zero, mA, measured while off (read only) | |

Joint writes, save and defaults answer busy unless the leg is off or faulted (servos unpowered); the leg-wide corrections can be written any time. An e-stop (pin or SYNC flag) also ends a calibration or self-test, which then replies done, as failed.

Reply status: 0 ok, 1 bad key, 2 bad value, 3 busy, 4 calibration result (one per key: mid, slope, fit error), 5 calibration failed, 6 calibration done, 7 test pass, 8 test fail, 9 test done (value = failures).

Self-test (leg off; about 350 ms): buck off, then on for 200 ms with no pulses. One pass/fail reply per item, key = item, value = reading: 1 current zero (mA, < 100), 2 6V0 with the buck off (mV, < 500), 3 VBAT (mV, 9–26 V), 4 temperature (°C, 0–60), 5 power good, 6 6V0 with the buck on (mV, 5.7–6.3 V), 7 current with the buck on (mA, < 1500), 8 ToF (mm, not 0), 9–11 wipers (mV, 100–3200: a servo is plugged in). `leg_config.py` in `vector_hw` wraps all of this.

Bootloader ops (byte 0; the reply echoes it with bit 7 set, byte 1 is the status, bytes 2–5 a little-endian argument):

| Op | | Payload | Reply argument |
|---|---|---|---|
| 1 | ENTER | `boot` | version, bit 8 image valid, bits 16+ space in KB |
| 2 | INFO | – | as ENTER |
| 3 | ERASE | image size, u32 | bytes erased |
| 4 | DATA | seq, up to 6 bytes | next seq, bytes written << 8 |
| 5 | DONE | CRC32 (zlib) of the image padded to 8 bytes with 0xFF | bytes written |
| 6 | RUN | – | – |

Status: 0 ok, 1 bad op, 2 bad sequence, 3 flash error, 4 bad CRC, 5 too big, 6 no image, 7 busy (leg powered). A repeated DATA seq is acked again without writing, for when an ack got lost.

```sh
cantools generate_c_source vector.dbc -o ../firmware/common/
candump can0 | cantools decode vector.dbc
```
