# Servos

18× MG996R, each with a wire soldered to the pot wiper (blue) for position feedback. Two batches: A1–B1 have the usual pot; B2 onward have a different, lower-resistance pot inside (about 30 Ω as measured), so expect their calibration to differ from the first batch.

## Names

Code and docs use L1–L3 / R1–R3 (1 front, 3 rear) and coxa / femur / tibia, e.g. `R1_coxa`.

The servos themselves carry printed labels A–F plus joint number (1 coxa, 2 femur, 3 tibia), going round the body:

| Label | Leg | Position |
|---|---|---|
| A | R1 | right front |
| B | R2 | right middle |
| C | R3 | right rear |
| D | L3 | left rear |
| E | L2 | left middle |
| F | L1 | left front |

So the servo labelled A1 is `R1_coxa`, C3 is `R3_tibia`, F2 is `L1_femur`.

## Calibration

Wiper voltage is linear in the pulse width: `wiper_mV = mid + slope × (pulse_us − 1500)`. On R1_coxa, 1000 µs of pulse is 90° of shaft rotation (0.09°/µs), so 0–180° is 500–2500 µs.

Measure with `test-tools/servo_test` on an Uno, wiper straight to A0, servo at 6.0 V: send `c R1_femur` and it sweeps 500–2500 µs and prints the row.

| Joint | Label | Date | mid (mV @ 1500 µs) | slope (mV/µs) | @ 500 µs | @ 2500 µs | fit error (mV) | noise p-p (mV) | Notes |
|---|---|---|---|---|---|---|---|---|---|
| R1_coxa | A1 | 2026-09-26 | 1646 | 1.366 | 269 | 3001 | < 15 | 190–260 | linear 400–2600 µs, 90° per 1000 µs by eye |
| R1_femur | A2 | 2026-09-26 | 1565 | 1.431 | 134 | 2996 | 20 | 620–800 | same noise with pulses stopped; about 3.5× R1_coxa |
| R1_tibia | A3 | 2026-09-26 | 1591 | 1.424 | 167 | 3015 | 16 | 390–570 | 41 mV rms, 97% above 160 Hz |
| R2_coxa | B1 | 2026-09-26 | 1593 | 1.374 | 219 | 2967 | 19 | 360–470 | 4 sweeps within 19 mV; parked past the top end it read 3.22 V |
| R2_femur | B2 | 2026-09-27 | 1522 | 1.465 | 57 | 2987 | 23 | | second batch (low-resistance pot); 3 sweeps within 24 mV; only ~60 mV at 500 µs, so the pot's low end is close |
| R2_tibia | B3 | 2026-09-27 | 1553 | 1.489 | 64 | 3042 | 21 | > 700 | second batch; 3 sweeps within 29 mV; noise taken parked at 500 µs where the wiper clips at 0 V, so the real p-p is higher |
| R3_coxa | C1 | 2026-09-27 | 1572 | 1.436 | 136 | 3007 | 19 | 570 | second batch; 3 sweeps within 20 mV; noise taken parked at ~3.0 V, not clipped |
| R3_femur | C2 | 2026-09-27 | 1573 | 1.427 | 146 | 3000 | 15 | 740–860 | second batch; 3 sweeps within 19 mV; noise parked at ~2.0 V over 7 readings |
| R3_tibia | C3 | 2026-09-27 | 1524 | 1.476 | 49 | 3000 | 17 | 750–890 | second batch; 3 sweeps within 25 mV; parked at ~2.98 V the spikes reach 3.34 V, above 3.3 V (filtered out on the leg board) |

Noise differs between servos but is high frequency: on R1_femur 45–48 mV rms and R1_tibia 41 mV rms, with sharp spikes, 96–97% of its power above 160 Hz (strongest around 1.7–4.3 kHz, possibly aliased by the Uno's 9.6 kHz sampling). The leg board's 10k + 100 nF (160 Hz) leaves about 9 mV rms (about 0.6°) before averaging.

## What R1_coxa (A1) showed

- The pot runs off the servo's internal regulator: same curve at 5 V and 6 V supply.
- Repeatable within 5–15 mV over six sweeps, no real hysteresis (up vs down within ±10 mV). A 12 µs (1°) step shows as a clean 10 mV change.
- Linear from 400 µs (132 mV) to 2600 µs (3143 mV), no end stop inside that: at least 198° of travel.
- Noise 190–260 mV p-p, the same holding and with pulses stopped, so it's the servo's electronics, not the motor. The mean is stable to 1 mV. Pushing on the horn barely changes the noise (220–225 mV) and shifts the mean 11 mV (about 0.7°): that commanded-vs-measured error is what contact sensing will use.
- Restart quirk: after the pulses stop, if the first new pulse is below where the servo sits, it ignores every command until sent past its held position. On R2_coxa a first pulse only ~30 µs (3°) low was enough, when its position was estimated with R1_coxa's calibration. So the leg firmware must resume at the position measured with that joint's own calibration, plus a small margin upward (~30 µs), then ramp.
- Parked beyond the calibrated range, R2_coxa's wiper read 3.22 V, the same as a constant reading when it was stuck there. That's most likely the pot's supply end, the most the wiper can reach, so it stays under the STM32's 3.3 V.
- Unpowered, wiper to GND reads 12 Ω at one end of travel and 2.687 kΩ at the other (in-circuit, so skewed by the servo board).

Consequence for the leg board: wiper through 10k + 100 nF straight into the STM32 ADC, no divider (see the spec, Position).
