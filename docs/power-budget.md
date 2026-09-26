# Power budget

Estimates for now. Replace them with measurements as they come in.

## Measurements to take first

- [ ] MG996R at 6.0 V: idle, holding under load, stall current
- [ ] One leg walking (3 servos): average and peak current
- [ ] Pot wiper voltage range and noise after the mod
- [ ] Cell capacity and internal resistance (every cell, for matching)

## Results so far

- 2026-09-26, first pot-modded MG996R, unpowered, output turned by hand with a horn: wiper (blue wire) to GND reads 12 Ω at one end of travel and 2.687 kΩ at the other end stop. The low end sits on the GND side of the pot. In-circuit reading, so the servo board's parallel paths skew it. 
- 2026-09-26, same servo powered at 5.0 V from a bench supply, wiper straight to Uno A0 (about 1 kΩ source impedance per the pull-up check), `servo_test` sweep in 55 µs steps, two runs:

  | Pulse | Wiper |
  |---|---|
  | 1000 µs | 0.958 V |
  | 1500 µs | 1.647 V |
  | 2000 µs | 2.322 V |

  Linear at 1.364 mV/µs (midpoint within 7 mV of a straight line), and the two runs agree within 1–2 Uno ADC counts (5 mV). Fits under 3.3 V at 5 V supply without a divider.
- Same servo at 6.0 V, three up and three down sweeps: same curve as at 5 V (960 / 1646 / ~2320 mV), so the pot runs off the servo's internal regulator and doesn't follow the supply. Runs agree within 5–15 mV, up vs down within ±10 mV (no real hysteresis). Stays linear to 2100 µs (2.46 V).
- Wiper noise at 6 V: 190–260 mV peak to peak, the same holding and with pulses stopped, so it's not the motor. Mean is stable to 1 mV. Needs an RC filter and averaging on the leg board.
- Restart quirk: after the pulses stop, if the first new pulse is well below where the servo is sitting (stopped at 1800–2000 µs, restarted at 1500), it ignores every command until sent past its held position. Restarting at the measured position (from the wiper) works every time. Tested 5 park positions.
- Real angle: 1000 → 2000 µs turns the horn 90° (seen by eye), so about 11.1 µs and 15.2 mV of wiper per degree. On the STM32's 12-bit / 3.3 V ADC that's about 19 counts per degree.
- Loaded (finger pushing on the horn at 1500 µs): noise 220–225 mV p-p vs 205–210 unloaded, and the mean drops 11 mV (about 0.7°): the servo yields a little before its loop pushes back. That commanded-vs-measured error is what contact and load sensing will use.
- Full range, stepped out in 50 µs steps: linear from 400 µs (132 mV) to 2600 µs (3143 mV), no end stop inside that, so at least 198° of travel. 0–180° real is 500–2500 µs (0.27–3.00 V). Noise at 600 µs was higher (352 mV).
- Alternating 2488 / 2500 µs (about 1°): 2991 / 3001 mV every time, so 1° is cleanly resolved even on the Uno's 10-bit ADC. Decision: no divider on the leg board (see spec, Position).
- Still to do: the other 17 servos (same calibration?).

## Rails

| Rail | Source | Loads | Avg | Peak | Notes |
|---|---|---|---|---|---|
| VBAT | 4S3P 18650 (12.0–16.8 V) | everything | | ~20 A | |
| 6V0_LEGn (×6) | TPS56A37 per leg cell | 3× MG996R | | ~7.5 A | Per-leg EN |
| 3V3_LEG (×2) | low-Iq buck per side board | 3× STM32C092, transceivers, ToF | | | On in Sentinel |
| 5V_SYS | Power board 5 V / 6 A | CM5, USB, SDR, gimbal | | 6 A | |
| 3V8_LTE | Carrier buck from VBAT_SYS | EC25-EUX | | 2 A bursts | |
| 3V3_M2 | Carrier | Hailo M.2 (optional) | | 3 A | |

## Mode totals

| Mode | Estimate | Measured |
|---|---|---|
| Walking (servos) | 40–70 W avg, >150 W peak | |
| Walking (compute + links + SDR) | 12–18 W | |
| Sentinel | 9–15 W | |
| Off / storage | < 1 mA target | |
