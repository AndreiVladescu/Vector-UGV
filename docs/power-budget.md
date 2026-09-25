# Power budget

Estimates for now. Replace them with measurements as they come in.

## Measurements to take first

- [ ] MG996R at 6.0 V: idle, holding under load, stall current
- [ ] One leg walking (3 servos): average and peak current
- [ ] Pot wiper voltage range and noise after the mod
- [ ] Cell capacity and internal resistance (every cell, for matching)

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
