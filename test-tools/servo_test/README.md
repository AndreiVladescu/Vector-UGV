# servo_test

Arduino Uno sketch to put one MG996R at a set angle over serial and read the modded pot wiper, for measuring the wiper voltage range and noise.

## Wiring

| Servo | Goes to |
|---|---|
| red (V+) | bench supply 6.0 V, current limit ~1 A (not the Uno 5V pin, the servo pulls too much) |
| brown (GND) | supply GND and Uno GND |
| orange (signal) | D9 |
| blue (wiper) | A0 directly |

The MG996R's pot runs off its internal regulator, so the wiper stays around 1–2.5 V even at 6 V supply and can go straight to A0. The sketch reads it as is (`k 100`); use `k 200` if you add a 100k/100k divider.

On the first `a`/`u` after a stop the sketch starts the pulses at the position read from the wiper plus 30 µs, because these servos ignore commands if the first pulse is even slightly below where they sit. That needs the right calibration: `c` loads the new servo's own numbers, `m <mid> <slope×1000>` loads them from `docs/servos.md` (e.g. `m 1593 1374`). The default is R1_coxa's. If a servo ignores commands, send a pulse above where it sits (`u 2600`) and it wakes up.

## Use

Serial monitor at 115200, line ending "Newline".

```
a 90              go to 90 deg, prints pulse, ADC and wiper mV
u 1500            go to a pulse width directly
s 0 180 10 500    sweep 0-180 in 10 deg steps, 500 ms settle, prints CSV
c R1_femur        calibrate that servo (label A2): sweep 500-2500 us, fit a line, print a row for docs/servos.md
m 1593 1374       load a calibration (mid mV, slope x1000) from docs/servos.md
n                 wiper min/avg/max over 1 s (noise; push on the horn to load it)
d                 dump 256 raw wiper samples at full ADC speed (~9.6 kHz), for looking at the noise
o                 stop pulses, servo goes limp
w 500             move speed in us/s (500 = ~90 deg/s, default), w 0 jumps straight there
p                 measure the pulses on D9 (should be ~50 Hz, width as commanded)
f                 check A0 is really on the wiper (a floating A0 reads garbage)
v 4950            set the real 5V pin voltage (measure it) for accurate mV
k 100             divider ratio x100 (100 = wiper straight to A0, the default; 200 for 100k/100k)
```

Run `f` first: if it says FLOATING, the wiper reading means nothing (check the blue wire lands on A0). In the Serial Monitor set the line ending to Newline, or no command runs.

Pulses map 0-180 deg to 500-2500 us (`US_MIN`/`US_MAX` at the top of the sketch), which is 180 deg of real rotation on the first modded servo: it turns 90 deg per 1000 us and stays linear from 400 to 2600 us.

What to record in `docs/power-budget.md`: wiper mV at 0, 90 and 180 deg, the pot's top-end voltage, and the noise spread while holding still and while loaded.
