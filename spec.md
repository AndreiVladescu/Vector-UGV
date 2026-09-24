# V.E.C.T.O.R. — Refined Specification (v0.7 draft)

**V**ersatile **E**dge **C**ompute & **T**errain **O**bservation **R**obot — an open, modular 18-DOF hexapod research platform with smart per-leg controllers on an RS-485 bus, edge-AI compute, an SDR payload and redundant links.

Target: PCBWay 9th Project Design Contest, category **Robotics** (secondary fit: Power / Edge AI).
Deadline: **2027-01-24**. Judging: Design 30% · Practical impact 30% · Documentation 25% · Popularity 15%.

Constraints: budget-conscious, **PCBs + stencil from PCBWay, assembled by hand**. Everything mechanical 3D printed (PETG structure, TPU feet) except servo horns and screws. Based in Romania (EU LTE bands). Hardware on hand: CM5 (Wi-Fi, 8 GB eMMC), Pi 5 + AI HAT+ 2, 18× MG996R, AD9363 SDR (2T2R), HackRF, Pi CSI camera, a modifiable USB camera, 18650 cells from e-scooter packs + nickel-strip spot welder, GX-12 connectors.

---

## 1. Positioning

- **Smart legs:** every leg has its own microcontroller, servo power supply, joint position (MG996R pot mod) and current sensing. Each leg also has a ToF sensor. The legs share one RS-485 bus; a failing leg is isolated, not fatal to the others.
- **Sentinel Stance:** the robot lowers its body to the ground and cuts leg power (servos at 0 W), then runs as a long-endurance stationary sensor node. The legs power back up one at a time, starting each servo at its measured angle so nothing jerks.
- **Payload:** optional M.2 AI accelerator, CSI camera on a pan/tilt gimbal, AD9363 SDR demo (receive-only).
- **Redundant links:** ExpressLRS, Wi-Fi, 4G LTE (video plus backup control) over a VPN. **2× GNSS + compass**, body IMU.
- **Real battery management:** BMS + charger with GX-12 DC and USB-C PD charging inputs.

## 2. Board set (3 unique designs, 4 boards)

```
   GX-12 DC ─┐   ┌─ USB-C PD                        TOP ANTENNA PLATE (printed)
             ▼   ▼                                  LTE×2 · GNSS×2 · Wi-Fi · ELRS
      ┌────────────────────────┐
      │ POWER & BMS BOARD      │◄─ 4S3P pack
      │ charge · BMS · e-stop  │                           ▲ U.FL pigtails
      └─┬──────────┬─────────┬─┘            ┌──────────────┴──────────────┐
  VBAT_L│    VBAT_R│    5V + VBAT_SYS ─────►│        CM5 CARRIER          │─ RJ45 ─ AD9363
        │          │                        │ CM5 · M.2 · LTE · 2× GNSS   │─ USB ── USB cam
        ▼          ▼                        │ RS-485 master · IMU · compass│─ CSI ── Pi cam
 ┌──────────────┐ ┌──────────────┐          └──────────────┬──────────────┘
 │ SIDE BOARD L │ │ SIDE BOARD R │                         │
 │ ┌──┐┌──┐┌──┐ │ │ ┌──┐┌──┐┌──┐ │   one RS-485 bus, daisy-chained:
 │ │L1││L2││L3│◄┼─┼─│R1││R2││R3│◄┼── carrier → side L → side R (+ power board)
 │ └──┘└──┘└──┘ │ │ └──┘└──┘└──┘ │   120 Ω at the carrier and at the end of side R
 └──────────────┘ └──────────────┘
   each "leg cell" = own MCU + RS-485 transceiver + 6 V buck + 3 servo ports + ToF port
```

### 2.1 Why a side board with 3 independent leg cells
- **Smart per leg.** Each leg cell is its own island on the PCB: its own MCU, bus transceiver, buck, fuse, servo and ToF connectors. Functionally that's 6 independent leg controllers.
- **The bus between the 3 legs of a side is PCB traces**, so it can't be cut and needs no extra connectors. Between boards there are two short cables inside the rigid body.
- **One PCB design for both sides:** rotated 180°, leg ID set by a resistor divider per cell.
- **One bus, not two.** If the bus dies the robot stops walking whatever the topology, so redundancy buys nothing. The answer is failing safe: each leg has a heartbeat watchdog, and on silence it does a controlled crouch, then powers its servos down.

### 2.2 Leg bus: RS-485 (CAN not required)
| Option | Verdict |
|---|---|
| **RS-485 half-duplex, 1 Mbaud (recommended)** | UART + a ~$0.10–0.20 transceiver. Works on any cheap MCU and needs no crystal (UART tolerates ~±2 %, and the internal oscillator is enough). The CM5 UART drives it directly (Linux kernel RS-485 mode toggles DE). This is how Dynamixel/Feetech smart servos work. You write the framing + CRC yourself (simple) |
| CAN 2.0 / CAN-FD | Hardware CRC, retransmit, arbitration, great Linux tooling (SocketCAN, candump). Needs an MCU with CAN, a crystal for in-spec timing, and an SPI CAN controller on the CM5. Still possible later: the STM32C092 is the CAN sibling of the recommended MCU |
| 10BASE-T1S | Also a linear bus, ~$3–5 per node, needs a bigger MCU. No gain here |
| Single-wire TTL half-duplex | No transceiver, but poor noise immunity next to 18 servos. No |
| I2C / SPI between boards | Not suited to cables in a noisy robot. No |

**Protocol: sync broadcast + time-slotted replies.**
1. Every cycle the master sends one broadcast frame with all 18 setpoints: ~42 B ≈ 420 µs at 1 Mbaud.
2. Each leg replies in its own time slot, ID × slot length after the broadcast: ~16 B each with 3 positions, current, ToF distance, status and CRC.
3. That's no polling round trips and ≈ 1.5 ms of bus time per cycle, so **200 Hz uses ~30 % of the bus.**
4. One more slot for the power board (BMS data).

The weakness vs CAN is a node stuck transmitting (its driver-enable line stuck on), which can block the bus. Mitigate by driving DE from the USART hardware (not GPIO) and adding a firmware guard.

About crystals and CAN (for the record): CAN's oscillator tolerance is a *ratio* (max ≈ 1.58 %) and doesn't get looser at low bitrates. A lower bitrate only makes it easier to configure the bit timing to reach that maximum. An internal RC oscillator at ±1 % works on the bench, but is out of spec at temperature extremes. Moot now that the bus is RS-485.

### 2.3 Leg cell MCU: STM32C031
| | CH32V003 | CH32V203 | **STM32C031 (recommended)** | STM32C092 |
|---|---|---|---|---|
| Price (low qty, approx.) | ~$0.15 | ~$0.60–1.00 | **~$0.40–0.70** | ~$0.80–1.20 |
| Core | RV32EC 48 MHz | RV32IMAC 144 MHz | Cortex-M0+ 48 MHz | Cortex-M0+ 48 MHz |
| Flash / RAM | 16 KB / 2 KB | 64 KB / 20 KB | 32 KB / 12 KB | up to 256 KB / 36 KB |
| ADC | 10-bit, 8 ch | 12-bit | 12-bit, 2.5 MSPS | 12-bit |
| RS-485 DE in hardware | no (GPIO in software) | no | **yes (USART DEM)** | yes |
| CAN | no | CAN 2.0B | no | **FDCAN** |
| Small packages | QFN20, TSSOP20 | QFN20 (with pin conflicts) | TSSOP20, **UFQFPN20/28** | TSSOP20, UFQFPN28/32 |
| External parts | caps | caps + crystal (for CAN) | **caps only** (internal 48 MHz oscillator) | caps (+ crystal for CAN) |
| Ecosystem | growing | growing | **STM32CubeIDE/CubeMX, Arduino, Zephyr, huge community** | same |

Why the STM32C031:
- It's the most popular ecosystem of the four.
- It needs only decoupling caps: no crystal, no regulator capacitor.
- It drives the RS-485 enable pin in hardware.
- It comes in small packages.
- The CAN upgrade path stays in the same family.

Package: **STM32C031G6U6 (UFQFPN28, 4 × 4 mm)** for pin headroom, or the F6P6 (TSSOP20) if the pin budget fits. Assign pins in CubeMX.

The CH32V003 remains the ultra-budget option. It works, but the pin budget is very tight with 8 ADC inputs + UART + I2C, and the DE line has to be toggled in software.

**Leg cell I/O budget (~23 pins):**
| Signal | Pins |
|---|---|
| PWM coxa / femur / tibia (timer channels) | 3 |
| ADC: 3× pot, leg current, VBAT, 6 V rail, NTC, leg ID divider | 8 |
| USART TX / RX / DE → RS-485 transceiver | 3 |
| I2C SDA/SCL + XSHUT + INT → ToF | 4 |
| SWD (SWDIO / SWCLK) | 2 |
| Buck EN, buck PGOOD, LED | 3 |

- **One current channel per leg**: per-joint load comes from the pot error (commanded vs measured angle).
- **Pot divider:** the MG996R pot is usually powered from the servo's supply (~6 V), so measure it; you'll probably need a ~10k/15k divider before the ADC.
- **Ground offset:** the servo return current shares the GND wire. Sample pots while it's quiet, or calibrate the offset.

### 2.4 Side board (×2, identical)
4-layer, ~200 × 50–60 mm (fit to the chassis side), 1 oz (2 oz if reasonably priced).

**Per leg cell (×3):**
| Function | Part (proposal) | Notes |
|---|---|---|
| MCU | STM32C031G6U6 | SWD pads per cell (Tag-Connect or 1.27 mm header) |
| RS-485 | THVD1450 / SP3485 / MAX3485-class, 3.3 V | Low-cost clones fine |
| Servo buck | **LM61495** (36 V in, 10 A) → 6.0 V, EN from the MCU | Per-leg power domain. 3 servos near stall ≈ 7.5 A |
| Current | 1× 10 mΩ shunt + INA181 → ADC | Power monitoring and stall protection |
| Position | 3× pot wiper → divider + RC → ADC | Pot mod on every servo |
| PWM | 3× timer channels, tri-stated before power-off | Prevents back-powering the servos through the signal pin |
| ToF port | JST-SH 6-pin: 3.3 V, GND, SDA, SCL, XSHUT, INT + pull-ups | **Each leg has its own I2C bus → no 0x29 address conflict** |
| Protection | Per-cell fuse / PTC on VBAT input | A shorted leg can't take down its neighbours |
| Connectors | 3× 4-pin latched (V+, GND, PWM, POT) | |
| Health | NTC at the buck, LED | |

**Shared on the board:** XT60 VBAT in, low-Iq 3.3 V buck for the logic (on in Sentinel), the RS-485 trunk as traces with in/out JST-GH connectors, a termination jumper, and VBAT sense.

### 2.4.1 Per-leg ToF sensors
**Sensor:** VL53L1X (popular, ~4 m indoors, 27° FoV, small ultra-lite driver that fits in 32 KB flash). Start with off-the-shelf breakouts, since the bare sensor is an LGA package that's hard to hand-solder.

**What they're good for (high value):**
- Terrain ahead of each foot: step, gap and obstacle detection before the foot lands.
- A 360° "virtual bumper" around the body.
- A replacement for the 3 fixed ToFs in the original idea.

**Swept ToF:** the pot feedback gives the exact pose of every sensor (forward kinematics). So each reading is a ray with known origin and direction. As the legs swing, the rays sweep the scene, and 6 sensors × 30–50 Hz build a local elevation/obstacle map.

**Limits for SLAM:**
- 6 single-point rays are too sparse for real SLAM.
- VL53L1X range drops sharply in direct sunlight.
- For real SLAM, add an optional 2D lidar (LDRobot LD19/LD06, ~€70–80, UART) on top, or later use camera-based depth. The carrier gets a spare UART header for it.

**Mounting:**
- **Coxa link (recommended):** it yaws with the coxa, so it sweeps horizontally while walking, and its cable crosses only the coxa joint (alongside the servo leads).
- **Knee (femur end):** better downward view of foot placement, but the I2C cable crosses the femur joint too. Keep it short (<20 cm), use a twisted pair, and run at 100–400 kHz.

### 2.5 CM5 carrier — compute + comms + navigation
4-layer, impedance-controlled.

| Block | Part (proposal) | Notes |
|---|---|---|
| Compute | CM5 (Wi-Fi, 8 GB eMMC, 4 GB RAM recommended), 2× DF40C-100 | |
| AI (optional) | M.2 M-key 2242, PCIe ×1 | Unpopulated at first; CPU fallback |
| Leg bus | CM5 UART + RS-485 transceiver (kernel RS-485 mode, DE via RTS), 120 Ω termination + failsafe bias resistors | 1 Mbaud master |
| Lidar (optional) | JST-GH UART header, 5 V | LD19/LD06 2D lidar for SLAM |
| 4G LTE | mPCIe (USB 2.0) + nano-SIM, Quectel EC25-EUX; 3.8 V/3 A buck | 2× U.FL → antenna plate |
| GNSS ×2 | u-blox MAX-M10S + Quectel LC76G, **active antennas** with a bias-tee each | U.FL → patches on the antenna plate. **SAW filter / good LNA**: LTE Band 3 uplink (1710–1785 MHz) sits close to GPS L1 |
| Compass | MMC5983MA **on the carrier's front edge** (the carrier sits at the front of the body) + a 4-pin I2C header for an external compass as plan B | See the placement rules in §2.7 |
| SDR | RJ45 MagJack → AD9363 over GbE | |
| USB | 2× USB-A 2.0 with TPS2553 current-limited switches | USB camera, flash drive |
| IMU | ICM-42688-P (SPI), near the centre of mass | |
| Gimbal | 2× servo headers, CM5 hardware PWM + 5 V/1.5 A buck | |
| ELRS | JST-GH UART (CRSF) | |
| Misc | USB-C device (rpiboot), fan, RTC battery, LEDs | |

Antennas: a printed top plate with SMA/U.FL mounts. Spread them out, with GNSS on top and furthest from the LTE antennas.

### 2.6 Power & BMS board
4-layer 2 oz (fallback 2-layer 2 oz).

| Block | Part (proposal) | Notes |
|---|---|---|
| MCU | STM32C031 (same as the legs) | Configures the BMS at boot, runs the charge/power state machine, reports in its RS-485 time slot |
| BMS | BQ76952 | High-side CHG/DSG FETs, cell OV/UV/OC/SC/temp, balancing, coulomb counting |
| Charger | BQ25798 | Dual-input mux: GX-12 DC (12–24 V) + USB-C PD (STUSB4500 / CH224K), up to 5 A charge, charge-only |
| Leg buses | 2× LM5069 + N-FET (left/right) | Soft start, current limit, hardware e-stop |
| Compute rail | 5 V / 6 A buck | Pi 5 during development, carrier later |
| Pack | XT60 + JST-XH balance + NTCs | |

### 2.7 Compass placement rules
The field from a wire is B = 2·10⁻⁷ · I / r: 10 A at 5 cm ≈ 40 µT, which is about Earth's field in Romania (~48 µT). What matters is distance from **changing** currents; fixed iron is removed by hard/soft-iron calibration.
- Place it at the front edge, ≥ 5 cm from buck inductors, the LTE module and its 3.8 V supply, and the CM5 fan.
- No high-current copper under it (keep the 5 V/VBAT paths on the rear half of the board).
- Twist every VBAT/GND pair in the body (the fields largely cancel). Keep battery and power board central/rear.
- Brass/nylon standoffs and screws near the sensor, no steel.
- Calibrate with the servos powered.
- **Sentinel Stance is when heading matters most** (pointing the camera or SDR), and the legs are off then, so the compass is at its cleanest. While walking, fuse it with the GNSS course and the IMU gyro.

## 3. Power & weight
- **Pack: 4S3P** e-scooter 18650s, matched by capacity/internal-resistance test, ~130 Wh, ~570 g.
  - Series links carry ~20 A peaks. A single 0.15 mm pure-nickel strip is too thin; use 0.2 mm pure nickel doubled, or nickel-plated copper.
  - Use pure nickel, not nickel-plated steel.
- Sentinel load 9–15 W → **~9–14 h**. Walking 50–90 W total.
- **Weight target ≤ 2.5 kg all-up.** Current estimate 2.4–2.6 kg; the frame is where to save mass.

## 4. Software stack
| Layer | Choice |
|---|---|
| Leg firmware | STM32C031, C (STM32CubeIDE + LL drivers). 1 kHz: PWM, ADC, soft-start, VL53L1X ULD driver, heartbeat watchdog → crouch → power off. Leg ID from the resistor divider |
| Bus protocol | RS-485 1 Mbaud: sync broadcast (all setpoints) + time-slotted replies, CRC-16; later a bootloader over the bus |
| Host OS | Raspberry Pi OS 64-bit |
| Middleware | ROS 2 Jazzy in Docker (same images on the Pi 5 and the CM5) |
| Locomotion | C++: IK, tripod/ripple/wave gaits, IMU leveling, contact-aware stepping; `ros2_control` hardware interface over the RS-485 UART; ToF rays → local elevation map |
| Perception | HailoRT (dev on the Pi 5 + AI HAT+ 2), CPU fallback |
| Nav | 2× GNSS arbitration/fusion + compass + IMU (robot_localization EKF) |
| Links | ELRS/CRSF; Wi-Fi + LTE over Tailscale; link manager with failsafe → Sentinel |
| Video / UI | MediaMTX WebRTC (software H.264), Foxglove / web dashboard |
| SDR demo | libiio/SoapySDR waterfall, ADS-B |

## 5. Development path
**Leg firmware dev kit (~€35):**
- 2× **NUCLEO-C031C6** (~€11 each). ST-LINK programmer/debugger on board, and it can also flash the final boards over SWD. Alternatively an ST-LINK V3MINIE.
- 3× 3.3 V RS-485 transceiver modules (MAX3485/SP3485 breakouts).
- 1× USB-RS485 adapter (FTDI or CH343 based) for the Pi 5.
- 1–2× VL53L1X breakout.
- INA181 breakout + 10 mΩ shunt.
- Tools: STM32CubeIDE + CubeMX (free). Alternatives: PlatformIO, Zephyr.

1. Pi 5 + AI HAT+ 2 + USB-RS485 adapter as the brain.
2. One side board on a bench PSU → full robot on the bench PSU.
3. Power board → battery.
4. CM5 carrier replaces the Pi 5.

## 6. Plan (~17 weeks, 2026-09-28 → 2027-01-24)
| Weeks | Milestone |
|---|---|
| 1–2 | Frame CAD + weight budget; pot-mod and characterize one MG996R; leg firmware on a Nucleo (PWM + pot + current + ToF + RS-485) with one modded servo; cell tests; side board schematic |
| 3 | **Side board rev A → fab.** Power board schematic. URDF + gait in sim |
| 4–5 | **Power board rev A → fab.** Carrier schematic. Print the frame |
| 6–7 | Side board bring-up; walking on the Pi 5 + bench PSU. **Carrier rev A → fab** |
| 8–9 | Power board bring-up; walking on battery |
| 10–11 | Carrier bring-up; rev B fixes → fab |
| 12–14 | Sentinel + wake-up, link failover, LTE/VPN, GNSS/compass, SDR + AI demos |
| 15–17 | Field tests, documentation, video, PCBWay page |

## 7. Open questions
- Chassis side length available for the side boards?
