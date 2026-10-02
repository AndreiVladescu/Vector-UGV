# V.E.C.T.O.R. spec

Versatile Edge Compute & Terrain Observation Robot. An 18-DOF hexapod built as an open research platform: a smart controller per leg on CAN, a CM5 carrier, a camera and SDR payload, and ELRS / Wi-Fi / 4G links. Entry for the PCBWay 9th Project Design Contest (Robotics), deadline 2027-01-24.

The main idea is "Sentinel Stance". The robot walks to a spot, lowers its body to the ground and cuts power to all servos. It then sits there for hours as a low-power sensor node (camera, SDR, GNSS, 4G). To get up again it powers the legs one at a time, starting each servo at the angle it actually measures, so nothing jerks.

Constraints: low budget. PCBs and stencils come from PCBWay and I assemble them myself. All mechanical parts are printed (PETG, TPU feet) and designed in Fusion 360. ICs come from Mouser.

Already on hand: CM5 (Wi-Fi, 8 GB eMMC), Pi 5 + AI HAT+ 2, 18× MG996R, AD9363 SDR (2T2R), HackRF, Pi CSI camera, a USB camera, 18650 cells from e-scooter packs, a spot welder, GX-12 connectors, a CANable, MCP2515 modules, 2 CH32V003 dev boards.

## Boards

Four designs, five boards:

| Board | Qty | Layers | Main parts |
|---|---|---|---|
| Side board | 2 | 4 | 3 leg cells, each with STM32C092, TCAN332DR, TPS56A37 6 V buck |
| Power & BMS | 1 | 4, 2 oz | BQ7694202, BQ25798 (USB-C), STM32C092 |
| CM5 carrier | 1 | 4, impedance-controlled | CM5, 5 V buck (TPS56A37), M.2, EC25-EUX LTE, 2× GNSS, MCP251863, IMU |
| Nose board | 1 | 2 | VL53L8CX front ToF, MMC5983MA compass, Pi camera mount |

```
[120Ω] L3 ─ L2 ─ L1 ── CM5 carrier ── R1 ─ R2 ─ R3 [120Ω]      one CAN bus, 1 Mbit/s
       └ side board L ┘      │         └ side board R ┘
                       power board (stub)

power board ──VBAT──► side L, side R, carrier (3× XT30; the carrier makes its own 5 V)
carrier ──J-NOSE (I2C, 3V3) + CSI ribbon──► nose board
```

### Side board (×2)

Both sides use the same PCB, rotated 180°. Each side board carries three independent leg cells, and the CAN bus between those cells is just traces. A leg cell has:

- **MCU:** STM32C092KCT6 (LQFP32) with a 40 MHz crystal for FDCAN (the same part as the carrier's MCP251863); the core runs from the internal 48 MHz. It has FDCAN in hardware and a factory CAN bootloader. Pin map in `firmware/leg-node/README.md`.
- **CAN transceiver:** TCAN332DR (SOIC-8), a 3.3 V-only part.
- **Servo buck:** TPS56A37RPAR (28 V in, 10 A, 3×3 mm HotRod QFN) set to 6.0 V, switched by the MCU through EN. Three servos near stall draw about 7.5 A.
- **Current sense:** one 10 mΩ shunt and an INA181 for the whole leg. The load on each joint comes from the pot error (commanded minus measured angle).
- **Position:** the MG996R pot wiper goes into the ADC through a series resistor and RC filter (about 10k + 100 nF). Measured 15.2 mV and 11.1 µs per degree, linear over 400–2600 µs (0.13–3.14 V, about 198°), the same at 5 V and 6 V supply. No divider: the joints stay within about ±80° (≤ 2.85 V); past 3.3 V the reading just saturates and the 10k limits clamp current to well under the STM32's injection limit. Firmware treats readings above ~3.2 V as "out of range", not an angle. Resolution checked: a 12 µs (1°) step shows as a clean 10 mV change. It carries ~200 mV of spikes from the servo's own electronics, so average many samples. Always restart the PWM at the measured position: this MG996R ignores commands if the first pulse after a stop is well below where it sits.
- **PWM:** the outputs are tri-stated before power-off, otherwise the servo gets back-powered through the signal pin.
- **ToF:** a VL53L1X on its own I2C bus, so there's no address clash (JST-XH 6-pin). The sensor sits on a breakout on the coxa link; pinout in `interfaces.md`.
- **Protection:** a fuse or PTC per cell, an NTC at the buck, VBAT and 6 V rail sensing.
- **Leg ID:** a resistor divider on one ADC pin.
- **SWD:** pads for debugging and recovery.

Shared on the board: an XT30 input, a low-Iq 3.3 V buck (stays on in Sentinel), CAN in/out on JST-GH, and a termination jumper at the outer end.

About 24 MCU pins in total: 3 PWM, 8 ADC, 3 CAN, 1–2 oscillator, 4 ToF, 3 SWD/NRST, 3 buck control/LED. Check the fit in CubeMX before layout.

### Power & BMS board

Battery protection, USB-C charging and soft power, with a power MCU on CAN. BOM in `hardware/power-board/BOM.md`.

- **Pack:** detachable, XT60 for power plus one 5-pin JST-XH balance plug (GND, C1–C4), as on drone packs, so a store-bought 4S LiPo also fits and a hobby balance charger is a backup. The 30 A fuse sits in the pack lead. Plugging the pack in doesn't spark: the BMS FETs start open.
- **BQ7694202 battery monitor** (the 02 variant has REG1 on from the factory, which starts the MCU): two high-side FETs (BSC010N04LS), so every board keeps a common ground. Cell over- and undervoltage, overcurrent and short circuit, temperature, balancing and coulomb counting. Its pre-discharge path is the inrush limit for the side boards and the carrier.
- **BQ25798 charger, USB-C only:** single input, no input FETs. A CYPD3177 asks a PD charger for 20 V, about 60 W (2.5 h); a 5 V-only source still charges at about 15 W. Plugging USB-C in wakes the BMS. The robot's load does not go through the charger.
- **No hot-swap controllers, no 5 V buck:** VBAT goes straight from the BMS FETs to both side boards and the carrier over XT30s. The 5 V buck is on the carrier; the power board switches it with 5V_EN in J-SYSCTL.
- **Soft power:** no switch in the 20 A path. Off means the BMS is in SHUTDOWN (about 1 µA, safe for months). A press on the power button (or plugging USB-C in) wakes it through TS2; REG1 enables the 3.3 V buck for the MCU, which configures the BMS and closes the FETs. A charger alone only charges, with 5V_EN held low so the CM5 stays off. A long press, low battery or a command from ROS asks the CM5 to shut down over J-SYSCTL (`gpio-shutdown`), waits for HALTED (`gpio-poweroff`, 30 s fallback), then opens the FETs and puts the BMS in SHUTDOWN, which also removes the MCU's own supply. A clean halt matters with the CM5 on microSD.
- **E-stop, no mushroom button:** stops come from ROS, the ELRS switch or the operator page, as the e-stop flag in SYNC. The power MCU then pulls ESTOP_N low from an open-drain pin, and a diode on every leg cell holds its servo buck's EN low: servo power is gone without any leg firmware. The CM5 stays up. If the CM5 or the power MCU hangs, SYNC stops and every leg crouches and powers down on its own.
- **MCU:** an STM32C092 (node 7) configures the BMS and charger at every start (their settings live in RAM), runs the power states, keeps the state of charge and reports on CAN. See `firmware/power-node/README.md`.

### CM5 carrier

- **CM5:** CM5 Lite, 2 GB, Wi-Fi, booting from a microSD socket on the carrier (more space than eMMC). 2 GB is enough, measured on a Pi 5 2 GB: the control stack (ros2_control, gait, monitor, foxglove_bridge) in Docker uses 481 MB system-wide, leaving 1.5 GB for camera and inference; H.264 in software costs 20 % of a core at 640×360 / 15 fps.
- **M.2 M-key 2242 on the single PCIe lane:** kept only as an upgrade path for a Hailo accelerator. Not fitted on the prototype or the CM5 board; vision runs on the CPU.
- **5 V for the CM5:** a TPS56A37 (the side boards' servo buck, set to 5.0 V) fed with VBAT over an XT30 from the power board. EN is pulled up on the carrier and pulled low by the power board (5V_EN in J-SYSCTL); PG goes back as 5V_PG. Without a power board, 12–16.8 V on the XT30 just runs it, which is how it's powered on the bench.
- **CAN:** MCP251863 (MCP2518FD plus transceiver in one package), which shows up as `can0` through the mainline `mcp251xfd` driver. The bus passes through the carrier, with no termination on it.
- **LTE:** Quectel EC25-EUX in an mPCIe socket (USB 2.0 lines only), with a nano-SIM and a 3.8 V / 3 A buck.
- **GNSS:** u-blox MAX-M10S plus Quectel LC76G, two vendors for real redundancy. Both use active antennas with a bias-tee each, plus a SAW filter, because LTE band 3 uplink sits close to GPS L1.
- **Nose board connector (J-NOSE):** 3V3 and I2C for the front sensors, plus the CSI connector for the camera ribbon. An I2C header for an external compass stays as a fallback.
- **SDR:** RJ45 to the AD9363 over GbE. That gives far more bandwidth than its USB 2.0 port.
- **IMU:** ICM-42688-P.
- **Other connectors:**
  - 2× USB-A behind current-limit switches
  - CSI camera (the main camera)
  - microSD socket (the CM5 Lite boots from it)
  - ELRS and lidar UARTs
  - gimbal PWM for 2× SG90 with a small 5 V buck
  - a USB-C port for rpiboot, data only: D+/D− to the CM5's USB 2.0, 5.1k on CC, VBUS not connected to the 5 V rail (the CM5 is powered from VBAT, or a bench supply on the XT30). With a CM5 Lite on microSD it's rarely needed
  - an nRPIBOOT jumper and a 3-pin console UART header
  - a fan
  - an RTC battery
- **SWD recovery header:** on CM5 GPIOs.
- **Antennas:** all go to a printed plate on top via U.FL pigtails, with GNSS as far from LTE as possible.

CM5 UARTs are all taken: console, ELRS, GNSS1, GNSS2, lidar.

### Nose board

A small 2-layer board in the nose cone, looking forward through an opening in the shell. It carries the sensors that need to see ahead or stay away from the power wiring:

- **VL53L8CX front ToF** (8×8 zones, up to ~4 m, 1–2 m in direct sun, 65° diagonal FoV), soldered directly: the chip is 35 RON against 150+ for a breakout. It gives measured distance to what the camera sees. The VL53L9CX (54×42 zones over MIPI CSI) was considered: far better data, but no Pi 5 / CM5 driver yet and about 8× the price, so it stays a possible upgrade.
  - Supplies: AVDD 3.3 V from the carrier (43–50 mA ranging), CORE_1V8 and IOVDD from one 1.8 V LDO (TLV75518P class, ≥ 150 mA). IOVDD can't be 3.3 V.
  - Level shifting to the CM5's 3.3 V: PCA9306 on SDA/SCL (2.2k to 1.8 V on the sensor side), a BSS138 each on LPn and INT.
  - Straps for I2C: SPI_I2C_N and NCS 47k to GND, 47k pull-ups on INT, LPn and SYNC, MISO open, RSVD1–3 and the thermal pad to GND.
  - LPn goes to a CM5 GPIO so the sensor can be restarted. Keep ST's protective film on the lens while soldering; no printed plastic in front of it, only an opening at least as wide as the exclusion cone.
- **MMC5983MA compass,** moved here from the carrier: the nose is the point furthest from the servo, buck and LTE currents. Same I2C bus (0x30, the ToF is 0x29).
- **Pi camera mount:** the camera module screws onto the nose board (standard Pi camera hole pattern), next to the ToF with parallel optical axes. That makes the camera–ToF transform fixed by the PCB instead of a printed part, so the ToF zones can be fused with the camera: each YOLO box takes its measured distance from the zones in its direction. The ribbon goes straight to the carrier's CSI connector.
- A status LED (and room for a small buzzer, a "robot about to move" warning).

The IMU stays on the carrier near the body's centre, where rotation doesn't add acceleration.

**Compass placement.** A wire carrying 10 A makes about 40 µT at 5 cm, which is close to Earth's field here (~48 µT). Fixed iron calibrates out; changing currents don't. So:
- keep it at least 5 cm from inductors, the LTE module and the fan
- no high-current copper under it
- twist all the power pairs
- use brass or nylon screws nearby
- calibrate with the servos powered

In Sentinel Stance the legs are off, so the compass is at its cleanest exactly when heading matters. While walking it gets fused with GNSS course and the gyro.

## Bus and flashing

Classic CAN at 1 Mbit/s. SYNC, commands and state for six legs at 200 Hz plus status at 50 Hz measure about 32% bus load on vcan. The message layout is in `protocol/vector.dbc`: `ID = (function << 4) | node`, nodes 1–6 are legs L1–R3, 7 is the power board.

CAN was picked over RS-485 for its hardware CRC, retransmit and arbitration. A babbling node goes bus-off on its own, and SocketCAN tooling is good. The CH32V003 was dropped because it has no CAN, only 16 KB / 2 KB of memory, and no practical way to flash it from Linux.

Every leg has a heartbeat watchdog. If the bus goes quiet, it crouches under control and then cuts its servos. A second bus wouldn't help: losing a side stops the walking anyway.

All MCUs get flashed from the CM5:
1. **Normal:** our own 7 KB CAN bootloader (CAN IDs from the leg ID, CRC-checked image, 200 ms listen window after reset); `leg_config.py flash` does one leg or all of them. ST's OpenBootloader was dropped: it's larger and doesn't know the leg ID.
2. **Fallback:** the factory FDCAN ROM bootloader (AN2606/AN5405), one node at a time.
3. **Recovery:** SWD through OpenOCD's `linuxgpiod` driver on CM5 GPIOs. No ST-LINK needed.

## Power and weight

- **Pack:** 4S3P from the scooter cells. Test capacity and internal resistance on every cell and match the parallel groups. 3P keeps a 20 A peak at about 7 A per cell. Expect ~130 Wh and ~570 g.
- **Series links:** use 0.2 mm pure nickel, doubled, or nickel-plated copper. Never nickel-plated steel.
- **Walking:** 40–70 W for the servos, 12–18 W for the rest.
- **Sentinel:** 9–15 W, so about 9–14 h.
- **Weight:** aim for 2.5 kg or less all-up. The estimate is already 2.4–2.6 kg.
  - In tripod gait each leg carries about a third of the mass. At 8 cm from the femur joint to the foot, that's roughly 6.7 kg·cm static per femur servo, which is near the MG996R's real limit.
  - Keep the femur and tibia short and the battery low. The frame is where to save weight.

## Software

**MCU firmware.** CubeMX generates a CMake project, built with arm-none-eabi-gcc and edited in VS Code, bare-metal on LL/HAL. Zephyr is overkill for a 1 kHz loop with a fixed CAN schedule.

The leg loop runs at 1 kHz: PWM out, ADC in, soft-start, VL53L1X read, watchdog.

**CM5 / Pi 5.** Raspberry Pi OS, with ROS 2 Jazzy in Docker. The same images run on the Pi 5 and the CM5. It's split in three:

1. **`libvector`:** plain C++ with no ROS in it (kinematics, gait, leg state machine, CAN encoding generated from the DBC). Unit-tested on the desktop.
2. **One real-time process:** the ros2_control hardware interface over SocketCAN plus the gait controller, at 200 Hz on an isolated core with SCHED_FIFO. Safety doesn't depend on its timing.
3. **Normal ROS nodes for everything else:**
   - `robot_localization` (2× GNSS, compass, IMU)
   - ToF ray mapping
   - perception
   - a link manager (ELRS > Wi-Fi > LTE; on total loss, stop and go into Sentinel)
   - BMS, teleop

**Remote access.** Tailscale on Wi-Fi and LTE. The UI is Foxglove through `foxglove_bridge`. Don't run DDS over LTE; if several hosts ever need ROS traffic, use `rmw_zenoh`.

**Vision.** The camera (CSI) runs on the host and hands raw frames to the vision container. YOLO26n runs on the CPU from an ONNX export with ONNX Runtime (no PyTorch on the robot); measured on a Pi 5: segmentation at 320 px 12 fps, detection at 480 px 7 fps, about 150 MB. It publishes detections and the class mask into ROS, and the annotated video goes to MediaMTX as RTSP and WebRTC for the operator. The camera is for obstacle avoidance; position comes from GNSS.

**Navigation.** All testing happens outdoors, so GNSS (plus the compass) is the primary sensor, and there's no loop-closing SLAM.
- Position: leg odometry and the IMU gyro fused with GNSS in `robot_localization` (`navsat_transform` for the GNSS side).
- Missions: GPS waypoints sent over LTE; a small waypoint follower first, Nav2 only if memory allows.
- Obstacles: a local costmap around the robot. The segmentation mask is projected onto the ground using the camera pose (leg angles plus IMU tilt); YOLO boxes give an object's distance from where their bottom edge meets the ground; the leg ToF rays add true 3D points near the feet; the nose board's VL53L8CX gives measured distance ahead, fused with the camera mounted on the same board.
- A YOLO-seg model fine-tuned for trail vs not-trail (RELLIS-3D, RUGD, own footage) comes later; COCO classes cover people, vehicles and animals first.
- The robot walks at 5–15 cm/s, so 5–10 fps of perception is enough.

**ToF on the legs.** The pot feedback gives each sensor's exact pose, so as the legs swing the six rays sweep the ground ahead. That's good for spotting steps, gaps and obstacles before a foot lands. It's too sparse for SLAM and weak in direct sun; that's what the lidar is for.

Mount the sensors on the coxa link, so the cable crosses only one joint. A knee mount sees the foot better, but its cable also crosses the femur joint.

## Development setup

- 2× NUCLEO-C092RC (ST-LINK and a CAN transceiver on board)
- my CANable on the Pi 5, flashed with candleLight so it shows up as `can0`, same as the carrier
- MCP2515 modules as a fallback:
  - their 8 MHz crystal limits them to 500 kbit/s
  - power the MCP2515 at 3.3 V, or it puts 5 V on the Pi's GPIOs
- VL53L1X and INA181 breakouts
- the CH32V003 boards, for early servo and pot tests

Bring-up order:
1. Pi 5 plus the Nucleos, with one modded servo.
2. One side board on a bench PSU.
3. The whole robot on the bench PSU.
4. The power board and the battery.
5. The CM5 carrier replaces the Pi 5.

## Plan

| Weeks (from 2026-09-28) | |
|---|---|
| 1–2 | Frame CAD and weight budget, servo and pot measurements, cell tests, leg firmware on a Nucleo, side board schematic |
| 3 | Side board rev A to fab, power board schematic, URDF and gait in sim |
| 4–5 | Power board rev A to fab, carrier schematic, print the frame |
| 6–7 | Side board bring-up, walking on the bench PSU, carrier rev A to fab |
| 8–9 | Power board bring-up, walking on battery |
| 10–11 | Carrier bring-up, rev B of everything |
| 12–14 | Sentinel and wake-up, link failover, LTE/VPN, GNSS/compass, SDR and AI demos |
| 15–17 | Field tests, write-up, video, PCBWay project page |

## Open

- Side board length (depends on the chassis).
- Connector choices marked TBD in `interfaces.md`.
