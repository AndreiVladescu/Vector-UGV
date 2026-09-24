# V.E.C.T.O.R. spec

Versatile Edge Compute & Terrain Observation Robot. An 18-DOF hexapod built as an open research platform: a smart controller per leg on CAN, a CM5 carrier, a camera and SDR payload, and ELRS / Wi-Fi / 4G links. Entry for the PCBWay 9th Project Design Contest (Robotics), deadline 2027-01-24.

The main idea is "Sentinel Stance". The robot walks to a spot, lowers its body to the ground and cuts power to all servos. It then sits there for hours as a low-power sensor node (camera, SDR, GNSS, 4G). To get up again it powers the legs one at a time, starting each servo at the angle it actually measures, so nothing jerks.

Constraints: low budget. PCBs and stencils come from PCBWay and I assemble them myself. All mechanical parts are printed (PETG, TPU feet) and designed in Fusion 360. ICs come from Mouser.

Already on hand: CM5 (Wi-Fi, 8 GB eMMC), Pi 5 + AI HAT+ 2, 18× MG996R, AD9363 SDR (2T2R), HackRF, Pi CSI camera, a USB camera, 18650 cells from e-scooter packs, a spot welder, GX-12 connectors, a CANable, MCP2515 modules, 2 CH32V003 dev boards.

## Boards

Three designs, four boards:

| Board | Qty | Layers | Main parts |
|---|---|---|---|
| Side board | 2 | 4 | 3 leg cells, each with STM32C092, TJA1051T/3, LM61495 6 V buck |
| Power & BMS | 1 | 4, 2 oz | BQ76952, BQ25798, 2× LM5069, 5 V / 6 A buck, STM32C092 |
| CM5 carrier | 1 | 4, impedance-controlled | CM5, M.2, EC25-EUX LTE, 2× GNSS, MCP251863, IMU, compass |

```
[120Ω] L3 ─ L2 ─ L1 ── CM5 carrier ── R1 ─ R2 ─ R3 [120Ω]      one CAN bus, 1 Mbit/s
       └ side board L ┘      │         └ side board R ┘
                       power board (stub)

power board ──VBAT_L──► side L     ──VBAT_R──► side R     ──5V, VBAT_SYS──► carrier
```

### Side board (×2)

Both sides use the same PCB, rotated 180°. Each side board carries three independent leg cells, and the CAN bus between those cells is just traces. A leg cell has:

- **MCU:** STM32C092 (UFQFPN28/32, at least 64 KB flash) with a crystal. It has FDCAN in hardware and a factory CAN bootloader.
- **CAN transceiver:** TJA1051T/3 (the /3 version has a VIO pin for 3.3 V logic). TCAN1044V fits the same footprint.
- **Servo buck:** LM61495 (36 V in, 10 A) set to 6.0 V, switched by the MCU through EN. Three servos near stall draw about 7.5 A.
- **Current sense:** one 10 mΩ shunt and an INA181 for the whole leg. The load on each joint comes from the pot error (commanded minus measured angle).
- **Position:** the MG996R pot wiper goes through a divider and an RC filter into the ADC. The pot usually sits on the servo's 6 V, so measure it first; roughly a 10k/15k divider.
- **PWM:** the outputs are tri-stated before power-off, otherwise the servo gets back-powered through the signal pin.
- **ToF:** a VL53L1X on its own I2C bus, so there's no address clash (JST-SH 6-pin).
- **Protection:** a fuse or PTC per cell, an NTC at the buck, VBAT and 6 V rail sensing.
- **Leg ID:** a resistor divider on one ADC pin.
- **SWD:** pads for debugging and recovery.

Shared on the board: an XT60 input, a low-Iq 3.3 V buck (stays on in Sentinel), CAN in/out on JST-GH, and a termination jumper at the outer end.

About 24 MCU pins in total: 3 PWM, 8 ADC, 3 CAN, 1–2 oscillator, 4 ToF, 3 SWD/NRST, 3 buck control/LED. Check the fit in CubeMX before layout.

### Power & BMS board

- **BQ76952 battery monitor:** high-side FETs, so every board keeps a common ground. It handles cell over- and undervoltage, overcurrent, temperature, balancing and coulomb counting. It works with the DIY pack or a store-bought 4S LiPo through the balance lead.
- **BQ25798 charger:** two inputs, GX-12 DC (12–24 V) and USB-C PD (STUSB4500 or CH224K, asking for 20 V). About 60 W of charging, charge-only. The robot's load does not go through it.
- **2× LM5069 hot-swap controllers, one per side:** they soft-start the leg bus, limit current, and act as the hardware e-stop (a latching switch on EN). The CM5 stays powered when the e-stop is hit.
- **5 V / 6 A buck:** powers the Pi 5 during development, and the carrier later.
- **MCU:** an STM32C092 configures the BMS at boot (its config lives in RAM), runs the power states, and reports on CAN.

### CM5 carrier

- **CM5:** use the 4 GB RAM version; 2 GB is tight for ROS, inference and video together.
- **M.2 M-key 2242 on the single PCIe lane:** for a Hailo accelerator later. Left empty at first, and the software falls back to the CPU.
- **CAN:** MCP251863 (MCP2518FD plus transceiver in one package), which shows up as `can0` through the mainline `mcp251xfd` driver. The bus passes through the carrier, with no termination on it.
- **LTE:** Quectel EC25-EUX in an mPCIe socket (USB 2.0 lines only), with a nano-SIM and a 3.8 V / 3 A buck.
- **GNSS:** u-blox MAX-M10S plus Quectel LC76G, two vendors for real redundancy. Both use active antennas with a bias-tee each, plus a SAW filter, because LTE band 3 uplink sits close to GPS L1.
- **Compass:** MMC5983MA on the front edge (details below), plus an I2C header for an external one.
- **SDR:** RJ45 to the AD9363 over GbE. That gives far more bandwidth than its USB 2.0 port.
- **IMU:** ICM-42688-P.
- **Other connectors:**
  - 2× USB-A behind current-limit switches
  - CSI camera
  - ELRS and lidar UARTs
  - gimbal PWM for 2× SG90 with a small 5 V buck
  - a USB-C port for rpiboot
  - a fan
  - an RTC battery
- **SWD recovery header:** on CM5 GPIOs.
- **Antennas:** all go to a printed plate on top via U.FL pigtails, with GNSS as far from LTE as possible.

CM5 UARTs are all taken: console, ELRS, GNSS1, GNSS2, lidar.

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
1. **Normal:** a bootloader based on ST OpenBootloader, with CAN IDs tied to the leg ID. A small python-can script flashes one leg or all of them.
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

**Vision.** One GStreamer process splits the camera three ways:
- inference on the Hailo or the CPU, publishing detections and segmentation into ROS
- a throttled low-res copy into ROS, for debugging and for rosbag training data
- H.264 into MediaMTX and WebRTC for the operator

Only the operator stream stays out of ROS.

**Trail navigation.**
- A YOLO-seg model marks trail vs not-trail. Fine-tune it on RELLIS-3D, RUGD and my own footage.
- The mask is projected onto the ground using the camera pose and the IMU tilt, and becomes a costmap layer.
- YOLO obstacle boxes, the ToF rays and the optional LD19 lidar go into the same costmap.
- First version: a reactive trail follower. Later: Nav2 with GNSS waypoints, driving the hexapod as a holonomic base through `cmd_vel`.
- The robot walks at 5–15 cm/s, so 5–10 fps of perception is enough, and even CPU-only YOLO nano at 320 px works.

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
