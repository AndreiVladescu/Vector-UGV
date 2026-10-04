# Software

Runs on the Pi 5 during development and on the CM5 later: Raspberry Pi OS, with ROS 2 Jazzy in Docker.

Packages in `ros2_ws/src/`:
- `vector_description`: URDF and `config/legs.yaml`, the one place leg lengths and hip positions live
- `vector_gait`: kinematics and gait in plain C++ (`vector_core`, no ROS), plus `gait_node`
- `vector_hw`: ros2_control hardware for the leg nodes over SocketCAN, plus `sim_legs` (the leg firmware logic with simulated servos) and `leg_config.py` (calibration and settings over CAN)
- `vector_bringup`: launch files, controller config, rviz config, Gazebo worlds
- `vector_link`: ExpressLRS (`elrs.py`), the cmd_vel mux and link watchdog, the operator page
- `vector_nav`: waypoint follower and obstacle grid
- `vector_tof`: the nose board's VL53L8CX
- `vector_io`: the carrier's IO MCU over UART2 (lidar, GNSS, ELRS frames, LoRa, housekeeping) and its flasher
- `vector_imu`: LSM6DSV16X on the carrier and MMC5983MA on the nose board -> `imu`, `mag`
- later: `vector_perception`

## Desktop setup (Mint 22.3 / Ubuntu 24.04)

```sh
sudo apt install curl
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F tag_name | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $UBUNTU_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
sudo apt install ros-jazzy-desktop ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-xacro ros-jazzy-teleop-twist-keyboard ros-jazzy-plotjuggler-ros \
  ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-robot-localization \
  ros-jazzy-nmea-navsat-driver python3-colcon-common-extensions python3-can can-utils
pip install --user --break-system-packages cantools
```

On Mint the `UBUNTU_CODENAME` line resolves to `noble`, which is what the ROS repo needs.

## Kinematic sim

```sh
cd software/ros2_ws
source /opt/ros/jazzy/setup.zsh          # setup.bash in bash
colcon build --symlink-install && source install/setup.zsh

ros2 launch vector_bringup robot.launch.py               # mock joints
ros2 run teleop_twist_keyboard teleop_twist_keyboard     # second terminal
ros2 param set /gait_node gait ripple                     # tripod / ripple / wave, while standing
ros2 param set /gait_node body_pitch 0.15                 # body_x/y/z/roll/pitch/yaw, live
```

Teleop keys (the teleop terminal needs focus). By default a key latches: the robot keeps doing it until the next key, and `k` stops. For hold-to-move instead, `ros2 param set /gait_node cmd_timeout 0.6`: it walks while the key is held (keyboard auto-repeat) and stops 0.6 s after release.

| Key | Motion |
|---|---|
| `i` `,` | forward / back |
| `j` `l` | turn left / right |
| `u` `o` `m` `.` | forward or back while turning |
| `J` `L` (shift) | strafe left / right |
| `U` `O` `M` `>` (shift) | diagonal |
| `t` `b` | body 5 mm up / down per press, −40 to +30 mm; doesn't stop the walk |
| `k` | stop |
| `q`/`z`, `w`/`x`, `e`/`c` | all / linear / turn speed ±10% |

Teleop starts at 0.5 m/s but the gait caps walking at about 0.12 m/s (6 cm stride), so the speed keys only show once you're below that. Speed changes ramp at 0.25 m/s² (`accel`, `turn_accel`), so starting, stopping and switching direction take about half a second. Switching gait needs the robot standing: press `k` first. In rviz the camera follows the robot, so strafing shows as the grid and foot trails sliding sideways.

Mock hardware, so the joints go exactly where they're told. Useful for checking foot paths (the coloured trails), reach and joint speeds (PlotJuggler on `/joint_states`), not loads. Change geometry in `legs.yaml` and relaunch.

## Gazebo

```sh
ros2 launch vector_bringup sim_gazebo.launch.py                      # flat ground
ros2 launch vector_bringup sim_gazebo.launch.py world:=slope level:=true
ros2 launch vector_bringup sim_gazebo.launch.py world:=rough rviz:=true
```

Same controllers and gait node, with masses from `legs.yaml` and the joint effort capped at MG996R stall (1.08 N·m). Joint torques are in the `effort` field of `/joint_states`, ground truth pose on `/ground_truth`, IMU on `/imu`. Gazebo's position control is an idealised servo, so trust standing torques more than walking peaks.

Worlds: `flat`, `slope` (8°), `rough` (40 slabs, 1–3 cm, fixed layout). `level:=true` (or `ros2 param set /gait_node level true`) feeds the IMU tilt back into the body pose. On the 8° slope the body goes from −8.0° to 0.0°.

## Sentinel Stance

```sh
ros2 service call /gait_node/sentinel std_srvs/srv/Trigger   # stop, lower onto the belly, legs off
ros2 service call /gait_node/wake std_srvs/srv/Trigger        # legs on, wait for all six, stand up
ros2 topic echo /gait_node/mode                                # walk, stopping, lowering, sentinel, powering, raising, halted
ros2 service call /gait_node/estop std_srvs/srv/SetBool "{data: true}"   # e-stop; false releases, then wake
```

The legs switch through the `leg_power` GPIO controller (`legs/enable`): on CAN it sets the enable bit in `LEG_CMD`, and its state reads 1 only when all six legs report active. If they don't within `power_timeout` (10 s) the robot stays down. Keys are ignored while sitting or getting up. `sentinel_height` (30 mm hip height) should become the real belly height once the body is designed. In Gazebo there is no leg power, so it only lowers and raises (body at 49 mm, back to 90 mm).

If a leg drops out while walking (a fault, see below), the node stops in `halted`: the other legs finish their step and stand, keys are ignored. Sentinel then wake power-cycles the legs, which clears the fault if its cause is gone.

E-stop sets the flag in SYNC (`legs/estop`): every leg switches off at once, which drops the body, and the node goes straight to sentinel. Wake is refused until the e-stop is released. The hardware e-stop line to the legs does the same without ROS.

## Touchdown (off by default)

`touchdown:=true` makes each foot stop where it meets the ground late in its swing, or probe up to 20 mm deeper into a hole; stance feet then keep their own height. Contact comes from `contact_from: position` (measured foot higher than anything commanded in the last 40 ms, from the pots) or `load` (femur + tibia torque, leg current on the robot).

In Gazebo it isn't convincing yet: on the rough world tilt goes from 2.7° to 2.0–2.5° RMS, but on flat ground the body bobs 2–5 mm more, and the gain with `load` came from feet pressing down, not from contact being sensed. Gazebo's servo is ideal and stiff; tune this against the real MG996R (pot error and leg current when a foot is blocked) before turning it on.

## CAN without hardware

`sim_legs` runs the actual leg firmware logic (`firmware/leg-node`) six times, each with three simulated MG996Rs that behave like the bench measurements (wiper noise, speed, the restart quirk), on a virtual CAN bus. `vector_hw` builds it.

```sh
sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0   # again after a reboot
ros2 run vector_hw sim_legs vcan0 --calibrated --limits -45.8,45.8,-80.2,80.2,-149,17   # joint limits from legs.yaml
ros2 launch vector_bringup robot.launch.py hardware:=can can_interface:=vcan0
candump vcan0 | cantools decode ../../protocol/vector.dbc      # watch the traffic
```

Without `--calibrated` the legs start like fresh boards and refuse to wake until calibrated. Try the calibration mode on them:

```sh
ros2 run vector_hw leg_config.py --channel vcan0 status
ros2 run vector_hw leg_config.py --channel vcan0 calibrate R1 --mode limits
ros2 run vector_hw leg_config.py --channel vcan0 push     # docs/servos.md + legs.yaml, run from the repo
```

On the real robot it's the same launch with `can_interface:=can0` (CANable on the Pi 5, MCP251863 on the carrier). The hardware waits for all six legs before activating, starts from their measured angles, and drops out if a leg is silent for 100 ms; SYNC then stops, and every leg crouches and powers down on its own. Walking at 200 Hz uses about 32% of the 1 Mbit/s bus.

With `hardware:=can` the launch also starts `leg_monitor`, which listens on the same bus:

```sh
ros2 topic echo /diagnostics          # per leg: state, faults, current, battery, servo rail, temperature, ToF
ros2 run rqt_robot_monitor rqt_robot_monitor
ros2 topic echo /legs/L1/tof          # sensor_msgs/Range in frame L1_tof (+inf: nothing in range)
```

Fault changes also go to the log as they happen. The `<leg>_tof` frames sit on the coxa links at a placeholder pose (`tof` in `legs.yaml`) until the sensor bracket exists.

## Power board

`sim_power` runs the power board firmware (`firmware/power-node`) against models of the BQ76942 and BQ25798 and a 4S pack that drains with what is switched on. `power_monitor` (started with `hardware:=can`) turns its frames into ROS:

```sh
ros2 run vector_hw sim_power vcan0 --soc 40          # ends when the board switches itself off
ros2 topic echo /battery                             # BatteryState at 10 Hz, current negative while discharging
ros2 service call /power/shutdown std_srvs/srv/Trigger              # the board asks the CM5 to halt
ros2 run vector_hw leg_config.py --channel vcan0 power              # state, cells, charger, settings
ros2 run vector_hw leg_config.py --channel vcan0 power set charge_ma 2500 --save
ros2 run vector_hw leg_config.py --channel vcan0 power set cell_ntc 1 --save   # DIY pack with a thermistor; next power-up
```

`power_monitor` also puts battery, board and charger entries in `/diagnostics` and a latched `power/estop`. When the board starts a shutdown, or the charge falls under `sentinel_soc` (10 %) while discharging, it calls `gait_node/sentinel` so the robot sits down before the power goes. `kill -USR1` / `-USR2` on `sim_power` press the power button and plug / unplug USB-C.

## Radio and links

With `links:=true` (`LINKS=true` in `/etc/vector.env`, the default with `can0`) `link_manager.py` is the only node that writes `cmd_vel`, from the first of: the ExpressLRS radio while armed (`cmd_vel/elrs`), an operator over Wi-Fi or LTE (`cmd_vel/teleop`), the GPS mission (`cmd_vel/nav`). The operator counts as connected while teleop commands or `operator/heartbeat` (std_msgs/Empty, about 1 Hz, e.g. from a Foxglove publish panel) keep coming. With no radio and no operator for 5 s the robot stops and sits down, and stays down until woken; a mission only moves while a link is up (`missions_alone` to change that).

```sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel/teleop
ros2 topic pub -r 1 /operator/heartbeat std_msgs/msg/Empty
ros2 topic echo /links/active          # elrs, teleop, nav or idle
```

The operator page, `http://<robot>:8080` on a phone or laptop (started with `links:=true`): the YOLO video, battery, links and gait mode, a joystick (forward and turn, or sideways), Sit / Wake / E-STOP, a map with the GNSS position where tapping adds waypoints to send as a mission, and a systems panel (chip button) with the carrier's battery, 5 V rail and board temperature, the GNSS, lidar, receiver, IMU and compass state, the LoRa beacon count and the last packet received, the LTE supply switch and a Beep button to find the robot. The map button lays the map faintly over the video (10-20 % opacity), centred on the robot with its track; the tiles button picks which small readouts stay on screen (ELRS link quality and RSSI, Wi-Fi, LTE, LoRa RSSI and SNR, position, GNSS, altitude, heading, tilt, speed, battery, carrier rail, nearest obstacle, distance home, mission, latency). Both choices are kept in the browser. While it's open it sends the operator heartbeat; commands stop 0.4 s after the last one, so a phone that drops out doesn't leave the robot walking. The map tiles come from OpenStreetMap, so they need internet on the phone; the rest doesn't. Anyone who can reach port 8080 can drive the robot: keep it on Tailscale, or set `page_key:=...` and open `http://<robot>:8080/?key=...`.

`elrs.py` reads an ExpressLRS receiver in CRSF mode (420000 baud) on UART2 (GPIO4 TX, GPIO5 RX, `/dev/ttyAMA2`) on the Pi; on the carrier the receiver hangs off the IO MCU and `elrs_port:=io` takes its frames from `io_bridge`. Mode 2: right stick forward / sideways, left stick sideways turns and up / down sets the top speed (30-100 % of 0.1 m/s). AUX1 arms, AUX2 high sits down (low wakes), AUX3 high is the e-stop. The handset shows the battery (voltage, current, used mAh, %) and the gait mode as telemetry. Set the receiver to failsafe "no pulses", so a lost link stops the sticks.

The LTE modem (A7670E) in RNDIS / ECM mode is a wired interface to Linux; `setup.sh` adds a NetworkManager profile that matches it by driver and puts it behind Wi-Fi (route metric 700). `/diagnostics` shows the control source, the Wi-Fi signal and the modem's state, plus its signal with `LTE_AT=/dev/ttyUSB2` (the AT port number can differ; `AT+CSQ` is plain 3GPP). The APN and the modem's USB mode are set once with its own AT commands, see SIMCom's A76XX documents.

## GNSS and waypoints

Position is GNSS first: `robot_localization` runs two EKFs (leg odometry `odom/legs` from `gait_node` plus the IMU for `odom -> base_link`; the same plus GNSS through `navsat_transform` for `map -> odom`), and `waypoints.py` walks GPS points. Config in `vector_nav/config/localization.yaml`.

```sh
ros2 launch vector_bringup robot.launch.py hardware:=can gnss_port:=/dev/ttyAMA0 nav:=true
ros2 launch vector_bringup sim_gazebo.launch.py nav:=true                 # simulated GNSS, 0.5 m noise
ros2 run vector_nav waypoints.py send 44.42690,26.10280 44.42700,26.10260
ros2 topic echo /mission/status
ros2 run vector_nav waypoints.py cancel
```

The receiver is any NMEA module on UART0 (GPIO14 TX, GPIO15 RX, 3.3 V), 115200 baud (`gnss_baud`); `setup.sh` enables the UART and passes `/dev/ttyAMA0` into the container (`GNSS=` and `NAV=` in `/etc/vector.env`). A mission is refused until there is a fix, and the robot stops while the fix or the filtered position is stale. It only sends `cmd_vel` while a mission runs, so teleop still works otherwise; the Gazebo world's origin is 44.4268 N, 26.1025 E.

`obstacles.py` (also started by `nav.launch.py`) keeps what sticks out of the ground for 3 s in `odom`: where YOLO's objects meet the ground (the lowest mask pixel per column, projected from the camera pose on flat ground), the nose ToF zones and the six leg ToF rays more than 5 cm above the ground. It publishes `obstacles/grid` (OccupancyGrid, for Foxglove) and `obstacles/clearance` (free metres straight ahead, and the side with more room). The follower stops with less than 0.6 m free, and after 2 s steps sideways around it; without obstacle data it walks on GNSS alone.

`navsat_transform` needs an absolute heading from the IMU (0 = east). Gazebo's IMU has one; on the robot it comes from `imu_node.py` with the nose board's compass (below). Without the nose board the IMU's yaw is gyro only and `nav:=true` has no heading.

## Operator page demo

Everything the page shows, on the desktop with no hardware: the mock robot and gait, the link manager and the page, the carrier's IO MCU simulated on a pseudo-terminal (`sim_io.py`, speaking the real link protocol to the real `io_bridge`) and `imu_node` on simulated chips. Walking from the page moves the GNSS position (`gnss_scale` multiplies it, so the map shows it), the heading, the lidar's view of a 6 x 4 m yard and the battery current; ELRS link quality wanders, a LoRa "ping" arrives every 20 s, Beep and the LTE switch reach the simulated MCU.

```sh
ros2 launch vector_bringup demo.launch.py gnss_scale:=20       # then http://localhost:8080
ros2 param set /sim_io radio false                              # also lidar, gnss, lora
ros2 param set /sim_io vbat 13.3                                # low battery
```

There's no video in the demo; the page shows NO VIDEO until a stream answers.

## Carrier IO MCU and IMU

The STM32C092 next to the CM5 (`firmware/io-node`) carries the ELRS receiver, the LD19 lidar, the MAX-M10S, the RFM95W and the battery / 5 V / NTC sensing, and talks to the CM5 on UART2 at 1 Mbaud. `io_bridge.py` turns that into `scan` (LaserScan, frame `lidar`), `gnss/nmea_sentence` (and through `nmea_topic_driver` the usual `gnss/fix`, `gnss/vel`), `io/crsf` for `elrs.py`, `lora/rx` / `lora/tx` and an `io: mcu` entry in `/diagnostics`. The MCU sends a LoRa position beacon every 30 s on its own (`beacon_s`, 0 = off), so a lost robot can be found with any SX127x receiver.

```sh
ros2 launch vector_bringup robot.launch.py hardware:=can io_port:=/dev/ttyAMA2 elrs_port:=io imu:=true nav:=true
ros2 service call /io/lte_power std_srvs/srv/SetBool "{data: true}"
ros2 param set /io_bridge lidar_pwm 0                      # 0 = the LD19's own 10 Hz
ros2 run vector_io io_flash.py io-node.bin                 # new IO firmware through the ROM bootloader
```

`imu_node.py` reads the LSM6DSV16X (0x6A) and the MMC5983MA (0x30) on I2C1 and publishes `imu` with the orientation in ENU, fused from gyro, accelerometer and compass, and `mag`. Keep the robot still for the first 2 s (gyro bias). `imu_axes` / `mag_axes` map each chip onto `base_link` and `declination_deg` (6 for Bucharest) turns magnetic north into true north. Calibrate the compass once it sits in the robot: `ros2 service call /imu/calibrate_mag std_srvs/srv/Trigger`, then turn the robot through every orientation for 30 s and copy the logged `mag_offset` / `mag_scale` into the launch parameters. In the container: `IO_PORT=/dev/ttyAMA2`, `ELRS_PORT=io` and `IMU=true` in `/etc/vector.env`.

## On the robot (Docker)

The robot runs the same packages in one image (no Gazebo or rviz), built natively on the Pi / CM5 in about 2.5 minutes:

```sh
docker build -f software/docker/Dockerfile -t vector .                        # from the repo root
docker build -f software/docker/Dockerfile --target build -t vector:build .   # with the tests
CAN_INTERFACE=can0 docker compose -f software/docker/compose.yaml up -d      # robot + foxglove_bridge
CAN_INTERFACE=vcan0 docker compose -f software/docker/compose.yaml --profile sim up -d   # with sim_legs
```

Always run ROS containers with a normal open-files limit (`nofile: 65536`, set in `compose.yaml`). Docker's default is about 2^30, and LTTng, which every ROS 2 process loads through `tracetools`, allocates a bitmap sized by it: 128 MB per process, 1.4 GB for this stack.

Measured on a Pi 5 2 GB (`software/tools/`): the stack uses 481 MB system-wide, Docker itself 133 MB of that; SYNC runs at 5.00 ms with a worst gap of 8 ms idle and 15 ms with all four cores busy (the legs crouch after 100 ms); the control loop gets SCHED_FIFO 50.

To make a fresh Pi OS install start everything on its own, run once from the repo root:

```sh
sudo software/host/setup.sh                                  # real legs on can0, vision on
sudo CAN=vcan0 PROFILES=sim,vision software/host/setup.sh    # bench, with sim_legs
```

It installs Docker (default nofile 65536, logs to journald) and adds `vector-can` (can0 at 1 Mbit/s or vcan0), `vector-camera` (rpicam-vid) and `vector-stack` (`docker compose up -d`), and arms the hardware watchdog at 15 s. Settings live in `/etc/vector.env`. From power-on on the Pi 5 (NVMe): Linux ready in 10.7 s, legs active at 13.3 s, first YOLO result at about 18 s.

### A card for the robot (CM5 on microSD)

```sh
software/image/make_card.sh vector.img --ssid Home --ssh-key ~/.ssh/pi5.pub      # an image file, flash it later
sudo software/image/make_card.sh /dev/sdX --ssid Home --ssh-key ~/.ssh/pi5.pub   # straight to a card
```

It takes the latest Raspberry Pi OS Lite (arm64, checked against its SHA-256), and puts in the boot partition a cloud-init `user-data` (hostname `vector`, user `pi` with only the SSH key, Wi-Fi or Ethernet) and this checkout's code as `vector.tar.gz` (the GitHub repo is private, so nothing is cloned; `VERSION` in it says which commit, and whether there were local changes). On first boot `firstboot.sh` runs `setup.sh`, builds the three images, exports the YOLO model and reboots into the running stack: about 20 minutes with internet, log in `/var/log/vector-firstboot.log`. `--can vcan0 --profiles sim,vision` makes a test box.

`setup.sh` also looks after the card: the journal stays in RAM (64 MB), swap only in zram, `fsck.repair=yes`. The root filesystem stays writable, since Docker's storage can't sit on an overlay root without a partition of its own; a clean halt through the power board is what keeps it intact.

## Camera and YOLO

The camera stays on the host, where libcamera is native, and hands raw frames to the vision container over a local socket; nothing is encoded twice. YOLO runs from ONNX with ONNX Runtime (`software/vision/yolo_onnx.py`), no PyTorch on the robot; PyTorch and Ultralytics live only in an export image, used once:

```sh
docker build -f software/vision/Dockerfile --target export -t vector-yolo-export .
SIZES="320 480" docker run --rm -e SIZES -v docker_models:/models vector-yolo-export    # yolo26n(-seg)-<size>.onnx
docker build -f software/vision/Dockerfile -t vector-vision .
docker compose -f software/docker/compose.yaml --profile vision up -d                    # YOLO + MediaMTX
YOLO_MODEL=yolo26n-480.onnx docker compose -f software/docker/compose.yaml --profile vision up -d
```

`yolo_node.py` publishes `/yolo/detections` (vision_msgs), `/yolo/mask` (class id + 1 per pixel) and `/yolo/debug/compressed`, and streams the annotated video through MediaMTX:

- VLC: `rtsp://<robot>:8554/yolo`
- Firefox / any browser: `http://<robot>:8889/yolo` (WebRTC)

On the Pi 5 2 GB, CPU only, with the control stack running alongside (SYNC never gaps more than 8 ms):

| Model | fps | frame to result | node memory |
|---|---|---|---|
| yolo26n-seg, 320 px | 12.2 | 112 ms | 147 MB |
| yolo26n-seg, 480 px | 5.2 | 225 ms | 182 MB |
| yolo26n, 320 px | 14.4 (camera-limited) | 86 ms | 136 MB |
| yolo26n, 480 px | 7.2 | 167 ms | 162 MB |

The camera captures at 1280x960 (`CAMERA_WIDTH`/`CAMERA_HEIGHT` in `/etc/vector.env`); the model gets a 320 px copy and the masks and boxes are scaled back and drawn on the full frame, so the stream stays sharp. That costs about 1 fps over a 640x480 capture (9 vs 10 fps with the stream on), and the 1280x960 encode takes 16 % of one core.

With the nose board fitted, `tof_front` (`vector_tof`, started with `hardware:=can`) runs the VL53L8CX through ST's ULD on `/dev/i2c-1`: `nose/tof/points` (a point per zone) and `nose/tof/depth` (8x8 metres, NaN = nothing). `yolo_node` takes the zones behind each box, uses a low percentile so the object wins over the background, prints the distance on the stream and publishes `~/detections_3d`. The ULD sources, with the sensor's firmware, are downloaded at build time from ST's X-CUBE-TOF1 (BSD-3-Clause), pinned by hash. The zone grid's orientation (`flip_x`, `flip_y`) needs a check on the bench: a hand in the top left corner should show in row 0, column 0 of the depth image.

Ultralytics itself runs the same ONNX model at half the speed and needs 430 MB, most of it PyTorch. The YOLO26 weights are AGPL-3.0 (Ultralytics), fine for an open project.

## Checks

```sh
ros2 run vector_gait gait_report src/vector_description/config/legs.yaml --mass 2.5
colcon test --ctest-args -LE gazebo          # unit tests, kinematic sim, CAN loop (skips without vcan0)
                                             # also checks that the DBC, firmware, C++ and leg_config.py agree
colcon test --ctest-args -L gazebo           # headless Gazebo: flat walk, slope leveling, rough walk
colcon test-result --verbose
```

`gait_report` runs every gait offline and flags joint speeds over the servo limit, joints near their limits, low stability margin and static torques over half of stall. Rerun it whenever `legs.yaml` changes.

First numbers with the placeholder legs (coxa 50 / femur 80 / tibia 120 mm):

| | 2.5 kg | 1.8 kg |
|---|---|---|
| femur, standing (Gazebo median) | 2.8 kg·cm | 1.9 kg·cm |
| femur, tripod static worst case (`gait_report`) | 8.8 kg·cm | |
| femur at the 11 kg·cm cap while walking (Gazebo) | 30% of samples | 12.5% |

So with these legs, 2.5 kg is too much for comfortable tripod walking on MG996Rs. Shorter femur reach or less mass is where to look. Wave gait also needs a longer period than 1 s (joints too fast).

`tools/` holds bring-up scripts (python-can, cantools), the CAN flasher and the robot-side tests: `sync_jitter.py` (SYNC timing), `mem_report.py` (memory per process), `walk_test.py` (walk, turn, Sentinel and wake in a loop while watching the legs, run inside the robot container) and `soak.py` (a line a minute of memory, temperature, CPU, SYNC and YOLO rate).

## When the robot doesn't move

1. Leftovers from earlier runs. A closed terminal or a crashed launch can leave a Gazebo server or an old `gait_node` running; the next launch then talks to the old controller manager and the new robot gets no controllers. `sim_gazebo.launch.py` prints a warning if it finds an old Gazebo. Clean up with:
   ```sh
   pkill -f "gz sim"; pkill -f gait_node; pkill -f ros2_control_node; ros2 daemon stop
   ```
2. Controllers: `ros2 control list_controllers` should show `joint_state_broadcaster` and `leg_controller` as active.
3. Commands arrive: `ros2 topic echo /cmd_vel` while pressing keys, and `ros2 node list` should show one `/gait_node`. The teleop terminal needs keyboard focus.
4. After pulling changes to C++ code, rebuild (`colcon build --symlink-install`) and re-source `install/setup.zsh` in every terminal.
