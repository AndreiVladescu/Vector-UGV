# Software

Runs on the Pi 5 during development and on the CM5 later: Raspberry Pi OS, with ROS 2 Jazzy in Docker.

Packages in `ros2_ws/src/`:
- `vector_description`: URDF and `config/legs.yaml`, the one place leg lengths and hip positions live
- `vector_gait`: kinematics and gait in plain C++ (`vector_core`, no ROS), plus `gait_node`
- `vector_hw`: ros2_control hardware for the leg nodes over SocketCAN, plus `sim_legs` (the leg firmware logic with simulated servos) and `leg_config.py` (calibration and settings over CAN)
- `vector_bringup`: launch files, controller config, rviz config, Gazebo worlds
- later: `vector_perception`, `vector_links`

## Desktop setup (Mint 22.3 / Ubuntu 24.04)

```sh
sudo apt install curl
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F tag_name | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $UBUNTU_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
sudo apt install ros-jazzy-desktop ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-xacro ros-jazzy-teleop-twist-keyboard ros-jazzy-plotjuggler-ros \
  ros-jazzy-ros-gz ros-jazzy-gz-ros2-control python3-colcon-common-extensions \
  python3-can can-utils
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
