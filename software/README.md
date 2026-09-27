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
ros2 topic echo /gait_node/mode                                # walk, stopping, lowering, sentinel, powering, raising
```

The legs switch through the `leg_power` GPIO controller (`legs/enable`): on CAN it sets the enable bit in `LEG_CMD`, and its state reads 1 only when all six legs report active. If they don't within `power_timeout` (10 s) the robot stays down. Keys are ignored while sitting or getting up. `sentinel_height` (30 mm hip height) should become the real belly height once the body is designed. In Gazebo there is no leg power, so it only lowers and raises (body at 49 mm, back to 90 mm).

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

On the real robot it's the same launch with `can_interface:=can0` (CANable on the Pi 5, MCP251863 on the carrier). The hardware waits for all six legs before activating, starts from their measured angles, and drops out if a leg is silent for 100 ms. Walking at 200 Hz uses about 32% of the 1 Mbit/s bus.

## Checks

```sh
ros2 run vector_gait gait_report src/vector_description/config/legs.yaml --mass 2.5
colcon test --ctest-args -LE gazebo          # unit tests, kinematic sim, CAN loop (skips without vcan0)
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

`tools/` holds bring-up scripts (python-can, cantools) and the CAN flasher.

## When the robot doesn't move

1. Leftovers from earlier runs. A closed terminal or a crashed launch can leave a Gazebo server or an old `gait_node` running; the next launch then talks to the old controller manager and the new robot gets no controllers. `sim_gazebo.launch.py` prints a warning if it finds an old Gazebo. Clean up with:
   ```sh
   pkill -f "gz sim"; pkill -f gait_node; pkill -f ros2_control_node; ros2 daemon stop
   ```
2. Controllers: `ros2 control list_controllers` should show `joint_state_broadcaster` and `leg_controller` as active.
3. Commands arrive: `ros2 topic echo /cmd_vel` while pressing keys, and `ros2 node list` should show one `/gait_node`. The teleop terminal needs keyboard focus.
4. After pulling changes to C++ code, rebuild (`colcon build --symlink-install`) and re-source `install/setup.zsh` in every terminal.
