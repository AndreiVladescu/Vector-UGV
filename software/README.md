# Software

Runs on the Pi 5 during development and on the CM5 later: Raspberry Pi OS, with ROS 2 Jazzy in Docker.

Packages in `ros2_ws/src/`:
- `vector_description`: URDF and `config/legs.yaml`, the one place leg lengths and hip positions live
- `vector_gait`: kinematics and gait in plain C++ (`vector_core`, no ROS), plus `gait_node`
- `vector_hw`: ros2_control hardware for the leg nodes over SocketCAN, plus `fake_legs.py`
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
  ros-jazzy-ros-gz ros-jazzy-gz-ros2-control python3-colcon-common-extensions
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

Mock hardware, so the joints go exactly where they're told. Useful for checking foot paths (the coloured trails), reach and joint speeds (PlotJuggler on `/joint_states`), not loads. Change geometry in `legs.yaml` and relaunch.

## Gazebo

```sh
ros2 launch vector_bringup sim_gazebo.launch.py                      # flat ground
ros2 launch vector_bringup sim_gazebo.launch.py world:=slope level:=true
ros2 launch vector_bringup sim_gazebo.launch.py world:=rough rviz:=true
```

Same controllers and gait node, with masses from `legs.yaml` and the joint effort capped at MG996R stall (1.08 N·m). Joint torques are in the `effort` field of `/joint_states`, ground truth pose on `/ground_truth`, IMU on `/imu`. Gazebo's position control is an idealised servo, so trust standing torques more than walking peaks.

Worlds: `flat`, `slope` (8°), `rough` (40 slabs, 1–3 cm, fixed layout). `level:=true` (or `ros2 param set /gait_node level true`) feeds the IMU tilt back into the body pose. On the 8° slope the body goes from −8.0° to 0.0°.

## CAN without hardware

`fake_legs.py` plays the six leg nodes on a virtual CAN bus, from `protocol/vector.dbc`. It follows commands at the servo speed limit, reports state and status, and faults if SYNC stops.

```sh
sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
pip install python-can cantools
ros2 run vector_hw fake_legs.py --channel vcan0
ros2 launch vector_bringup robot.launch.py hardware:=can can_interface:=vcan0
candump vcan0 | cantools decode ../../protocol/vector.dbc      # watch the traffic
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
