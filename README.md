# V.E.C.T.O.R.

An 18-DOF hexapod research platform: a smart controller per leg on CAN, a Raspberry Pi CM5 carrier, camera and SDR payload, ELRS / Wi-Fi / 4G links. Built for the [PCBWay 9th Project Design Contest](https://www.pcbway.com/activity/9th-project-design-contest.html).

- `docs/`: spec, connector pinouts, power budget, servo names and calibration
- `hardware/`: one KiCad project per board, shared library in `lib/`
- `mechanical/`: Fusion 360 exports (STEP, STL), harness drawings
- `firmware/`: STM32C092 code for the leg and power nodes, CAN bootloader
- `software/`: CM5 / Pi 5 side: ROS 2 workspace, tools, Docker
- `protocol/vector.dbc`: CAN message definitions, used by firmware and host
- `test-tools/`: bench sketches, e.g. `servo_test` (Arduino Uno, set a servo angle over serial and read the pot)

Start with `docs/spec.md`.

## Try it without hardware

Everything below runs on a PC with Ubuntu 24.04 or Linux Mint 22 and ROS 2 Jazzy (install commands in [`software/README.md`](software/README.md#desktop-setup-mint-223--ubuntu-2404)). The leg nodes run their real firmware logic against simulated servos on a virtual CAN bus.

```sh
cd software/ros2_ws
source /opt/ros/jazzy/setup.zsh                 # or setup.bash
colcon build --symlink-install && source install/setup.zsh

# virtual CAN bus (again after every reboot)
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0

ros2 run vector_hw sim_legs vcan0 --calibrated --limits -45.8,45.8,-80.2,80.2,-149,17  # terminal 1: six legs
ros2 launch vector_bringup robot.launch.py hardware:=can can_interface:=vcan0   # terminal 2: robot + rviz
ros2 run teleop_twist_keyboard teleop_twist_keyboard                           # terminal 3: drive (i, j, l, J, L, t, b, k)
candump vcan0 | cantools decode ../../protocol/vector.dbc                      # optional: watch the bus
```

Each new terminal needs the two `source` lines. Stop `sim_legs` while the robot runs and the hardware drops out after 100 ms, the same as it would with a dead leg. Without `--calibrated` the legs start like fresh boards; `leg_config.py` calibrates them (see `software/README.md`).

Without CAN: `robot.launch.py` alone (mock joints), or `sim_gazebo.launch.py world:=rough level:=true` for physics.
