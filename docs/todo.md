# To do

## Software

- **3D robot on the operator page.** Once the Fusion 360 model exists: export it with fusion2urdf (body and every coxa / femur / tibia as its own component, revolute joints on the real axes, zero pose and signs as in `vector_gait` kinematics), decimate the meshes to a few MB, replace the placeholder geometry in `vector.urdf.xacro` (rviz, Gazebo and Foxglove then show the real robot, and Gazebo gets real masses), and render it on the page with three.js + urdf-loader, driven by `/api/pose`. Keep the 2D drawing as the offline fallback.
- Check the VL53L8CX zone orientation (`flip_x`, `flip_y` in `tof_front`) with a hand in one corner.
- Calibrate the OV5647 (checkerboard): the ground projection and the ToF fusion use datasheet angles.
- LTE (A7670E on UART4): APN on the modem, PPP over the UART, then a drive over LTE with the operator page.
- On the carrier: set `imu_axes`, `mag_axes` from the board layouts and calibrate the compass in the robot; set `lidar_yaw` and `lidar_clockwise` once the LDS01RR is mounted (an object on one side shows the direction).
- IO MCU bring-up: check the ROM bootloader jump and `io_flash.py` on the real chip, the LDS01RR motor loop on the real motor and the buzzer frequency.

## Hardware

- Carrier: J602 to JST-XH 4-pin and add J-LIDARMOT with its FET and diode for the LDS01RR (see the spec).
- Decide whether J-SYSCTL and the power board's J-CAN merge into one 10-pin JST-GH to the carrier.
- Side boards: add the e-stop diode (BAT54J, BUCK_EN → ESTOP_N) and the 1k in series with PB0 on each leg cell.
- Carrier: pick the 4.7 µH inductor for the LTE buck, then layout (CM5 placement and the 90/100 Ω pairs from the CM5 IO board).
- XT60 / XT30 aren't on Mouser: order from TME.

## Mechanical

- Leg geometry and mass: 2.5 kg is too much for MG996Rs with the current legs (`gait_report`).
