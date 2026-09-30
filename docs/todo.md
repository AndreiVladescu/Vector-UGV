# To do

## Software

- **3D robot on the operator page.** Once the Fusion 360 model exists: export it with fusion2urdf (body and every coxa / femur / tibia as its own component, revolute joints on the real axes, zero pose and signs as in `vector_gait` kinematics), decimate the meshes to a few MB, replace the placeholder geometry in `vector.urdf.xacro` (rviz, Gazebo and Foxglove then show the real robot, and Gazebo gets real masses), and render it on the page with three.js + urdf-loader, driven by `/api/pose`. Keep the 2D drawing as the offline fallback.
- Check the VL53L8CX zone orientation (`flip_x`, `flip_y` in `tof_front`) with a hand in one corner.
- Calibrate the OV5647 (checkerboard): the ground projection and the ToF fusion use datasheet angles.
- LTE (A7670E): APN and USB mode on the modem, then a drive over LTE with the operator page.
- GitHub Actions: firmware tests, ARM builds, colcon tests, protocol checks.

## On hold

- Power board firmware extras (5 V / VBAT ADC, ALERT pin, OTP): the power board, its BOM and the charging scheme are being reworked.
- IMU and compass drivers: the parts aren't chosen yet. `nav:=true` on the robot needs the compass for heading.

## Mechanical

- Leg geometry and mass: 2.5 kg is too much for MG996Rs with the current legs (`gait_report`).
