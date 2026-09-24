# Software

Runs on the Pi 5 during development and on the CM5 later: Raspberry Pi OS, with ROS 2 Jazzy in Docker.

Planned packages in `ros2_ws/src/`:
- `vector_description`: URDF, meshes
- `vector_hw`: ros2_control over SocketCAN
- `vector_gait`: wraps `libvector` (kinematics and gait, no ROS inside)
- `vector_perception`: ToF mapping, detection and trail segmentation
- `vector_links`: ELRS teleop, link failover
- `vector_bringup`: launch files, params

`tools/` holds bring-up scripts (python-can, cantools) and the CAN flasher.
