# To do

## Software

- Set materials in Fusion so the CAD masses mean something (the export is scaled by volume), then rerun `tools/cad_urdf.py` with the real total.
- Check the VL53L8CX zone orientation (`flip_x`, `flip_y` in `tof_front`) with a hand in one corner.
- Calibrate the OV5647 (checkerboard): the ground projection and the ToF fusion use datasheet angles.
- LTE (A7670E on UART4): APN on the modem, PPP over the UART, then a drive over LTE with the operator page.
- On the carrier: set `imu_axes`, `mag_axes` from the board layouts and calibrate the compass in the robot; set `lidar_yaw` and `lidar_clockwise` once the LDS01RR is mounted (an object on one side shows the direction).
- IO MCU bring-up: check the ROM bootloader jump and `io_flash.py` on the real chip, the LDS01RR motor loop on the real motor and the buzzer frequency.

## Hardware

- Schematics for the 10-pin J-SYSCTL (power board J101 + J102, carrier J502) and the J-USBC pigtail header (power board J301), per `interfaces.md`.
- Carrier: layout (CM5 placement and the 90/100 Ω pairs from the CM5 IO board).
- XT60 / XT30 aren't on Mouser: order from TME.
