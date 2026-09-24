# Firmware

STM32C092 for all nodes. CubeMX (`.ioc`) generates a CMake project, built with arm-none-eabi-gcc and worked on in VS Code.

- `common/`: CAN code generated from `protocol/vector.dbc`, shared helpers
- `bootloader/`: CAN bootloader based on ST OpenBootloader
- `leg-node/`: servo PWM, pot and current ADC, ToF, soft-start, watchdog
- `power-node/`: BMS setup, charger, power states

Flash over CAN from the CM5 (`software/tools/`). If a node is bricked, use SWD through OpenOCD on the CM5 GPIO header.
