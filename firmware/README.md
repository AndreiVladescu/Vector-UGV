# Firmware

STM32C092 for all nodes, bare-metal on ST's HAL, built with CMake and arm-none-eabi-gcc. Each node keeps its logic in plain C under `src/`, tested on the PC against simulated hardware, and a thin `stm32/` layer for the chip.

- `common/`: CAN frames (`vector_can`, matching `protocol/vector.dbc`) and the bootloader protocol (`boot`)
- `stm32c0/`: shared by every node: clocks and FDCAN (`mcu.c`), the CAN bootloader, flash map and linker scripts, the build helper (`stm32c0.cmake`), `flash_swd.sh`
- `leg-node/`: servo PWM, pot and current ADC, ToF, wake, calibration, protection, self-test
- `power-node/`: BMS setup, charger, power states, e-stop (not started)

First flash over SWD with `stm32c0/flash_swd.sh`, then over CAN with `leg_config.py flash`. A node with a broken application can still be reached through the bootloader's 200 ms window after reset.
