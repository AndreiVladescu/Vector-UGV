cmake_minimum_required(VERSION 3.20)

# cmake -S firmware/io-node/stm32 -B build/io-stm32 [-DBOARD=nucleo]
set(STM32C0 ${CMAKE_CURRENT_LIST_DIR}/../../stm32c0)
set(CMAKE_TOOLCHAIN_FILE ${STM32C0}/arm-gcc.cmake CACHE FILEPATH "")
project(io_node_stm32 C ASM)

# no crystal and no CAN on the IO MCU; HSE_VALUE only keeps the HAL's arithmetic happy
set(HSE 48000000)
set(BOARD carrier CACHE STRING "carrier (CM5 carrier) or nucleo (NUCLEO-C092RC, link on the ST-LINK VCP)")
if(BOARD STREQUAL nucleo)
  set(BOARD_DEF BOARD_NUCLEO MCU_NO_HSE)
else()
  set(BOARD_DEF BOARD_IO MCU_NO_HSE)
endif()
set(NODE_DIR ${CMAKE_CURRENT_LIST_DIR}/..)
include(${STM32C0}/stm32c0.cmake)

# linked at the start of flash: no CAN bootloader, ST's ROM bootloader on USART1 instead
firmware(io-node standalone.ld 0x0 main.c
  ${NODE_DIR}/src/io.c ${NODE_DIR}/src/link.c ${NODE_DIR}/src/frames.c ${NODE_DIR}/src/sx1276.c)
