cmake_minimum_required(VERSION 3.20)

# cmake -S firmware/power-node/stm32 -B build/power-stm32 [-DBOARD=nucleo]
set(STM32C0 ${CMAKE_CURRENT_LIST_DIR}/../../stm32c0)
set(CMAKE_TOOLCHAIN_FILE ${STM32C0}/arm-gcc.cmake CACHE FILEPATH "")
project(power_node_stm32 C ASM)

set(BOARD power CACHE STRING "power (power board, 40 MHz crystal) or nucleo (NUCLEO-C092RC)")
if(BOARD STREQUAL nucleo)
  set(HSE 48000000)
  set(BOARD_DEF BOARD_NUCLEO)
else()
  set(HSE 40000000)
  set(BOARD_DEF BOARD_POWER)
endif()

set(NODE_DIR ${CMAKE_CURRENT_LIST_DIR}/..)
include(${STM32C0}/stm32c0.cmake)

firmware(power-node app.ld 0x4000 main.c
  ${NODE_DIR}/src/power.c ${NODE_DIR}/src/bq76942.c ${NODE_DIR}/src/bq25798.c ${NODE_DIR}/src/gauge.c
  ${NODE_DIR}/src/config.c ${COMMON}/vector_can.c ${COMMON}/boot.c)
bootloader()
