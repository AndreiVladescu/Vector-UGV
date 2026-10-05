cmake_minimum_required(VERSION 3.20)

# cmake -S firmware/leg-node/stm32 -B build/leg-stm32 [-DBOARD=nucleo]
set(STM32C0 ${CMAKE_CURRENT_LIST_DIR}/../../stm32c0)
set(CMAKE_TOOLCHAIN_FILE ${STM32C0}/arm-gcc.cmake CACHE FILEPATH "")
project(leg_node_stm32 C ASM)

set(BOARD side CACHE STRING "side (side board, 40 MHz crystal) or nucleo (NUCLEO-C092RC)")
set(SIDE_HSE 40000000 CACHE STRING "side board crystal in Hz (25 or 40 MHz both work)")
option(NUCLEO_CAN_PA11 "Nucleo: FDCAN on PA11/PA12 like the boards (jumper PA11-PD0, PA12-PD1)" OFF)
set(LEG_ID 1 CACHE STRING "CAN node on the Nucleo (the side board reads its jumpers)")
if(BOARD STREQUAL nucleo)
  set(HSE 48000000)
  set(BOARD_DEF BOARD_NUCLEO LEG_ID=${LEG_ID})
  if(NUCLEO_CAN_PA11)
    list(APPEND BOARD_DEF NUCLEO_CAN_PA11)
  endif()
else()
  set(HSE ${SIDE_HSE})
  set(BOARD_DEF BOARD_SIDE)
endif()

set(NODE_DIR ${CMAKE_CURRENT_LIST_DIR}/..)
include(${STM32C0}/stm32c0.cmake)

firmware(leg-node app.ld 0x4000 main.c
  ${NODE_DIR}/src/leg.c ${NODE_DIR}/src/joint.c ${NODE_DIR}/src/filter.c ${NODE_DIR}/src/config.c
  ${NODE_DIR}/src/vl53l1x.c ${COMMON}/vector_can.c ${COMMON}/boot.c)
bootloader()
