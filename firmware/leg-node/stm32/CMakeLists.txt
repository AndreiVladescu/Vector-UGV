cmake_minimum_required(VERSION 3.20)

# cmake -S firmware/leg-node/stm32 -B build/leg-stm32 -DCMAKE_TOOLCHAIN_FILE=arm-gcc.cmake [-DBOARD=nucleo]
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_LIST_DIR}/arm-gcc.cmake CACHE FILEPATH "")
project(leg_node_stm32 C ASM)

set(BOARD side CACHE STRING "side (side board, 40 MHz crystal) or nucleo (NUCLEO-C092RC)")
set(SIDE_HSE 40000000 CACHE STRING "side board crystal in Hz (25 or 40 MHz both work)")
set(LEG_ID 1 CACHE STRING "CAN node on the Nucleo (the side board reads its jumpers)")
if(BOARD STREQUAL nucleo)
  set(HSE 48000000)
  set(BOARD_DEF BOARD_NUCLEO LEG_ID=${LEG_ID})
else()
  set(HSE ${SIDE_HSE})
  set(BOARD_DEF BOARD_SIDE)
endif()

include(FetchContent)
set(FETCHCONTENT_QUIET ON)
FetchContent_Declare(cmsis_core GIT_REPOSITORY https://github.com/STMicroelectronics/cmsis-core.git
  GIT_TAG v5.9.0_20250520 GIT_SHALLOW TRUE)
FetchContent_Declare(cmsis_device_c0 GIT_REPOSITORY https://github.com/STMicroelectronics/cmsis-device-c0.git
  GIT_TAG v1.4.1 GIT_SHALLOW TRUE)
FetchContent_Declare(c0_hal GIT_REPOSITORY https://github.com/STMicroelectronics/stm32c0xx-hal-driver.git
  GIT_TAG v1.4.1 GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(cmsis_core cmsis_device_c0 c0_hal)

configure_file(${c0_hal_SOURCE_DIR}/Inc/stm32c0xx_hal_conf_template.h
  ${CMAKE_BINARY_DIR}/conf/stm32c0xx_hal_conf.h COPYONLY)

set(LEG ${CMAKE_CURRENT_LIST_DIR}/..)
set(HAL ${c0_hal_SOURCE_DIR}/Src)
add_executable(leg-node
  main.c
  ${LEG}/src/leg.c ${LEG}/src/joint.c ${LEG}/src/filter.c ${LEG}/src/config.c ${LEG}/src/vl53l1x.c ${LEG}/../common/vector_can.c
  ${cmsis_device_c0_SOURCE_DIR}/Source/Templates/gcc/startup_stm32c092xx.s
  ${cmsis_device_c0_SOURCE_DIR}/Source/Templates/system_stm32c0xx.c
  ${HAL}/stm32c0xx_hal.c ${HAL}/stm32c0xx_hal_cortex.c ${HAL}/stm32c0xx_hal_rcc.c ${HAL}/stm32c0xx_hal_rcc_ex.c
  ${HAL}/stm32c0xx_hal_gpio.c ${HAL}/stm32c0xx_hal_dma.c ${HAL}/stm32c0xx_hal_dma_ex.c
  ${HAL}/stm32c0xx_hal_adc.c ${HAL}/stm32c0xx_hal_adc_ex.c ${HAL}/stm32c0xx_hal_tim.c ${HAL}/stm32c0xx_hal_tim_ex.c
  ${HAL}/stm32c0xx_hal_fdcan.c ${HAL}/stm32c0xx_hal_flash.c ${HAL}/stm32c0xx_hal_flash_ex.c
  ${HAL}/stm32c0xx_hal_i2c.c ${HAL}/stm32c0xx_hal_i2c_ex.c ${HAL}/stm32c0xx_hal_iwdg.c ${HAL}/stm32c0xx_hal_pwr.c ${HAL}/stm32c0xx_hal_pwr_ex.c)
target_include_directories(leg-node PRIVATE
  ${CMAKE_CURRENT_LIST_DIR} ${LEG}/src ${LEG}/../common ${CMAKE_BINARY_DIR}/conf
  ${cmsis_core_SOURCE_DIR}/Include ${cmsis_device_c0_SOURCE_DIR}/Include ${c0_hal_SOURCE_DIR}/Inc)
target_compile_definitions(leg-node PRIVATE STM32C092xx USE_HAL_DRIVER HSE_VALUE=${HSE}U ${BOARD_DEF})

set(CPU -mcpu=cortex-m0plus -mthumb)
target_compile_options(leg-node PRIVATE ${CPU} -Os -g -ffunction-sections -fdata-sections -Wall
  $<$<COMPILE_LANGUAGE:C>:-std=gnu11>)
target_link_options(leg-node PRIVATE ${CPU} -T${CMAKE_CURRENT_LIST_DIR}/stm32c092kc.ld
  -Wl,--gc-sections -Wl,-Map=leg-node.map --specs=nano.specs --specs=nosys.specs -Wl,--print-memory-usage)
target_link_libraries(leg-node PRIVATE m)

add_custom_command(TARGET leg-node POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O binary leg-node leg-node.bin
  COMMAND ${CMAKE_SIZE} leg-node)
