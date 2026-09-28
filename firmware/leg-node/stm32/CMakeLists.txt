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

# fw_version.h with the short git hash, bit 28 set when the firmware tree has local
# changes; regenerated on every build, rewritten only when it changes
add_custom_target(fw_version ALL
  COMMAND ${CMAKE_COMMAND} -DSRC=${CMAKE_CURRENT_LIST_DIR}/.. -DOUT=${CMAKE_BINARY_DIR}/conf/fw_version.h
    -P ${CMAKE_CURRENT_LIST_DIR}/version.cmake
  BYPRODUCTS ${CMAKE_BINARY_DIR}/conf/fw_version.h)

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
set(COMMON ${LEG}/../common)
set(HAL ${c0_hal_SOURCE_DIR}/Src)
set(CPU -mcpu=cortex-m0plus -mthumb)

add_library(st_hal STATIC
  ${HAL}/stm32c0xx_hal.c ${HAL}/stm32c0xx_hal_cortex.c ${HAL}/stm32c0xx_hal_rcc.c ${HAL}/stm32c0xx_hal_rcc_ex.c
  ${HAL}/stm32c0xx_hal_gpio.c ${HAL}/stm32c0xx_hal_dma.c ${HAL}/stm32c0xx_hal_dma_ex.c
  ${HAL}/stm32c0xx_hal_adc.c ${HAL}/stm32c0xx_hal_adc_ex.c ${HAL}/stm32c0xx_hal_tim.c ${HAL}/stm32c0xx_hal_tim_ex.c
  ${HAL}/stm32c0xx_hal_fdcan.c ${HAL}/stm32c0xx_hal_flash.c ${HAL}/stm32c0xx_hal_flash_ex.c
  ${HAL}/stm32c0xx_hal_i2c.c ${HAL}/stm32c0xx_hal_i2c_ex.c ${HAL}/stm32c0xx_hal_iwdg.c
  ${HAL}/stm32c0xx_hal_pwr.c ${HAL}/stm32c0xx_hal_pwr_ex.c)
target_include_directories(st_hal PUBLIC
  ${CMAKE_CURRENT_LIST_DIR} ${LEG}/src ${COMMON} ${CMAKE_BINARY_DIR}/conf
  ${cmsis_core_SOURCE_DIR}/Include ${cmsis_device_c0_SOURCE_DIR}/Include ${c0_hal_SOURCE_DIR}/Inc)
target_compile_definitions(st_hal PUBLIC STM32C092xx USE_HAL_DRIVER HSE_VALUE=${HSE}U ${BOARD_DEF})
target_compile_options(st_hal PUBLIC ${CPU} -Os -g -ffunction-sections -fdata-sections -Wall
  $<$<COMPILE_LANGUAGE:C>:-std=gnu11>)

# name, linker script, flash offset of the vector table, sources
function(firmware name ld vtor)
  add_executable(${name} ${ARGN} board.c
    ${cmsis_device_c0_SOURCE_DIR}/Source/Templates/gcc/startup_stm32c092xx.s
    ${cmsis_device_c0_SOURCE_DIR}/Source/Templates/system_stm32c0xx.c)
  target_compile_definitions(${name} PRIVATE VECT_TAB_OFFSET=${vtor})
  target_link_options(${name} PRIVATE ${CPU} -T${CMAKE_CURRENT_LIST_DIR}/${ld} -L${CMAKE_CURRENT_LIST_DIR}
    -Wl,--gc-sections -Wl,-Map=${name}.map --specs=nano.specs --specs=nosys.specs -Wl,--print-memory-usage)
  target_link_libraries(${name} PRIVATE st_hal m)
  add_dependencies(${name} fw_version)
  add_custom_command(TARGET ${name} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary ${name} ${name}.bin
    COMMAND ${CMAKE_SIZE} ${name})
endfunction()

firmware(leg-node stm32c092kc.ld 0x4000 main.c
  ${LEG}/src/leg.c ${LEG}/src/joint.c ${LEG}/src/filter.c ${LEG}/src/config.c ${LEG}/src/vl53l1x.c
  ${COMMON}/vector_can.c ${COMMON}/boot.c)
firmware(bootloader bootloader.ld 0x0 bootloader.c ${COMMON}/boot.c)
