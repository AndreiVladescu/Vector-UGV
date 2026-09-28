# Shared STM32C092 build: ST's HAL and CMSIS at pinned tags, the version header, and
# firmware() for an application or the bootloader. A node's stm32/CMakeLists.txt sets
# NODE_DIR, HSE and BOARD_DEF, then includes this.

# fw_version.h with the short git hash, bit 28 set when the firmware tree has local
# changes; regenerated on every build, rewritten only when it changes
add_custom_target(fw_version ALL
  COMMAND ${CMAKE_COMMAND} -DSRC=${NODE_DIR} -DOUT=${CMAKE_BINARY_DIR}/conf/fw_version.h
    -P ${STM32C0}/version.cmake
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

set(COMMON ${STM32C0}/../common)
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
  ${CMAKE_CURRENT_SOURCE_DIR} ${STM32C0} ${NODE_DIR}/src ${COMMON} ${CMAKE_BINARY_DIR}/conf
  ${cmsis_core_SOURCE_DIR}/Include ${cmsis_device_c0_SOURCE_DIR}/Include ${c0_hal_SOURCE_DIR}/Inc)
target_compile_definitions(st_hal PUBLIC STM32C092xx USE_HAL_DRIVER HSE_VALUE=${HSE}U ${BOARD_DEF})
target_compile_options(st_hal PUBLIC ${CPU} -Os -g -ffunction-sections -fdata-sections -Wall
  $<$<COMPILE_LANGUAGE:C>:-std=gnu11>)

# name, linker script (app.ld or bootloader.ld), flash offset of the vector table, sources.
# Every node has board.c (pins, board_gpio, board_node_id) in its stm32/ directory.
function(firmware name ld vtor)
  add_executable(${name} ${ARGN} ${CMAKE_CURRENT_SOURCE_DIR}/board.c ${STM32C0}/mcu.c
    ${cmsis_device_c0_SOURCE_DIR}/Source/Templates/gcc/startup_stm32c092xx.s
    ${cmsis_device_c0_SOURCE_DIR}/Source/Templates/system_stm32c0xx.c)
  target_compile_definitions(${name} PRIVATE VECT_TAB_OFFSET=${vtor})
  target_link_options(${name} PRIVATE ${CPU} -T${STM32C0}/${ld} -L${STM32C0}
    -Wl,--gc-sections -Wl,-Map=${name}.map --specs=nano.specs --specs=nosys.specs -Wl,--print-memory-usage)
  target_link_libraries(${name} PRIVATE st_hal m)
  add_dependencies(${name} fw_version)
  add_custom_command(TARGET ${name} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary ${name} ${name}.bin
    COMMAND ${CMAKE_SIZE} ${name})
endfunction()

function(bootloader)
  firmware(bootloader bootloader.ld 0x0 ${STM32C0}/bootloader.c ${COMMON}/boot.c)
endfunction()
