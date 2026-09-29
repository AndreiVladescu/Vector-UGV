cmake_minimum_required(VERSION 3.16)
project(power_node C)

# Host build: the power logic, its tests and a simulator on SocketCAN.
# The STM32 build lives in stm32/ and links the same sources.

set(CMAKE_C_STANDARD 11)
add_compile_options(-Wall -Wextra -Werror=implicit-function-declaration)

add_library(power_core STATIC
  src/power.c src/bq76942.c src/bq25798.c src/gauge.c src/config.c
  ../common/vector_can.c ../common/boot.c)
target_include_directories(power_core PUBLIC src ../common)
target_link_libraries(power_core PUBLIC m)

add_library(power_sim STATIC sim/bq76942_sim.c sim/bq25798_sim.c sim/power_sim.c)
target_include_directories(power_sim PUBLIC sim)
target_link_libraries(power_sim PUBLIC power_core)

enable_testing()
add_executable(test_power test/test_power.c)
target_link_libraries(test_power power_sim)
add_test(NAME test_power COMMAND test_power)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  add_executable(sim_power sim/sim_power.c)
  target_link_libraries(sim_power power_sim)
endif()
