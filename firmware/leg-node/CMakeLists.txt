cmake_minimum_required(VERSION 3.16)
project(leg_node C)

# Host build: the leg logic, its tests and a simulator on SocketCAN.
# The STM32 build lives in stm32/ (CubeMX project) and links the same sources.

set(CMAKE_C_STANDARD 11)
add_compile_options(-Wall -Wextra -Werror=implicit-function-declaration)

add_library(leg_core STATIC
  src/leg.c src/joint.c src/filter.c src/config.c src/vl53l1x.c
  ../common/vector_can.c)
target_include_directories(leg_core PUBLIC src ../common)
target_link_libraries(leg_core PUBLIC m)

add_library(leg_sim STATIC sim/servo_sim.c sim/leg_sim.c sim/tof_sim.c)
target_include_directories(leg_sim PUBLIC sim)
target_link_libraries(leg_sim PUBLIC leg_core)

enable_testing()
add_executable(test_leg test/test_leg.c)
target_link_libraries(test_leg leg_sim)
add_test(NAME test_leg COMMAND test_leg)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  add_executable(sim_legs sim/sim_legs.c)
  target_link_libraries(sim_legs leg_sim)
endif()
