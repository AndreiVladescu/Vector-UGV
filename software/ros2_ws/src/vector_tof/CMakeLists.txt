cmake_minimum_required(VERSION 3.16)
project(vector_tof C CXX)

set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# ST's VL53L8CX ULD (BSD-3-Clause) from ST's X-CUBE-TOF1 on GitHub, pinned by commit and
# hash. vl53l8cx_buffers.h holds the ~85 KB sensor firmware uploaded at every start.
set(ULD_URL https://raw.githubusercontent.com/STMicroelectronics/x-cube-tof1/c12709c3ac802d91dd2a215178f53cb94c024709/Drivers/BSP/Components/vl53l8cx/modules)
set(ULD_DIR ${CMAKE_BINARY_DIR}/uld)
foreach(f
    "vl53l8cx_api.c|7e370b8018b2d4bf1354e7bde3b85d4030707e73676f2825837cb8958ccb6cc1"
    "vl53l8cx_api.h|c7fc8b7aea8bf7f1e74953339990ad4ec08e7da5c2d77b1cb55438279a32e492"
    "vl53l8cx_buffers.h|3b834927e23e24eb0fe540132b4413b8df815f347abcfbe5f9a0dde4cbc84109")
  string(REPLACE "|" ";" parts ${f})
  list(GET parts 0 name)
  list(GET parts 1 hash)
  if(NOT EXISTS ${ULD_DIR}/${name})
    file(DOWNLOAD ${ULD_URL}/${name} ${ULD_DIR}/${name} EXPECTED_HASH SHA256=${hash})
  endif()
endforeach()

# the ULD with our Linux i2c-dev platform layer (uld/platform.h replaces ST's)
add_library(vl53l8cx_uld STATIC ${ULD_DIR}/vl53l8cx_api.c uld/platform.c)
target_include_directories(vl53l8cx_uld PUBLIC uld ${ULD_DIR})
set_target_properties(vl53l8cx_uld PROPERTIES C_STANDARD 11 POSITION_INDEPENDENT_CODE ON)
target_compile_options(vl53l8cx_uld PRIVATE -w)  # ST's code, not ours to fix

add_executable(tof_front src/tof_front.cpp)
target_link_libraries(tof_front vl53l8cx_uld rclcpp::rclcpp ${sensor_msgs_TARGETS})
target_compile_options(tof_front PRIVATE -Wall -Wextra)
install(TARGETS tof_front DESTINATION lib/${PROJECT_NAME})

ament_package()
