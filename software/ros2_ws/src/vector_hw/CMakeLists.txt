cmake_minimum_required(VERSION 3.16)
project(vector_hw)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Protocol + SocketCAN, no ROS.
add_library(vector_can STATIC src/protocol.cpp src/socketcan.cpp)
set_target_properties(vector_can PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(vector_can PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

add_library(vector_hw SHARED src/vector_system.cpp)
target_link_libraries(vector_hw vector_can
  hardware_interface::hardware_interface pluginlib::pluginlib rclcpp::rclcpp rclcpp_lifecycle::rclcpp_lifecycle)

pluginlib_export_plugin_description_file(hardware_interface vector_hw.xml)

install(TARGETS vector_hw vector_can LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
# Bench tool: simulated leg nodes on a (v)can interface, plus the DBC it reads.
install(PROGRAMS scripts/fake_legs.py DESTINATION lib/${PROJECT_NAME})
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../../../protocol/vector.dbc DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol vector_can)
endif()

ament_package()
