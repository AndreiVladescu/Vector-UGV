cmake_minimum_required(VERSION 3.16)
project(vector_gait)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Core library, no ROS dependencies.
add_library(vector_core src/kinematics.cpp src/gait.cpp src/leveler.cpp)
target_include_directories(vector_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(gait_node src/gait_node.cpp)
target_link_libraries(gait_node vector_core
  rclcpp::rclcpp ${geometry_msgs_TARGETS} ${std_msgs_TARGETS} ${visualization_msgs_TARGETS} ${sensor_msgs_TARGETS} ${control_msgs_TARGETS} ${std_srvs_TARGETS} tf2_ros::tf2_ros)

install(TARGETS vector_core EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
# Offline report: speeds, limits, stability, torques. Links yaml-cpp only.
add_executable(gait_report src/gait_report.cpp)
target_link_libraries(gait_report vector_core yaml-cpp::yaml-cpp)

install(TARGETS gait_node gait_report DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_kinematics test/test_kinematics.cpp)
  target_link_libraries(test_kinematics vector_core)
  ament_add_gtest(test_gait test/test_gait.cpp)
  target_link_libraries(test_gait vector_core)
  ament_add_gtest(test_leveler test/test_leveler.cpp)
  target_link_libraries(test_leveler vector_core)
endif()

ament_export_targets(export_${PROJECT_NAME})
ament_package()
