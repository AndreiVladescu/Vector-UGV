cmake_minimum_required(VERSION 3.16)
project(vector_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz worlds DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_kinematic.py TIMEOUT 120)
  # Needs a vcan0 interface, skips itself otherwise
  add_launch_test(test/test_can_sim_legs.py TIMEOUT 120 LABELS can)
  add_launch_test(test/test_leg_config.py TIMEOUT 180 LABELS can)
  # Needs Gazebo; slow, run with: colcon test --ctest-args -L gazebo
  add_launch_test(test/test_sim_gazebo.py TIMEOUT 300 LABELS gazebo)
  add_launch_test(test/test_gazebo_slope.py TIMEOUT 300 LABELS gazebo)
  add_launch_test(test/test_gazebo_rough.py TIMEOUT 300 LABELS gazebo)
endif()

ament_package()
