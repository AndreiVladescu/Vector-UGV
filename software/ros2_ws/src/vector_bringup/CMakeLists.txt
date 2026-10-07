cmake_minimum_required(VERSION 3.16)
project(vector_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/sim_tof.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_kinematic.py TIMEOUT 120)
  # the operator page demo: mock robot, simulated IO MCU and IMU (uses port 8080 and /tmp/vector-io-sim)
  add_launch_test(test/test_demo.py TIMEOUT 150)
  # Needs a vcan0 interface, skips itself otherwise
  add_launch_test(test/test_can_sim_legs.py TIMEOUT 240 LABELS can)
  add_launch_test(test/test_leg_config.py TIMEOUT 180 LABELS can)
  add_launch_test(test/test_can_sim_power.py TIMEOUT 120 LABELS can)
  add_launch_test(test/test_calibration_page.py TIMEOUT 120 LABELS can)
  # Needs Gazebo; slow, run with: colcon test --ctest-args -L gazebo
  add_launch_test(test/test_sim_gazebo.py TIMEOUT 300 LABELS gazebo)
  add_launch_test(test/test_gazebo_slope.py TIMEOUT 300 LABELS gazebo)
  add_launch_test(test/test_gazebo_rough.py TIMEOUT 300 LABELS gazebo)
  add_launch_test(test/test_gazebo_gnss.py TIMEOUT 400 LABELS gazebo)
  add_launch_test(test/test_gazebo_sensors.py TIMEOUT 300 LABELS gazebo)
  # DDS on this machine only: a robot (or the Pi) on the LAN publishes its own
  # robot_description, cmd_vel and so on, which would end up in these tests
  get_property(tests DIRECTORY PROPERTY TESTS)
  set_tests_properties(${tests} PROPERTIES ENVIRONMENT "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST")
endif()

ament_package()
