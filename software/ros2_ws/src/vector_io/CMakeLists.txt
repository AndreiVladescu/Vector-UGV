cmake_minimum_required(VERSION 3.16)
project(vector_io)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/io_bridge.py scripts/io_flash.py scripts/sim_io.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_link test/test_link.py)
  ament_add_pytest_test(test_lidar test/test_lidar.py)
  find_package(launch_testing_ament_cmake REQUIRED)
  # a fake IO MCU on a pseudo-terminal, through io_bridge.py and elrs.py
  add_launch_test(test/test_io_bridge.py TIMEOUT 90)
  set_tests_properties(test_test_io_bridge.py PROPERTIES ENVIRONMENT "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST")
endif()

ament_package()
