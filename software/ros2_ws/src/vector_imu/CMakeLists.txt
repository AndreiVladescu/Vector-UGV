cmake_minimum_required(VERSION 3.16)
project(vector_imu)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/imu_node.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_fusion test/test_fusion.py)
  ament_add_pytest_test(test_drivers test/test_drivers.py)
  find_package(launch_testing_ament_cmake REQUIRED)
  # imu_node.py on simulated chips
  add_launch_test(test/test_imu_node.py TIMEOUT 60)
  set_tests_properties(test_test_imu_node.py PROPERTIES ENVIRONMENT "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST")
endif()

ament_package()
