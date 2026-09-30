cmake_minimum_required(VERSION 3.16)
project(vector_link)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/elrs.py scripts/link_manager.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_crsf test/test_crsf.py)
  ament_add_pytest_test(test_manager test/test_manager.py)
  ament_add_pytest_test(test_net test/test_net.py)
  find_package(launch_testing_ament_cmake REQUIRED)
  # a radio on a pseudo-terminal, through elrs.py and the link manager
  add_launch_test(test/test_radio.py TIMEOUT 90)
  set_tests_properties(test_test_radio.py PROPERTIES ENVIRONMENT "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST")
endif()

ament_package()
