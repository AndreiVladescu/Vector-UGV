cmake_minimum_required(VERSION 3.16)
project(vector_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf config DESTINATION share/${PROJECT_NAME})
# the CAD meshes (Git LFS, ~90 MB) are for rviz on the desktop; the robot image leaves them out
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/meshes)
  install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})
endif()

ament_package()
