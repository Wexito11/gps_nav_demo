cmake_minimum_required(VERSION 3.8)
project(gps_nav_demo)
find_package(ament_cmake REQUIRED)

install(DIRECTORY launch urdf worlds config rviz
  DESTINATION share/${PROJECT_NAME})

file(GLOB NODE_SCRIPTS scripts/*.py)
if(NODE_SCRIPTS)
  install(PROGRAMS ${NODE_SCRIPTS} DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()
