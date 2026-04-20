cmake_minimum_required(VERSION 3.8)
project(p3at)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(DIRECTORY
  launch
  robots
  worlds
  meshes
  config
  images
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  src/waypoint_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
