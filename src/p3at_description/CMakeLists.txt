cmake_minimum_required(VERSION 3.8)
project(p3at_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes rviz models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
