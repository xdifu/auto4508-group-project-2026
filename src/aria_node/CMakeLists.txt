cmake_minimum_required(VERSION 3.8)
project(aria_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(ARIACODA_HINTS "")
if(DEFINED ENV{ARIACODA_PREFIX})
  list(APPEND ARIACODA_HINTS "$ENV{ARIACODA_PREFIX}")
endif()
if(DEFINED ENV{ARIACODA_ROOT})
  list(APPEND ARIACODA_HINTS "$ENV{ARIACODA_ROOT}")
endif()

find_path(
  ARIA_INCLUDE_DIR
  NAMES Aria/Aria.h
  HINTS ${ARIACODA_HINTS}
  PATH_SUFFIXES include
  PATHS /usr/local /usr
)

find_library(
  ARIA_LIBRARY
  NAMES Aria libAria.so
  HINTS ${ARIACODA_HINTS}
  PATH_SUFFIXES lib lib64
  PATHS /usr/local /usr
)

if(NOT ARIA_INCLUDE_DIR OR NOT ARIA_LIBRARY)
  message(FATAL_ERROR "AriaCoda was not found. Install it and set ARIACODA_PREFIX if not installed under /usr/local.")
endif()

add_executable(ariaNode src/ariaNode.cpp)
target_include_directories(ariaNode PRIVATE ${ARIA_INCLUDE_DIR})
target_link_libraries(ariaNode ${ARIA_LIBRARY})
ament_target_dependencies(
  ariaNode
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_ros
)
target_compile_features(ariaNode PRIVATE cxx_std_17)

install(TARGETS ariaNode DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
