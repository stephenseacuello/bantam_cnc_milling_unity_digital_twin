cmake_minimum_required(VERSION 3.8)
project(miracle_microros)

# Default to C11 / C++17
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Note: micro_ros_setup and micro_ros_agent are runtime-only dependencies
# managed outside of colcon (PlatformIO handles MCU firmware builds).
# miracle_msgs is expected from the workspace.
find_package(miracle_msgs QUIET)

# ---------------------------------------------------------------------------
# Install configuration files into the share directory so that launch files
# and the micro-ROS agent can locate them at runtime.
# ---------------------------------------------------------------------------
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install firmware sources (reference copies available in the ROS workspace).
install(
  DIRECTORY firmware/
  DESTINATION share/${PROJECT_NAME}/firmware
)

# ---------------------------------------------------------------------------
# Testing
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
