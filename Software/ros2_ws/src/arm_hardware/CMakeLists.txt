# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (c) 2026 Emanuel Käfer, Matteo Gassler, and Alexander Dick and contributors
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# This program is distributed WITHOUT ANY EXPRESS OR IMPLIED WARRANTY.
# See LICENSE-SOFTWARE in the project root for details.

cmake_minimum_required(VERSION 3.16)
project(arm_hardware LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# hardware_interface (Jazzy) requires C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# ---------------------------------------------------------------------------
# Plugin library
#   serial_port.cpp   POSIX serial port (no ROS)
#   stm_protocol.cpp  frame encode/decode, CRC, parser (no ROS)
#   arm_system.cpp    the ros2_control SystemInterface (uses the two above)
# ---------------------------------------------------------------------------
add_library(arm_hardware SHARED
  src/serial_port.cpp
  src/stm_protocol.cpp
  src/arm_system.cpp
)
target_include_directories(arm_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(arm_hardware PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

# Makes "arm_hardware/ArmSystem" known to the controller_manager (see arm_hardware.xml)
pluginlib_export_plugin_description_file(hardware_interface arm_hardware.xml)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS arm_hardware
  EXPORT export_arm_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ---------------------------------------------------------------------------
# Tests (colcon test --packages-select arm_hardware)
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_stm_protocol test/test_stm_protocol.cpp)
  target_link_libraries(test_stm_protocol arm_hardware)
endif()

ament_export_targets(export_arm_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()
