cmake_minimum_required(VERSION 3.10)
project(stepper_hardware_interface)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(motion_sdk REQUIRED)

add_library(stepper_hardware SHARED
  src/stepper_hardware.cpp
)

target_include_directories(stepper_hardware PUBLIC
  include
)

ament_target_dependencies(stepper_hardware
  rclcpp
  hardware_interface
  pluginlib
  motion_sdk
)

pluginlib_export_plugin_description_file(
  hardware_interface plugin.xml
)

install(TARGETS stepper_hardware LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

ament_package()
