cmake_minimum_required(VERSION 3.5)
project(arm_hardware)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(control_msgs REQUIRED)

# Create library
add_library(${PROJECT_NAME} SHARED
  src/arm_hardware.cpp
)

# Include headers
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  control_msgs
)

# Export as plugin
pluginlib_export_plugin_description_file(hardware_interface robot_system.xml)

# Install targets and headers
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
