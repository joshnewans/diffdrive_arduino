cmake_minimum_required(VERSION 3.16)
project(diffdrive_arduino LANGUAGES CXX)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Specify the required version of ros2_control
find_package(controller_manager 4.0.0)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
  "Are you using the correct branch of the ros2_control_demos repository?")
endif()

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  diffdrive_arduino
  SHARED
  hardware/diffbot_system.cpp
)
target_compile_features(diffdrive_arduino PUBLIC cxx_std_17)
target_include_directories(diffdrive_arduino PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/diffdrive_arduino>
)
target_link_libraries(diffdrive_arduino PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  serial
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface diffdrive_arduino.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/diffdrive_arduino
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf description/rviz
  DESTINATION share/diffdrive_arduino
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/diffdrive_arduino
)
install(TARGETS diffdrive_arduino
  EXPORT export_diffdrive_arduino
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_diffdrive_arduino HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
