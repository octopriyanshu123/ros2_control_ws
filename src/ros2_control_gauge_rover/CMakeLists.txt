cmake_minimum_required(VERSION 3.16)
project(ros2_control_gauge_rover LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  ros2_control_gauge_rover
  SHARED
  hardware/rrbot_actuator.cpp
)
target_compile_features(ros2_control_gauge_rover PUBLIC cxx_std_17)
target_include_directories(ros2_control_gauge_rover PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/ros2_control_gauge_rover>
)
ament_target_dependencies(
  ros2_control_gauge_rover PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface ros2_control_gauge_rover.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/ros2_control_gauge_rover
)

install(
  DIRECTORY rrbot/urdf rrbot/rviz
  DESTINATION share/${PROJECT_NAME}/rrbot
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/ros2_control_gauge_rover
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/ros2_control_gauge_rover
)
install(TARGETS ros2_control_gauge_rover
  EXPORT export_ros2_control_gauge_rover
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  
endif()

## EXPORTS
ament_export_targets(export_ros2_control_gauge_rover HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
