# cmake_minimum_required(VERSION 3.16)
# project(example_7 LANGUAGES CXX)

# if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
#   add_compile_options(-Wall -Wextra)
# endif()

# # set the same behavior for windows as it is on linux
# set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# # find dependencies
# set(HW_IF_INCLUDE_DEPENDS
#   pluginlib
#   rcpputils
#   hardware_interface
# )
# set(REF_GEN_INCLUDE_DEPENDS
#   kdl_parser
#   rclcpp
#   trajectory_msgs
# )
# set(CONTROLLER_INCLUDE_DEPENDS
#   pluginlib
#   rcpputils
#   controller_interface
#   realtime_tools
#   trajectory_msgs
# )

# # find dependencies
# find_package(backward_ros REQUIRED)
# find_package(ament_cmake REQUIRED)
# foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
#   find_package(${Dependency} REQUIRED)
# endforeach()
# foreach(Dependency IN ITEMS ${REF_GEN_INCLUDE_DEPENDS})
#   find_package(${Dependency} REQUIRED)
# endforeach()
# foreach(Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
#   find_package(${Dependency} REQUIRED)
# endforeach()

# ## COMPILE
# add_executable(send_trajectory reference_generator/send_trajectory.cpp)

# ament_target_dependencies(
#   send_trajectory PUBLIC
#   ${REF_GEN_INCLUDE_DEPENDS}
# )
# ament_target_dependencies(example_7
#   hardware_interface
#   pluginlib
#   rclcpp
# )
# install(TARGETS example_7
#   LIBRARY DESTINATION lib
# )

# add_library(
#   example_7
#   SHARED
#   hardware/r6bot_hardware.cpp
#   controller/r6bot_controller.cpp
#   hardware/arm_hardware.cpp


# )

# target_compile_features(example_7 PUBLIC cxx_std_17)
# target_include_directories(example_7 PUBLIC
# $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
# $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controller/include>
# $<INSTALL_INTERFACE:include/example_7>
# )
# ament_target_dependencies(
#   example_7 PUBLIC
#   ${HW_IF_INCLUDE_DEPENDS}
#   ${CONTROLLER_INCLUDE_DEPENDS}
# )

# # Export hardware plugins
# pluginlib_export_plugin_description_file(hardware_interface example_7.xml)
# # Export controller plugins
# pluginlib_export_plugin_description_file(controller_interface example_7.xml)

# # INSTALL
# install(
#   DIRECTORY hardware/include/
#   DESTINATION include/example_7
# )
# install(
#   DIRECTORY description/launch description/ros2_control description/urdf
#   DESTINATION share/example_7
# )
# install(
#   DIRECTORY bringup/launch bringup/config
#   DESTINATION share/example_7
# )
# install(
#     TARGETS send_trajectory
#     RUNTIME DESTINATION lib/example_7
# )

# install(TARGETS example_7
#   EXPORT export_example_7
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# if(BUILD_TESTING)
#   find_package(ament_cmake_pytest REQUIRED)

#   ament_add_pytest_test(example_7_urdf_xacro test/test_urdf_xacro.py)
#   ament_add_pytest_test(view_example_7_launch test/test_view_robot_launch.py)
#   ament_add_pytest_test(run_example_7_launch test/test_r6bot_controller_launch.py)
# endif()

# ## EXPORTS
# ament_export_targets(export_example_7 HAS_LIBRARY_TARGET)
# ament_export_dependencies(${HW_IF_INCLUDE_DEPENDS} ${REF_GEN_INCLUDE_DEPENDS} ${CONTROLLER_INCLUDE_DEPENDS})
# ament_package()



cmake_minimum_required(VERSION 3.16)
project(example_7 LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# Dependencies
set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  hardware_interface
)
set(REF_GEN_INCLUDE_DEPENDS
  kdl_parser
  rclcpp
  trajectory_msgs
)
set(CONTROLLER_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  controller_interface
  realtime_tools
  trajectory_msgs
)

# Find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

foreach(Dependency IN ITEMS ${REF_GEN_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

foreach(Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Create executable for trajectory sender
add_executable(send_trajectory reference_generator/send_trajectory.cpp)

ament_target_dependencies(
  send_trajectory
  ${REF_GEN_INCLUDE_DEPENDS}
)

# Create shared library for the hardware interface and controller
add_library(example_7 SHARED
  controller/r6bot_controller.cpp
  hardware/arm_hardware.cpp
)

# Specify C++ version and include directories
target_compile_features(example_7 PUBLIC cxx_std_17)
target_include_directories(example_7 PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controller/include>
  $<INSTALL_INTERFACE:include/example_7>
)

ament_target_dependencies(
  example_7
  ${HW_IF_INCLUDE_DEPENDS}
  ${CONTROLLER_INCLUDE_DEPENDS}
)

# Export hardware and controller plugins
pluginlib_export_plugin_description_file(hardware_interface example_7.xml)
pluginlib_export_plugin_description_file(controller_interface example_7.xml)

# Install headers, launch, and other files
install(
  DIRECTORY hardware/include/
  DESTINATION include/example_7
)

install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/example_7
)

install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/example_7
)

# Install executables and libraries
install(
  TARGETS send_trajectory example_7
  EXPORT export_example_7
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(example_7_urdf_xacro test/test_urdf_xacro.py)
  ament_add_pytest_test(view_example_7_launch test/test_view_robot_launch.py)
  ament_add_pytest_test(run_example_7_launch test/test_r6bot_controller_launch.py)
endif()

# Export targets and dependencies
ament_export_targets(export_example_7 HAS_LIBRARY_TARGET)
ament_export_dependencies(${HW_IF_INCLUDE_DEPENDS} ${REF_GEN_INCLUDE_DEPENDS} ${CONTROLLER_INCLUDE_DEPENDS})

ament_package()
