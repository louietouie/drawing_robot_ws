cmake_minimum_required(VERSION 3.16)
project(base_package LANGUAGES CXX)

message(WARNING "configuring project...")

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# set(drake_DIR /home/louis/drake/tools/install/libdrake)
# set(drake_DIR /home/louis/drake/bazel-bin/tools/install/libdrake)
# set(drake_DIR /home/louis/drake/bazel-drake/tools/install/libdrake)
# list(APPEND CMAKE_PREFIX_PATH /home/louis/drake/bazel-bin/tools/install/libdrake)

set(drake_DIR /home/louis/Documents/drake-build/install/lib/cmake/drake/)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  controller_interface
  generate_parameter_library
  realtime_tools
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
find_package(drake REQUIRED)
# find_package(drake CONFIG REQUIRED PATHS /opt/drake)
find_package(drake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# custom to generate_parameter_library to auto generate a header file
generate_parameter_library(
  ik_controller_parameters
  controller/ik_controller_parameters.yaml
)

## COMPILE
add_library(
  base_package
  SHARED
  hardware/rrbot.cpp hardware/can_comms.cpp
  controller/ik_controller.cpp
  controller/drake_calculations.cpp
)
target_compile_features(base_package PUBLIC cxx_std_17)
target_include_directories(base_package PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controllers/include>
  $<INSTALL_INTERFACE:include/base_package>
)
# add_executable(drake_calculations controller/drake_calculations.cpp)
# target_link_libraries(drake_calculations)
# add_executable(
#   drake_calculations controller/drake_calculations.cpp
# )

target_link_libraries(base_package PUBLIC
  ik_controller_parameters                # custom to generate_parameter_library
  drake::drake
)
ament_target_dependencies(
  base_package PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface base_package.xml)
# Export controller plugins
pluginlib_export_plugin_description_file(controller_interface base_package.xml)

# INSTALL

install(
  DIRECTORY hardware/include/
  DESTINATION include/base_package
)
install(
  DIRECTORY controller/include/
  DESTINATION include/base_package
)
install(
  DIRECTORY models worlds
  DESTINATION share/base_package
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/base_package
)
install(
  TARGETS
    base_package
    ik_controller_parameters
  EXPORT export_base_package
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## EXPORTS
ament_export_targets(export_base_package HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()

## Louis notes
## Built drake on Rpi5 Ubuntu 24.04 with...
## bazel build //tools/install/libdrake:libdrake.so
# INFO: Found 1 target...
# Target //tools/install/libdrake:libdrake.so up-to-date:
#   bazel-bin/tools/install/libdrake/libdrake.so
# INFO: Elapsed time: 17295.341s, Critical Path: 156.09s
# INFO: 8175 processes: 4062 internal, 4113 processwrapper-sandbox.
# INFO: Build completed successfully, 8175 total actions


# By not providing "Finddrake.cmake" in CMAKE_MODULE_PATH this project has
# asked CMake to find a package configuration file provided by "drake", but
# CMake did not find one.

# Could not find a package configuration file provided by "drake" with any of
# the following names:

#   drakeConfig.cmake
#   drake-config.cmake

# Add the installation prefix of "drake" to CMAKE_PREFIX_PATH or set
# "drake_DIR" to a directory containing one of the above files.  If "drake"
# provides a separate development package or SDK, be sure it has been
# installed.


message(WARNING "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")
message(WARNING "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")
message(WARNING "CMAKE_FIND_PACKAGE_REDIRECTS_DIR: ${CMAKE_FIND_PACKAGE_REDIRECTS_DIR}")