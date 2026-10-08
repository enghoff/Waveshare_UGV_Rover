cmake_minimum_required(VERSION 3.16)
project(ugv_behaviors CXX)

# Plain CMake, and deliberately not colcon.
#
# ros_nav/install.sh says there is no workspace on this rover and explains why:
# every node here is a script the launch files name by absolute path, so there
# is nothing to build and therefore nothing to forget to rebuild. A C++ plugin
# has no such option -- pluginlib loads a shared object or it does not -- so this
# is the one build step in the ROS stack, and build.sh beside it is what runs it.
# The manifest runs build.sh on every deploy of ros_nav for exactly the reason
# lidar_slam/ does: a stale .so is a rover running last week's code with this
# week's source sitting next to it.
#
# What this does not do is install into the conda environment. The environment
# is an installed dependency that install.sh builds; writing into it would make
# it something a deploy edits, and the next `mamba install` would be entitled to
# take this back out. So the package installs into ./install here, and
# nav.launch.py puts that prefix on the behaviour server's environment and
# nobody else's.

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# The escape behaviours, and the live obstacle layer the planner's costmap loads
# (include/ugv_behaviors/live_obstacle_layer.hpp), in one library.
add_library(ugv_behaviors SHARED src/escape_behaviors.cpp src/live_obstacle_layer.cpp)
target_include_directories(ugv_behaviors PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(ugv_behaviors
  nav2_behaviors nav2_core nav2_costmap_2d nav2_msgs nav2_util
  geometry_msgs pluginlib rclcpp rclcpp_lifecycle sensor_msgs tf2 tf2_ros)

# `Spin` is not header-only the way `DriveOnHeading` is: its constructor, onRun
# and onConfigure live in libnav2_spin_behavior.so, so the subclass has to link
# against it or fail at load time with an undefined symbol rather than at build
# time with a sensible message.
find_library(NAV2_SPIN_BEHAVIOR_LIB nav2_spin_behavior
  HINTS ${nav2_behaviors_DIR}/../../../lib $ENV{CONDA_PREFIX}/lib REQUIRED)
target_link_libraries(ugv_behaviors ${NAV2_SPIN_BEHAVIOR_LIB})
# RoboStack exports tracing dependencies as bare library names. The system
# compiler needs the conda library directory to resolve them on a clean build.
get_filename_component(NAV2_LIBRARY_DIR "${NAV2_SPIN_BEHAVIOR_LIB}" DIRECTORY)
target_link_directories(ugv_behaviors PUBLIC "${NAV2_LIBRARY_DIR}")

# Goal setup only: no action server or velocity publisher, so these checks never
# command the rover. Run against the installed Nav2 semantics, not a sign model.
enable_testing()
add_executable(test_direction src/test_direction.cpp)
ament_target_dependencies(test_direction
  nav2_behaviors nav2_core nav2_costmap_2d nav2_msgs nav2_util
  geometry_msgs pluginlib rclcpp rclcpp_lifecycle tf2 tf2_ros)
find_library(NAV2_BACK_UP_BEHAVIOR_LIB nav2_back_up_behavior
  HINTS ${nav2_behaviors_DIR}/../../../lib $ENV{CONDA_PREFIX}/lib REQUIRED)
target_link_libraries(test_direction ugv_behaviors ${NAV2_BACK_UP_BEHAVIOR_LIB})
add_test(NAME behavior_direction COMMAND test_direction)

# The live layer's marking, on a costmap with no node behind it.
add_executable(test_live_layer src/test_live_layer.cpp)
ament_target_dependencies(test_live_layer nav2_costmap_2d rclcpp sensor_msgs)
target_link_libraries(test_live_layer ugv_behaviors)
add_test(NAME live_layer COMMAND test_live_layer)

pluginlib_export_plugin_description_file(nav2_core ugv_behaviors.xml)
pluginlib_export_plugin_description_file(nav2_costmap_2d ugv_costmap_layers.xml)

install(TARGETS ugv_behaviors
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(ugv_behaviors)
ament_package()
