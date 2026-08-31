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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(ugv_behaviors SHARED src/escape_behaviors.cpp)
target_include_directories(ugv_behaviors PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(ugv_behaviors
  nav2_behaviors nav2_core nav2_costmap_2d nav2_msgs nav2_util
  geometry_msgs pluginlib rclcpp rclcpp_lifecycle tf2 tf2_ros)

# `Spin` is not header-only the way `DriveOnHeading` is: its constructor, onRun
# and onConfigure live in libnav2_spin_behavior.so, so the subclass has to link
# against it or fail at load time with an undefined symbol rather than at build
# time with a sensible message.
find_library(NAV2_SPIN_BEHAVIOR_LIB nav2_spin_behavior
  HINTS ${nav2_behaviors_DIR}/../../../lib $ENV{CONDA_PREFIX}/lib REQUIRED)
target_link_libraries(ugv_behaviors ${NAV2_SPIN_BEHAVIOR_LIB})

pluginlib_export_plugin_description_file(nav2_core ugv_behaviors.xml)

install(TARGETS ugv_behaviors
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(ugv_behaviors)
ament_package()
