#!/usr/bin/env bash
# LDRobot's reference ROS 2 driver for the STL-19P / LD19.
#
# Nothing in the ROS archive covers this sensor -- only RPLidar variants -- so it
# is a source build. Chosen over wrapping the parser in lidar/lidar_view.py
# because a sensor_msgs/LaserScan carries angle_increment, scan_time and
# time_increment that scan matching depends on, and those are easy to get subtly
# wrong. The existing parser stays the reference for what correct output means.
set -eo pipefail

source /opt/ros/humble/setup.bash

mkdir -p "$HOME/ros2_ws/src"
cd "$HOME/ros2_ws/src"
if [ ! -d ldlidar_stl_ros2 ]; then
    git clone --depth 1 https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git
fi

cd "$HOME/ros2_ws"
colcon build --symlink-install --packages-select ldlidar_stl_ros2

source "$HOME/ros2_ws/install/setup.bash"
echo "=== package ==="
ros2 pkg prefix ldlidar_stl_ros2

echo "=== launch files ==="
ls src/ldlidar_stl_ros2/launch/

echo "=== which launch files mention LD19 / STL19 ==="
grep -ril -E 'ld19|stl19|LD19|STL19' src/ldlidar_stl_ros2/launch/ || echo "(none by name)"

echo "=== port defaults referenced ==="
grep -rn 'ttyUSB\|port_name\|serial_port' src/ldlidar_stl_ros2/launch/ | head -20
