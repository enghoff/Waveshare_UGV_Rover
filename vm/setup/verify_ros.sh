#!/usr/bin/env bash
# Verify the ROS 2 install and find out what exists for the D500 lidar.
#
# Deliberately no 'set -u': /opt/ros/humble/setup.bash reads
# AMENT_TRACE_SETUP_FILES without a default, so nounset makes sourcing ROS fatal.
set -eo pipefail

source /opt/ros/humble/setup.bash
echo "ROS_DISTRO=$ROS_DISTRO"

echo "=== packages ==="
for pkg in depthai_ros_driver depthai_bridge rtabmap_ros rtabmap_slam rtabmap_odom \
           imu_filter_madgwick rviz2 rosbag2_storage_mcap slam_toolbox; do
    if ros2 pkg prefix "$pkg" > /dev/null 2>&1; then
        echo "  ok:      $pkg"
    else
        echo "  MISSING: $pkg"
    fi
done

echo "=== packaged lidar drivers in the ROS archive ==="
apt-cache search ros-humble 2>/dev/null \
    | grep -i -E 'ldlidar|ld19|ldrobot|sllidar|rplidar' \
    || echo "  (nothing matching LD19/LDROBOT)"

echo "=== depthai_ros_driver launch files ==="
ls "$(ros2 pkg prefix depthai_ros_driver)/share/depthai_ros_driver/launch/" 2>/dev/null | head -20

echo "=== persist ROS in the shell ==="
grep -q 'source /opt/ros/humble/setup.bash' "$HOME/.bashrc" \
    || echo 'source /opt/ros/humble/setup.bash' >> "$HOME/.bashrc"
echo "done"
