#!/usr/bin/env bash
# Confirm the measured geometry reached TF, and that the stack is a single instance.
set -eo pipefail
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

echo "=== instance count (must be 1 each) ==="
echo "component_container: $(pgrep -c -f component_container || echo 0)"
echo "ldlidar:             $(pgrep -c -f ldlidar_stl_ros2_node || echo 0)"
echo "rviz2:               $(pgrep -c -f rviz2 || echo 0)"

echo "=== load ==="
uptime

echo "=== base_link -> lidar_link (expect 0.040, 0.000, 0.157 @ yaw 90) ==="
timeout 12 ros2 run tf2_ros tf2_echo base_link lidar_link 2>&1 | grep -A4 -m1 Translation || echo "(none)"

echo "=== base_link -> oak_right_camera_optical_frame (expect x 0.085, z 0.116) ==="
timeout 12 ros2 run tf2_ros tf2_echo base_link oak_right_camera_optical_frame 2>&1 | grep -A4 -m1 Translation || echo "(none)"

echo "=== topics alive ==="
timeout 10 ros2 topic list 2>/dev/null | grep -E 'scan|stereo/image_raw|imu/data' || echo "(none)"
