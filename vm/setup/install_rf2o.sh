#!/usr/bin/env bash
# rf2o: odometry by matching consecutive laser scans.
#
# Needed because slam_toolbox wants an odom frame and this rover has no wheel
# encoders. The BMI270 cannot fill the gap -- it gives orientation and turn rate,
# not position. Same choice Waveshare made on their own stack, for the same reason.
set -eo pipefail

source /opt/ros/humble/setup.bash
mkdir -p "$HOME/ros2_ws/src"
cd "$HOME/ros2_ws/src"

if [ ! -d rf2o_laser_odometry ]; then
    git clone --depth 1 -b ros2 https://github.com/MAPIRlab/rf2o_laser_odometry.git \
        || git clone --depth 1 https://github.com/MAPIRlab/rf2o_laser_odometry.git
fi

cd "$HOME/ros2_ws"
colcon build --symlink-install --packages-select rf2o_laser_odometry

source "$HOME/ros2_ws/install/setup.bash"
echo "=== built ==="
ros2 pkg prefix rf2o_laser_odometry

echo "=== its launch file, for the real parameter names ==="
find "$HOME/ros2_ws/src/rf2o_laser_odometry" -name '*.launch.py' -exec cat {} \; | head -60
