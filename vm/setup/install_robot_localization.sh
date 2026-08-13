#!/usr/bin/env bash
# robot_localization: the EKF that fuses rf2o's translation with the BMI270's
# yaw rate, and then owns the odom -> base_link transform that slam_toolbox
# consumes. Binary package, unlike rf2o and the LD19 driver -- it is in the
# Humble archive.
set -eo pipefail

sudo apt-get update -qq
sudo apt-get install -y ros-humble-robot-localization

source /opt/ros/humble/setup.bash
ros2 pkg prefix robot_localization
ls /opt/ros/humble/lib/robot_localization/
