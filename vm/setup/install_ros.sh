#!/usr/bin/env bash
# ROS 2 Humble plus the three packages this rig actually needs.
#
# Not ugv_ws: with no Pi and no motors, ugv_base_node, ugv_bringup, ugv_gazebo
# and ugv_web_app have nothing to drive. What is needed is a camera driver, a
# lidar driver and a SLAM backend.
set -euo pipefail

export DEBIAN_FRONTEND=noninteractive
export NEEDRESTART_MODE=a

echo "=== apt sources ==="
sudo apt-get update -qq
sudo apt-get install -y -qq software-properties-common curl gnupg lsb-release
sudo add-apt-repository -y universe > /dev/null

sudo curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
ARCH="$(dpkg --print-architecture)"
CODENAME="$(. /etc/os-release && echo "$UBUNTU_CODENAME")"
echo "deb [arch=$ARCH signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $CODENAME main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt-get update -qq

echo "=== ros-humble-desktop (includes RViz2) ==="
sudo apt-get install -y -qq ros-humble-desktop ros-dev-tools

echo "=== stack packages ==="
# imu-filter-madgwick matters: the OAK-D-Lite's BMI270 gives raw accel+gyro with
# no on-chip fusion, so gravity has to be estimated in software before RTAB-Map
# can use it.
WANTED=(
    ros-humble-depthai-ros
    ros-humble-depthai-ros-driver
    ros-humble-rtabmap-ros
    ros-humble-imu-filter-madgwick
    ros-humble-rmw-cyclonedds-cpp
    ros-humble-robot-state-publisher
    ros-humble-tf2-tools
    ros-humble-rosbag2-storage-mcap
)
AVAILABLE=()
for pkg in "${WANTED[@]}"; do
    if apt-cache show "$pkg" > /dev/null 2>&1; then
        AVAILABLE+=("$pkg")
    else
        echo "NOT IN ARCHIVE: $pkg"
    fi
done
sudo apt-get install -y -qq "${AVAILABLE[@]}"

echo "=== versions ==="
source /opt/ros/humble/setup.bash
echo "ROS_DISTRO=$ROS_DISTRO"
for pkg in depthai_ros_driver rtabmap_ros imu_filter_madgwick rviz2 rosbag2_storage_mcap; do
    if ros2 pkg prefix "$pkg" > /dev/null 2>&1; then
        echo "  ok: $pkg"
    else
        echo "  MISSING: $pkg"
    fi
done

grep -q 'source /opt/ros/humble/setup.bash' "$HOME/.bashrc" \
    || echo 'source /opt/ros/humble/setup.bash' >> "$HOME/.bashrc"

echo "=== done ==="
