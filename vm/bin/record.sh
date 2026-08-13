#!/usr/bin/env bash
# Record a mapping run to an MCAP bag, attaching to whatever is already running.
#
#   record.sh              record until Ctrl-C
#   record.sh 300          record 300 seconds, then stop
#   record.sh 300 hallway  ... into ~/bags/hallway
#
# Deliberately does NOT launch anything: start the stack first with
# bin/start_slam.sh, confirm it with checks/slam.sh, then start recording. A
# recorder that brings up its own camera is how you end up with two copies of the
# driver fighting over one USB device.
#
# MCAP, and mono/depth rather than RGB: measured at roughly 0.83 GB per minute,
# so a ten minute push is about 8 GB. The free-space guard below is not
# ceremony -- filling the guest's disk mid-run corrupts the bag being written.
set -eo pipefail

DURATION="${1:-0}"
NAME="${2:-run_$(date +%Y%m%d_%H%M%S)}"
BAG="$HOME/bags/$NAME"

source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

if ! pgrep -f ldlidar_stl_ros2_node > /dev/null; then
    echo "REFUSING: nothing is publishing. Start bin/start_slam.sh first."
    exit 1
fi

FREE_MB=$(df -Pm "$HOME" | awk 'NR==2 {print $4}')
if [ "$FREE_MB" -lt 10000 ]; then
    echo "REFUSING: only ${FREE_MB} MB free, and this records ~850 MB per minute."
    exit 1
fi
echo "free: ${FREE_MB} MB (~$((FREE_MB / 850)) minutes of recording)"

# /tf_static carries the measured sensor geometry, and it is latched -- miss it
# and the bag cannot be reprocessed into any common frame later.
TOPICS=(
    /scan /tf /tf_static /odom_rf2o /map
    /oak/right/image_rect /oak/right/camera_info
    /oak/stereo/image_raw /oak/stereo/camera_info
    /oak/imu/data /imu/data
)

mkdir -p "$HOME/bags"
echo "recording to $BAG"
if [ "$DURATION" -gt 0 ]; then
    timeout -s INT "$DURATION" ros2 bag record -s mcap -o "$BAG" "${TOPICS[@]}" || true
else
    ros2 bag record -s mcap -o "$BAG" "${TOPICS[@]}"
fi

echo "=== bag info ==="
ros2 bag info "$BAG"
