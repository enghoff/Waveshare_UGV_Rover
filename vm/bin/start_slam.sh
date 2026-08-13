#!/usr/bin/env bash
# Start mapping, and optionally RViz, then return immediately.
#
#   start_slam.sh                mapping + camera, EKF odometry, headless
#   start_slam.sh rviz           ... + RViz on :0
#   start_slam.sh rviz crop      ... with the rover's rear sector masked
#   start_slam.sh nocam          lidar only, for a long bag recording
#   start_slam.sh rf2o           unfused rf2o odometry, for comparison
#   start_slam.sh static         no odometry; slam_toolbox matches every scan
#
# Odometry defaults to the EKF: rf2o for translation, the OAK's gyro for
# rotation. Keep the rover still for the first 15 s of any run -- that window is
# where the gyro bias is measured, and moving through it poisons the correction
# for the whole session.
set -eo pipefail

source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

bash "$HOME/ugv/bin/stop.sh" > /dev/null 2>&1 || true
for p in rviz2 slam_toolbox rf2o ekf_node fusion_prep lidar_watchdog; do
    pkill -9 -f "$p" 2>/dev/null || true
done
sleep 2

# Check every singleton, not just the camera. The camera announces a second copy
# of itself with X_LINK_DEVICE_ALREADY_IN_USE; nothing else does. A duplicate
# ekf_node just publishes to /odometry/filtered alongside the first, and the
# resulting interleaved estimate looks like an unstable filter rather than two
# stable ones.
for exe in component_container ekf_node fusion_prep ldlidar_stl_ros2_node; do
    if pgrep -f "(^|/)$exe" > /dev/null; then
        echo "REFUSING: $exe survived cleanup -- $(pgrep -c -f "(^|/)$exe") still running"
        exit 1
    fi
done

CROP=false
ODOM=ekf
CAM=true
for a in "$@"; do
    [ "$a" = "crop" ] && CROP=true
    [ "$a" = "static" ] && ODOM=static
    [ "$a" = "rf2o" ] && ODOM=rf2o
    [ "$a" = "nocam" ] && CAM=false
done

# The IMU comes from the OAK, so no camera means no gyro and the EKF is left
# fusing nothing but the source it exists to correct. Fall back loudly, rather
# than leave a filter running that silently reproduces plain rf2o.
if [ "$ODOM" = "ekf" ] && [ "$CAM" = "false" ]; then
    echo "note: nocam leaves the EKF without a gyro -- falling back to odom_source=rf2o"
    ODOM=rf2o
fi

setsid nohup ros2 launch "$HOME/ugv/launch/slam.launch.py" \
    "crop:=$CROP" "odom_source:=$ODOM" "camera:=$CAM" \
    > /tmp/slam.log 2>&1 < /dev/null &
echo "launched slam (crop=$CROP odom=$ODOM camera=$CAM)"

for a in "$@"; do
    if [ "$a" = "rviz" ]; then
        for _ in $(seq 1 50); do
            if ros2 topic list 2>/dev/null | grep -qx /scan; then break; fi
            sleep 1
        done
        sleep 3
        # The SVGA output comes up at 800x600 after a reboot, and RViz records
        # its dock layout in Qt settings rather than in slam.rviz. At 800x600
        # there is no room to dock the image panel, RViz disables the display to
        # cope, and that state then outlives the resize -- the camera view stays
        # missing at any resolution until these are cleared. Toggling the display
        # back on by hand is not a workaround: creating the image render panel
        # against this GL driver after startup takes RViz down with it.
        DISPLAY=:0 xrandr --output Virtual1 --mode 1280x800 2>/dev/null || true
        rm -f "$HOME/.config/ros.org/persistent_settings"
        DISPLAY=:0 setsid nohup rviz2 -d "$HOME/ugv/config/slam.rviz" \
            > /tmp/rviz.log 2>&1 < /dev/null &
        echo "launched rviz"
    fi
done
exit 0
