#!/bin/bash
# Keep the ROS 2 mapping and navigation stack running, and bring it back after a
# reboot. Installed as a `@reboot` crontab entry by install-crontab.sh.
#
#     ~/ugv/ros_nav/run_ros_nav.sh              # mapping only
#     ~/ugv/ros_nav/run_ros_nav.sh --nav        # mapping and Nav2
#
# Bash and not /bin/sh. This sources env.sh, and env.sh activates a conda
# environment whose hooks are bash scripts that call `source` -- which dash does
# not have. Under /bin/sh this dies with "source: not found", a message that
# names neither this file nor the shell.
#
# The retry loop is here for the same reason the daemon's is: the lidar
# enumerates about ninety seconds after the kernel starts on this board, and the
# board bridge only exists once the rover daemon has come up and opened the UART.
# Both are races this can lose at boot and win a few seconds later.

DIR="$(cd "$(dirname "$0")" && pwd)"
LOG="$DIR/ros_nav.log"
RETRY=15

LAUNCH="slam.launch.py"
if [ "${1:-}" = "--nav" ]; then
    LAUNCH="nav.launch.py"
    shift
fi

# Wait for the daemon's board bridge. Without it the base node has no odometry,
# and slam_toolbox handed a rover with no odom -> base_link transform does not
# fail -- it waits, silently, for ever.
i=0
while ! (exec 3<>/dev/tcp/127.0.0.1/8772) 2>/dev/null && [ $i -lt 40 ]; do
    sleep 3
    i=$((i + 1))
done
exec 3<&- 2>/dev/null

# Kill anything this script has ever started that is still about.
#
# This is not tidiness, it is the difference between a working rover and a broken
# one. `ros2 launch` does **not** reliably take its nodes down with it: killed
# with SIGTERM it exits and its children keep running, so every reload left
# another lidar_node behind. Three of them accumulated once, all reading the same
# serial port -- Linux allows that -- and each got a third of the packets. The
# symptom was /scan arriving at 18 Hz from a 10 Hz sensor, with three publishers
# nobody had asked for. Nothing errored anywhere.
#
# The patterns carry $DIR, so they name the deployed paths and cannot match this
# script's own command line.
cleanup() {
    pkill -f "$DIR/lidar_node.py" 2>/dev/null
    pkill -f "$DIR/base_node.py" 2>/dev/null
    pkill -f 'async_slam_toolbox_node' 2>/dev/null
    pkill -f 'nav2_lifecycle_manager/lifecycle_manager' 2>/dev/null
    pkill -f 'nav2_controller/controller_server' 2>/dev/null
    pkill -f 'nav2_planner/planner_server' 2>/dev/null
    pkill -f 'nav2_bt_navigator/bt_navigator' 2>/dev/null
    pkill -f 'nav2_behaviors/behavior_server' 2>/dev/null
    pkill -f 'nav2_smoother/smoother_server' 2>/dev/null
    pkill -f 'nav2_waypoint_follower/waypoint_follower' 2>/dev/null
    pkill -f 'nav2_velocity_smoother/velocity_smoother' 2>/dev/null
    # The lidar and the wheels need a moment to actually let go of their handles.
    sleep 2
}

# Same rule as run_daemon.sh: stop when *this* is signalled, never because a
# child was. `restart.sh` reloads by killing the launch, and a supervisor that
# quit on that would be no supervisor at all. What is different here is that
# stopping must also sweep -- see cleanup().
stop() {
    echo "--- run_ros_nav.sh signalled at $(date -Is), stopping ---" >> "$LOG"
    kill "$child" 2>/dev/null
    cleanup
    exit 0
}
trap stop INT TERM

# shellcheck disable=SC1091
. "$DIR/env.sh"

echo "--- run_ros_nav.sh starting $LAUNCH at $(date -Is) ---" >> "$LOG"
while true; do
    # Before every launch, not just the first. This is what makes a reload
    # idempotent: whatever the previous launch left holding the lidar is gone
    # before the new one goes looking for it.
    cleanup
    ros2 launch "$DIR/$LAUNCH" "$@" >> "$LOG" 2>&1 &
    child=$!
    wait "$child"
    status=$?
    echo "--- launch exited $status at $(date -Is), restarting in ${RETRY}s ---" >> "$LOG"
    sleep $RETRY
done
