#!/bin/bash
# Reload the ROS 2 stack after deploying new nodes or a new config.
#
#     ssh bpi-m4zero '~/ugv/ros_nav/restart.sh'
#
# The patterns below are why this is a file and not an ssh command line. `pkill
# -f 'ros2 launch'` typed into ssh matches the shell that is running that very
# command and kills the session, which looks like the rover dropping off the
# network -- the exact fault this repository spent a day instrumenting. The same
# trap caught `lidar_slam/` and `rover_daemon/`, and it catches everybody once.
#
# Belt and braces on the pattern anyway: the brackets mean the text of the
# pattern never matches itself, so even pasted into a shell it is safe.

DIR="$(cd "$(dirname "$0")" && pwd)"

if pgrep -f 'ros_nav/run_ros_nav[.]sh' > /dev/null; then
    # Supervised: kill the launch and let the supervisor bring it back with the
    # arguments it was started with -- which is where `--nav` lives.
    pkill -f 'ros2 launch.*ros_nav'
else
    # Nothing is supervising it. Start it the way boot would, reading the flags
    # from the crontab rather than inventing a second set here -- the mistake
    # rover_daemon/restart.sh documents, where a hand relaunch silently dropped
    # --vision and the rover lost its camera.
    args="$(crontab -l 2>/dev/null | sed -n 's|^@reboot .*run_ros_nav\.sh *||p' | head -1)"
    # shellcheck disable=SC2086 -- word splitting is the point: these are flags.
    setsid nohup "$DIR/run_ros_nav.sh" $args > /dev/null 2>&1 < /dev/null &
fi

# Long, because this is not a Python process starting. The launch brings up three
# or eight nodes, slam_toolbox allocates its 40 MB stack and its correlation
# grids, and the lifecycle transitions happen after all of that.
sleep 30
tail -6 "$DIR/ros_nav.log"

# One of each, and it is worth checking rather than assuming. `ros2 launch` does
# not reliably take its nodes down when it is killed, so a reload can leave the
# old lidar_node running beside the new one -- both reading the same serial port,
# each getting half the packets, nothing anywhere reporting an error. The
# supervisor sweeps before every launch to prevent it; this is how you find out
# that it did.
echo "--- one of each?"
for name in lidar_node.py base_node.py async_slam_toolbox_node; do
    n=$(pgrep -fc "$name" 2>/dev/null || echo 0)
    if [ "$n" -eq 1 ]; then
        printf '  ok   %-26s 1\n' "$name"
    else
        printf '  !!   %-26s %s  <- expected 1\n' "$name" "$n"
    fi
done

echo "--- nodes:"
# shellcheck disable=SC1091
. "$DIR/env.sh"
ros2 node list 2>/dev/null | sort -u
ros2 lifecycle get /slam_toolbox 2>/dev/null
