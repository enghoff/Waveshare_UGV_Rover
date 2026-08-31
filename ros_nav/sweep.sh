#!/bin/bash
# Kill every node this stack starts, so the next launch has the hardware to itself.
#
#     ~/ugv/ros_nav/sweep.sh          # and then check nothing is left
#
# This is not tidiness, it is the difference between a working rover and a broken
# one. `ros2 launch` does **not** reliably take its nodes down with it: killed
# with SIGTERM it exits and its children keep running, so every reload used to
# leave another lidar_node behind. Three of them accumulated once, all reading the
# same serial port -- Linux allows that -- and each got a third of the packets.
# The symptom was /scan arriving at 18 Hz from a 10 Hz sensor, with three
# publishers nobody had asked for. Nothing errored anywhere.
#
# **Why this is its own file and not a function in run_ros_nav.sh.** It was a
# function, and that turned out to be a trap with a long fuse. The supervisor runs
# for weeks; bash parses a function once, when the script starts, and keeps that
# copy. So adding a node to the sweep and deploying it changed nothing at all
# until the supervisor itself was replaced -- and the way that failed was as bad
# as the fault it was meant to prevent. A newly added nav_bridge was not swept, it
# survived the reload, the *new* one could not bind its port and died, and the
# stack came back running the previous deploy's code while `restart.sh` counted
# one of each and said everything was fine.
#
# A separate script is read from disk every time it is called, so what runs is
# what was deployed. Anything that adds a node to this stack adds it here.
#
# The paths are absolute so that the patterns name the deployed files and cannot
# match this script's own command line -- the mistake that has cost this
# repository three ssh sessions.

DIR="$(cd "$(dirname "$0")" && pwd)"

# SIGTERM first so a process that can still run its `finally:` can drop the
# serial port and the 8773 listener.
pkill -f "$DIR/lidar_node.py" 2>/dev/null
pkill -f "$DIR/base_node.py" 2>/dev/null
pkill -f "$DIR/nav_bridge.py" 2>/dev/null
pkill -f 'async_slam_toolbox_node' 2>/dev/null
# RTAB-Map, by the path it is started from rather than by the bare name
# `rtabmap`: that word appears in this stack's own config path, in the
# wrapper's arguments and in anything an operator has typed, and a bare
# pattern would take an ssh session down with it -- the trap the header of
# restart.sh describes.
pkill -f "$DIR/run_rtabmap[.]sh" 2>/dev/null
pkill -f '/lib/rtabmap_slam/rtabmap' 2>/dev/null
pkill -f 'nav2_lifecycle_manager/lifecycle_manager' 2>/dev/null
pkill -f 'nav2_controller/controller_server' 2>/dev/null
pkill -f 'nav2_planner/planner_server' 2>/dev/null
pkill -f 'nav2_bt_navigator/bt_navigator' 2>/dev/null
pkill -f 'nav2_behaviors/behavior_server' 2>/dev/null
pkill -f 'nav2_smoother/smoother_server' 2>/dev/null
pkill -f 'nav2_waypoint_follower/waypoint_follower' 2>/dev/null
pkill -f 'nav2_velocity_smoother/velocity_smoother' 2>/dev/null

# The lidar and the wheels need a moment to actually let go of their handles, and
# a listening socket needs a moment to leave TIME_WAIT. Without the wait the next
# launch's nav_bridge hits "Address already in use" against a process that is
# already gone.
sleep 2

# SIGTERM is not enough when CycloneDDS is wedged: the Python nodes sit inside
# `rclpy.spin` and ignore the signal for longer than the two seconds above.
# The next launch then starts a second copy, the new nav_bridge cannot bind
# 8773, it exits 1, and `restart.sh` still counts one of each -- the leftover.
# Kill what is left, then wait once more for the handles.
pkill -9 -f "$DIR/lidar_node.py" 2>/dev/null
pkill -9 -f "$DIR/base_node.py" 2>/dev/null
pkill -9 -f "$DIR/nav_bridge.py" 2>/dev/null
pkill -9 -f 'async_slam_toolbox_node' 2>/dev/null
pkill -9 -f "$DIR/run_rtabmap[.]sh" 2>/dev/null
pkill -9 -f '/lib/rtabmap_slam/rtabmap' 2>/dev/null
pkill -9 -f 'nav2_lifecycle_manager/lifecycle_manager' 2>/dev/null
pkill -9 -f 'nav2_controller/controller_server' 2>/dev/null
pkill -9 -f 'nav2_planner/planner_server' 2>/dev/null
pkill -9 -f 'nav2_bt_navigator/bt_navigator' 2>/dev/null
pkill -9 -f 'nav2_behaviors/behavior_server' 2>/dev/null
pkill -9 -f 'nav2_smoother/smoother_server' 2>/dev/null
pkill -9 -f 'nav2_waypoint_follower/waypoint_follower' 2>/dev/null
pkill -9 -f 'nav2_velocity_smoother/velocity_smoother' 2>/dev/null
sleep 1

# What is left, if anything, as the exit status. A caller can then say so rather
# than discovering it later as a rover running last week's code.
left=0
for pattern in "$DIR/lidar_node.py" "$DIR/base_node.py" "$DIR/nav_bridge.py" \
               async_slam_toolbox_node '/lib/rtabmap_slam/rtabmap'; do
    # pgrep -c prints 0 and exits 1 when nothing matches. `|| echo 0` then
    # appends a second 0 and `[[ "$n" -gt 0 ]]` becomes "integer expression
    # expected" -- which is how a clean sweep used to look like a failure.
    n=$(pgrep -fc "$pattern" 2>/dev/null || true)
    n=${n:-0}
    if [ "$n" -gt 0 ]; then
        echo "!! $n x $pattern survived the sweep" >&2
        left=$((left + n))
    fi
done
exit $((left > 0 ? 1 : 0))
