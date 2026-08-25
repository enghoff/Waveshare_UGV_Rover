#!/bin/bash
# Reload the ROS 2 stack after deploying new nodes or a new config.
#
#     ssh bpi-m4zero '~/ugv/ros_nav/restart.sh'
#     ssh bpi-m4zero '~/ugv/ros_nav/restart.sh --supervisor'   # after changing
#                                                             # run_ros_nav.sh
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

SUPERVISOR=no
if [ "${1:-}" = "--supervisor" ]; then
    SUPERVISOR=yes
fi

if [ "$SUPERVISOR" = yes ]; then
    # Replace the supervisor itself, not just the launch under it. Needed when
    # run_ros_nav.sh has changed, because bash holds a parsed copy of a running
    # script -- and after that, the crontab is where the arguments live, so they
    # are read from there rather than guessed.
    echo "--- replacing the supervisor"
    pkill -f 'ros_nav/run_ros_nav[.]sh'
    sleep 2
    bash "$DIR/sweep.sh"
fi

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
    setsid nohup bash "$DIR/run_ros_nav.sh" $args > /dev/null 2>&1 < /dev/null &
fi

# Long, because this is not a Python process starting. The launch brings up four
# or nine nodes, slam_toolbox allocates its 40 MB stack and its correlation
# grids, and the lifecycle transitions happen after all of that.
sleep 30
tail -6 "$DIR/ros_nav.log"

# One of each, and it is worth checking rather than assuming. `ros2 launch` does
# not reliably take its nodes down when it is killed, so a reload can leave the
# old lidar_node running beside the new one -- both reading the same serial port,
# each getting half the packets, nothing anywhere reporting an error. sweep.sh
# runs before every launch to prevent it; this is how you find out that it did.
echo "--- one of each?"
for name in lidar_node.py base_node.py nav_bridge.py async_slam_toolbox_node; do
    n=$(pgrep -fc "$name" 2>/dev/null || true)
    n=${n:-0}
    if [ "$n" -eq 1 ]; then
        printf '  ok   %-26s 1\n' "$name"
    else
        printf '  !!   %-26s %s  <- expected 1\n' "$name" "$n"
    fi
done

# Counting processes is not enough on its own, and this is the check that would
# have caught the worst reload this stack has had. A node that was not swept
# survives, the replacement for it dies immediately -- on a serial port it cannot
# open, or a socket it cannot bind -- and the count is still exactly one. What
# gives it away is a death in the log *after* the launch started, so that is what
# is looked for. The stale node then answers every question with last week's
# code, which is a far worse place to be than a stack that is plainly down.
echo "--- anything die on the way up?"
died=$(tail -200 "$DIR/ros_nav.log" | grep -c 'process has died')
if [ "$died" -eq 0 ]; then
    echo "  ok   nothing died"
else
    echo "  !!   $died process death(s) in the last 200 log lines:"
    tail -200 "$DIR/ros_nav.log" | grep 'process has died' | tail -4 | sed 's/^/       /'
    echo "       A node that survived the sweep is the usual cause: its"
    echo "       replacement cannot have the port and exits, and the count above"
    echo "       still says one. Try: ~/ugv/ros_nav/restart.sh --supervisor"
fi

echo "--- nodes:"
# shellcheck disable=SC1091
. "$DIR/env.sh"
# shellcheck disable=SC1091
. "$DIR/dds.sh"
# `ros2 node list` talks to DDS. When CycloneDDS is wedged it never returns, the
# SSH client dies at 90 s, and a `restart.sh` is left running on the rover.
# Capture first, then sort: a pipe would hide timeout's exit status.
if nodes=$(timeout 15 ros2 node list 2>/dev/null); then
    printf '%s\n' "$nodes" | sort -u
else
    echo "  !! ros2 node list did not finish in 15s -- DDS is wedged;" \
         "process counts above are the check that still works"
fi
timeout 10 ros2 lifecycle get /slam_toolbox 2>/dev/null || true

# And that the daemon can actually reach the bridge, which is the whole point of
# the stack being up. Checked from here because it is one line and because the
# alternative is finding out from a console that shows a rover it cannot move.
echo "--- the daemon's way in:"
if (exec 3<>/dev/tcp/127.0.0.1/8773) 2>/dev/null; then
    exec 3<&- 2>/dev/null
    echo "  ok   something is listening on 8773"
else
    echo "  !!   nothing is listening on 8773, so the daemon has no driving tools"
fi

# The check that would have named the silent-graph deaths: processes listed,
# lidar still 9.9 Hz, CycloneDDS writing to a radio that is no longer there.
# Sourced from dds.sh into this launch; if it is missing, the next failover
# looks like Nav2 crashing.
echo "--- discovery:"
bridge=$(pgrep -n -f "$DIR/nav_bridge.py" || true)
if [ -n "$bridge" ] && tr '\0' '\n' < "/proc/$bridge/environ" \
        | grep -q '^ROS_LOCALHOST_ONLY=1$'; then
    echo "  ok   nav_bridge is localhost-only, so a dead radio cannot take the graph"
else
    echo "  !!   nav_bridge is discovering on the LAN;" \
         "dds.sh was not sourced into this launch"
fi
