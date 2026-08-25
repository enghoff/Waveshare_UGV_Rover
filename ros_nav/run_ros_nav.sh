#!/bin/bash
# Keep the ROS 2 mapping and navigation stack running, and bring it back after a
# reboot. Installed as a `@reboot` crontab entry by install-boot.sh.
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

# Taking the previous launch's nodes down is `sweep.sh`, and it lives in its own
# file rather than in a function here. That is not a style choice, it is a bug
# that was paid for.
#
# bash parses a function once, when the script starts. This supervisor runs for
# weeks. So a sweep written as a function is whichever copy was on disk the last
# time the supervisor started, and adding a node to it does nothing at all until
# the supervisor itself is replaced -- while every visible sign says the deploy
# worked. That happened with `nav_bridge`: it was not swept, the old one survived
# the reload, the new one could not bind port 8773 and died in the log, and the
# stack came back running the previous deploy's code while `restart.sh` counted
# one of each and reported everything fine.
#
# A separate script is read from disk every time it is called. Anything that adds
# a node to this stack adds it to sweep.sh.

# Same rule as run_daemon.sh: stop when *this* is signalled, never because a
# child was. `restart.sh` reloads by killing the launch, and a supervisor that
# quit on that would be no supervisor at all. What is different here is that
# stopping must also sweep, because the launch will not.
stop() {
    echo "--- run_ros_nav.sh signalled at $(date -Is), stopping ---" >> "$LOG"
    kill "$child" 2>/dev/null
    bash "$DIR/sweep.sh" >> "$LOG" 2>&1
    exit 0
}
trap stop INT TERM

# shellcheck disable=SC1091
. "$DIR/env.sh"

echo "--- run_ros_nav.sh starting $LAUNCH at $(date -Is) ---" >> "$LOG"
while true; do
    # After env.sh, every launch: RoboStack's activate hook sets discovery to
    # the subnet, and a dead radio's leftover address then takes the graph down
    # while every process is still listed. dds.sh is a file (like sweep.sh) so
    # a deploy of it is picked up on the next child restart, not only when this
    # supervisor itself is replaced.
    # shellcheck disable=SC1091
    . "$DIR/dds.sh"
    # Before every launch, not just the first. This is what makes a reload
    # idempotent: whatever the previous launch left holding the lidar is gone
    # before the new one goes looking for it. `bash` in front of it because a
    # checkout that arrived by scp is mode 644 and the shebang is never consulted.
    if ! bash "$DIR/sweep.sh" >> "$LOG" 2>&1; then
        echo "--- sweep left something running at $(date -Is); launching anyway, "\
             "expect a port clash or a shared serial port ---" >> "$LOG"
    fi
    ros2 launch "$DIR/$LAUNCH" "$@" >> "$LOG" 2>&1 &
    child=$!
    wait "$child"
    status=$?
    echo "--- launch exited $status at $(date -Is), restarting in ${RETRY}s ---" >> "$LOG"
    sleep $RETRY
done
