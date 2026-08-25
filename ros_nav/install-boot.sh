#!/bin/sh
# Arrange for the ROS 2 stack to come back after a reboot, and say what changed.
#
#     ssh bpi-m4zero 'sh ~/ugv/ros_nav/install-boot.sh'          # mapping only
#     ssh bpi-m4zero 'sh ~/ugv/ros_nav/install-boot.sh --nav'    # mapping and Nav2
#     ssh bpi-m4zero 'sh ~/ugv/ros_nav/install-boot.sh --off'    # stop starting it
#
# A crontab @reboot entry, like the daemon and the depth service, and for the
# same reason: a system unit needs a sudo password no script here has, and a user
# unit needs `loginctl enable-linger`, which needs the same. cron is already
# running and needs neither.
#
# It also checks the *daemon's* entry, because the two are a pair now. Three
# things have to be true of it and each fails differently:
#
#   --board-bridge   or the base node has no odometry at all
#   --ros-nav        or the daemon offers no driving tools, so the drive console
#                    and the voice chat can watch the rover and not move it
#   no --lidar       that flag was deleted along with the daemon's own planner,
#                    so a crontab still carrying it is a daemon that will not
#                    start at all -- argparse refuses the unknown argument
#
# Getting any of them wrong is the most likely way to install this and find
# nothing works, so they are checked here rather than left to be discovered.

set -eu

HERE=$(cd "$(dirname "$0")" && pwd)
ENTRY="@reboot $HERE/run_ros_nav.sh"
ARGS=""

case "${1:-}" in
    --nav) ARGS=" --nav" ;;
    --off) ARGS="OFF" ;;
    "") ;;
    *) echo "usage: $0 [--nav|--off]"; exit 1 ;;
esac

current=$(crontab -l 2>/dev/null || true)
kept=$(printf '%s\n' "$current" | grep -v 'run_ros_nav\.sh' || true)

if [ "$ARGS" = "OFF" ]; then
    printf '%s\n' "$kept" | crontab -
    echo "== the ROS stack will no longer start at boot"
else
    printf '%s\n%s%s\n' "$kept" "$ENTRY" "$ARGS" | grep -v '^$' | crontab -
    echo "== installed: $ENTRY$ARGS"
fi

# ext4 here is mounted commit=120, so a crontab written and not flushed can be
# two minutes behind a power cut. A crontab edit has already been lost that way
# on this board.
sync

echo "== crontab now:"
crontab -l

echo
daemon_line=$(crontab -l 2>/dev/null | grep 'run_daemon\.sh' || true)
if [ -z "$daemon_line" ]; then
    echo "!! the rover daemon has no @reboot entry -- without it there is no board"
    echo "   bridge, so the base node will have no odometry."
elif printf '%s' "$daemon_line" | grep -q -- '--lidar'; then
    echo "!! the daemon still starts with --lidar. That flag no longer exists -- it"
    echo "   went with the daemon's own planner -- so this entry does not merely"
    echo "   fight the ROS stack for the serial port, it stops the daemon starting"
    echo "   at all. Change it:"
    echo "     crontab -l | sed 's/--lidar/--board-bridge --ros-nav/' | crontab - && sync"
elif ! printf '%s' "$daemon_line" | grep -q -- '--board-bridge'; then
    echo "!! the daemon does not start with --board-bridge, so the base node will"
    echo "   have nothing to read the wheels and gyro from. Add it to that entry."
elif ! printf '%s' "$daemon_line" | grep -q -- '--ros-nav'; then
    echo "!! the daemon starts without --ros-nav, so it will come up with no"
    echo "   driving or mapping tools -- 11 of them instead of 17. The drive"
    echo "   console will show a rover it cannot move and a map it cannot fetch."
    echo "   Add it to that entry, and note that changing the crontab is not"
    echo "   enough on its own: the running supervisor is holding the old"
    echo "   arguments, so it has to be replaced."
    echo "     crontab -l | sed 's|run_daemon.sh --vision --board-bridge|& --ros-nav|' | crontab - && sync"
    echo "     pkill -f 'ugv/run_daemon[.]sh' ; sleep 1 ; ~/ugv/restart.sh"
else
    echo "== the daemon's entry is right: --board-bridge --ros-nav, and no --lidar"
fi
