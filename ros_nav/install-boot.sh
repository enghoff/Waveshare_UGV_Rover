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
# It also checks the *daemon's* entry, because the two are a pair now. The rover
# daemon must run with `--board-bridge` or the base node has no odometry, and it
# must run without `--lidar` or the two stacks fight over one serial port -- the
# daemon would win, silently, and slam_toolbox would wait for a scan that never
# comes. Getting that wrong is the single most likely way to install this and
# find nothing works, so it is checked here rather than left to be discovered.

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
    echo "!! the daemon still starts with --lidar, so it will take the lidar and"
    echo "   slam_toolbox will wait for a scan that never arrives. Change that entry"
    echo "   to --board-bridge:"
    echo "     crontab -l | sed 's/--lidar/--board-bridge/' | crontab - && sync"
elif ! printf '%s' "$daemon_line" | grep -q -- '--board-bridge'; then
    echo "!! the daemon does not start with --board-bridge, so the base node will"
    echo "   have nothing to read the wheels and gyro from. Add it to that entry."
else
    echo "== the daemon's entry is right: --board-bridge, and no --lidar"
fi
