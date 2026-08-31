#!/bin/sh
# Arrange for the ROS 2 stack to come back after a reboot, and say what changed.
#
#     ssh orin 'sh ~/ugv/ros_nav/install-boot.sh'          # mapping only
#     ssh orin 'sh ~/ugv/ros_nav/install-boot.sh --nav'    # mapping and Nav2
#     ssh orin 'sh ~/ugv/ros_nav/install-boot.sh --off'    # stop starting it
#
# Which mapper owns the map is part of the same entry, because the supervisor
# takes its arguments from the crontab and nowhere else:
#
#     ... install-boot.sh --nav --rtabmap'         # RTAB-Map owns map -> odom
#     ... install-boot.sh --nav --slam-toolbox'    # slam_toolbox does
#
# **Naming neither keeps the mapper the entry already has.** That is deliberate
# and it is the difference between this script and a fresh install: somebody
# turning Nav2 on has not thereby asked to change mappers, and silently reverting
# to the launch file's default would be the same class of mistake as the hand
# relaunch that once dropped `--vision` from the daemon and lost the rover its
# camera. What is installed is printed either way, so an entry that is not what
# was wanted says so on screen rather than at the next reboot.
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
MAPPER=""
OFF=no

usage="usage: $0 [--nav] [--rtabmap|--slam-toolbox]   or   $0 --off"
while [ $# -gt 0 ]; do
    case "$1" in
        --nav)          ARGS=" --nav" ;;
        # A launch argument rather than a flag, so it goes after `--nav`: the
        # supervisor takes the first word for itself and hands the rest to
        # `ros2 launch`.
        --rtabmap)      MAPPER=" rtabmap:=primary" ;;
        --slam-toolbox) MAPPER=" rtabmap:=off" ;;
        --off)          OFF=yes ;;
        *) echo "$usage"; exit 1 ;;
    esac
    shift
done

current=$(crontab -l 2>/dev/null || true)
kept=$(printf '%s\n' "$current" | grep -v 'run_ros_nav\.sh' || true)

# The mapper the entry already carries, when this run did not name one. Read back
# out of the crontab rather than remembered anywhere, because the crontab is the
# only place it lives.
if [ -z "$MAPPER" ]; then
    existing=$(printf '%s\n' "$current" |
               sed -n 's|.*run_ros_nav\.sh.*[ ]\(rtabmap:=[a-z]*\).*|\1|p' | head -1)
    if [ -n "$existing" ]; then
        MAPPER=" $existing"
        echo "== keeping the mapper the entry already had: $existing"
    fi
fi

if [ "$OFF" = yes ]; then
    printf '%s\n' "$kept" | crontab -
    echo "== the ROS stack will no longer start at boot"
else
    printf '%s\n%s%s%s\n' "$kept" "$ENTRY" "$ARGS" "$MAPPER" |
        grep -v '^$' | crontab -
    echo "== installed: $ENTRY$ARGS$MAPPER"
fi

# ext4 here is mounted commit=120, so a crontab written and not flushed can be
# two minutes behind a power cut. A crontab edit has already been lost that way
# on this board.
sync

echo "== crontab now:"
crontab -l

# A boot entry naming a mapper that is not installed is a rover that comes up
# with a lidar, wheels, Nav2 and no map at all -- and the way it fails is in the
# launch log, hours later, rather than here. RTAB-Map is the one that can be
# missing: it is not part of the conda environment everything else comes from,
# because RoboStack publishes no package for it, so it is Ubuntu's own binary in
# /opt/ros/jazzy and install-rtabmap.sh is what puts it there.
RTABMAP_BIN=/opt/ros/jazzy/lib/rtabmap_slam/rtabmap
if [ "$MAPPER" = " rtabmap:=primary" ] && [ ! -x "$RTABMAP_BIN" ]; then
    echo "!! this entry makes RTAB-Map the mapper, but $RTABMAP_BIN"
    echo "   is not there. Install it first, or the stack boots with no map:"
    echo "     sh ~/ugv/ros_nav/install-rtabmap.sh"
fi

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
