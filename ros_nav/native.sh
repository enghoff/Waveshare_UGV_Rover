#!/bin/bash
# Run a command against /opt/ros/jazzy instead of against the conda ROS.
#
#     ~/ugv/ros_nav/native.sh ros2 topic list
#     ~/ugv/ros_nav/native.sh ros2 interface show rtabmap_msgs/msg/Info
#     ~/ugv/ros_nav/native.sh python3 ~/ugv/ros_nav/slam_compare.py --seconds 120
#
# ## Why there are two ROS installations to choose between
#
# Everything install.sh puts on this rover comes from RoboStack, unpacked into
# ~/miniforge3/envs/ros with no sudo at all -- and RoboStack publishes no
# rtabmap package, for any platform, under any name. So RTAB-Map comes from
# Ubuntu's own ROS 2 Jazzy packages in /opt/ros/jazzy instead, which
# install-rtabmap.sh puts there, and anything that needs to speak rtabmap's
# message types has to run against that install rather than the conda one.
#
# ## Why this is `env -i` and not `source /opt/ros/jazzy/setup.bash`
#
# Two ROS installations on one board are fine as long as no single process is
# asked to use both. Sourcing the native setup on top of an activated conda
# environment does exactly that: conda's lib directory stays ahead of the system
# one on LD_LIBRARY_PATH, so Ubuntu's binaries load RoboStack's libstdc++, its
# libtinyxml2 and its rclcpp. That does not fail cleanly. It fails as a symbol
# lookup error naming a library nobody mentioned, or it loads and misbehaves.
#
# The launch file makes this trap easy to fall into, because launch hands every
# child the environment it was itself started with -- and it was started from
# run_ros_nav.sh, which sources env.sh. So the environment is not adjusted here,
# it is discarded: `env -i` and a fresh start from what a login would have given
# us. The two halves of the stack then meet only where they should, on the wire,
# both pointed at CycloneDDS on loopback by dds.sh and both on domain 42.
#
# ROS_DOMAIN_ID is the one ROS variable carried across, so that a caller who has
# moved the rover off domain 42 moves this with it.

set -eu

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SELF="$DIR/$(basename "${BASH_SOURCE[0]}")"
ROS_NATIVE=${ROS_NATIVE:-/opt/ros/jazzy}

if [ $# -eq 0 ]; then
    echo "usage: native.sh <command> [args...]" >&2
    exit 2
fi

if [ "${ROS_NAV_CLEAN_ENV:-}" != 1 ]; then
    exec env -i \
        ROS_NAV_CLEAN_ENV=1 \
        HOME="$HOME" \
        USER="${USER:-$(id -un)}" \
        LOGNAME="${LOGNAME:-$(id -un)}" \
        PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
        TERM="${TERM:-dumb}" \
        LANG="${LANG:-C.UTF-8}" \
        ROS_NATIVE="$ROS_NATIVE" \
        ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}" \
        bash "$SELF" "$@"
fi

if [ ! -r "$ROS_NATIVE/setup.bash" ]; then
    echo "native.sh: $ROS_NATIVE is not there -- run install-rtabmap.sh" >&2
    exit 1
fi

# The nounset dance around the sourcing is the same one env.sh does around
# conda's activation hooks, and for the same reason: ROS's own setup.bash reads
# $AMENT_TRACE_SETUP_FILES without a default, so under `set -u` -- which every
# careful script in this repository uses -- it dies with
# "AMENT_TRACE_SETUP_FILES: unbound variable" and takes the caller with it. That
# message names ROS's file rather than this one and reads as a broken install.
# Turned off across the sourcing only, and put back exactly as it was found.
case $- in *u*) _had_nounset=1 ;; *) _had_nounset=0 ;; esac
set +u
# shellcheck disable=SC1091
. "$ROS_NATIVE/setup.bash"
[ "$_had_nounset" = 1 ] && set -u
unset _had_nounset

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# The loopback XML and LOCALHOST discovery, from the same file the conda half
# sources -- read off disk on every start, so a deploy of it is picked up here
# too. If it is missing dds.sh says so loudly, rather than quietly discovering on
# the LAN and taking the graph down at the next wifi_dual failover.
# shellcheck disable=SC1091
. "$DIR/dds.sh"

exec "$@"
