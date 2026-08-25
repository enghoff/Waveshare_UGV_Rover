#!/bin/bash
# Record a drive for dwb_replay.py, with the ROS environment sorted out here.
#
#   ssh bpi-m4zero '~/ugv/ros_nav/run_record.sh --seconds 240'
#
# **The point of this file is that the command line has no quotes in it.** The
# obvious one-liner is
#
#   ssh bpi-m4zero 'cd ~/ugv/ros_nav && bash -c "source ./env.sh; python3 ..."'
#
# and from PowerShell that does not work, in a way that takes a while to see.
# PowerShell strips the inner double quotes on its way to a native command, so
# the rover receives `bash -c source ./env.sh; python3 ...`: bash runs the
# builtin `source` with no filename and `./env.sh` as its $0, which fails as
#
#   ./env.sh: line 1: source: filename argument required
#
# naming a file whose line 1 is a comment. The `python3` then runs in a shell
# that never got the conda environment and dies with "No module named 'rclpy'",
# so the visible symptom is a missing ROS install. Both messages point away
# from the actual fault, which is the quoting.
#
# bash and not sh: env.sh needs it, and the reason is in env.sh's own header.
#
# No `set -e`, and that is deliberate rather than sloppy: RoboStack's activation
# hooks run dozens of commands and some of them return nonzero without anything
# being wrong, so errexit across the sourcing kills the launcher silently, part
# way through, with no message at all. run_ros_nav.sh and restart.sh source it
# the same bare way for the same reason.
DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=/dev/null
. "$DIR/env.sh"
exec python3 "$DIR/nav_record.py" "$@"
