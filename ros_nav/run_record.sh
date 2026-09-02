#!/bin/bash
# Record a drive for dwb_replay.py, with the ROS environment sorted out here.
#
#   ssh orin '~/ugv/ros_nav/run_record.sh --seconds 240'
#
# **The point of this file is that the command line has no quotes in it.** The
# obvious one-liner is
#
#   ssh orin 'cd ~/ugv/ros_nav && bash -c "source ./env.sh; python3 ..."'
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
# shellcheck source=/dev/null
. "$DIR/dds.sh"

# `rclpy.spin_once` does not honour its timeout when CycloneDDS is wedged, so a
# `--seconds 60` recording can sit in the loop for tens of minutes as a second
# participant on a four-core board. The watchdog in nav_record.py is the real
# backstop; this is the one that still fires if Python itself is stuck.
secs=180
prev=
for arg in "$@"; do
    if [ "$prev" = "--seconds" ]; then
        secs=${arg%%.*}
    fi
    prev=$arg
done
case $secs in
    ''|*[!0-9]*) secs=180 ;;
esac
exec timeout --kill-after=15 $((secs + 90)) python3 "$DIR/nav_record.py" "$@"
