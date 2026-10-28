#!/bin/bash
# The ROS half of a recording session for Perception (recorder_node.py), with the
# ROS environment sorted out here. The daemon starts it when a session opens
# (rover_daemon/rover_recording.py); it exits by itself when the session ends.
#
# No `set -e`, for the reason run_record.sh gives: RoboStack's activation hooks
# return nonzero without anything being wrong.
DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=/dev/null
. "$DIR/env.sh"
# shellcheck source=/dev/null
. "$DIR/dds.sh"
LOG="$HOME/.ugv/recording/recorder.log"
mkdir -p "$(dirname "$LOG")"
exec python3 "$DIR/recorder_node.py" >> "$LOG" 2>&1
