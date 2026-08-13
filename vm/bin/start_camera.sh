#!/usr/bin/env bash
# Bring the OAK up alongside a running SLAM session, and return immediately.
#
# Returning immediately matters: holding the SSH channel open while the launch
# runs is what let a second invocation stack on top of the first and wedge the
# guest. setsid detaches it so closing the session does not kill it either.
#
# Refuses to start if a component_container is already alive -- depthai answers
# a second open with X_LINK_DEVICE_ALREADY_IN_USE, and the old instance's topics
# stay visible, which makes that failure look like a configuration problem.
set -eo pipefail

# -f, not -x: comm is truncated to 15 characters, so "component_conta" is all an
# exact-name match ever sees. Safe from self-matching only because this runs from
# a script file, whose own command line is just "bash start_camera.sh".
if pgrep -f component_container >/dev/null 2>&1; then
    echo "REFUSING: a component_container is already running. Run bin/stop.sh first."
    pgrep -af component_container
    exit 1
fi

source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

LOG=/tmp/camera.log
setsid nohup ros2 launch "$HOME/ugv/launch/camera.launch.py" "$@" \
    >"$LOG" 2>&1 < /dev/null &
echo "launched (pid $!), log: $LOG"
