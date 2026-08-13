#!/usr/bin/env bash
# Start exactly one instance of the stack (and optionally RViz) and return
# immediately.
#
# The earlier version waited around printing status, which held the SSH channel
# open; when that call was retried the previous launch was still running, and
# four stacked copies of the camera driver and RViz saturated the guest's 4 vCPUs
# until sshd could no longer complete a banner exchange. Launch, verify from a
# separate short-lived connection.
#
#   start_sensors.sh          sensors only
#   start_sensors.sh rviz     sensors + RViz on :0
set -eo pipefail

source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

bash "$HOME/ugv/bin/stop.sh" > /dev/null 2>&1 || true
pkill -9 -f rviz2 2>/dev/null || true
sleep 2

# Refuse to stack. If anything survived cleanup, stop rather than pile on.
if pgrep -f component_container > /dev/null; then
    echo "REFUSING: a component_container is still running"
    exit 1
fi

setsid nohup ros2 launch "$HOME/ugv/launch/bringup.launch.py" \
    > /tmp/bringup.log 2>&1 < /dev/null &
echo "launched bringup (geometry from the launch file's measured defaults)"

if [ "${1:-}" = "rviz" ]; then
    for _ in $(seq 1 40); do
        if ros2 topic list 2>/dev/null | grep -qx /scan; then break; fi
        sleep 1
    done
    DISPLAY=:0 setsid nohup rviz2 -d "$HOME/ugv/config/rover.rviz" \
        > /tmp/rviz.log 2>&1 < /dev/null &
    echo "launched rviz"
fi

exit 0
