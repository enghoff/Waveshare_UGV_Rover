#!/bin/sh
# Keep the world model's job up (job.py), and bring it back after a reboot.
#
# Started the way every service on this rover is: a `@reboot` crontab entry
# for the rover user, which install_job.sh puts down.
#
#     @reboot /home/jetson/ugv/world_state/visit_model/run_job.sh
#
#     ~/ugv/world_state/visit_model/restart_job.sh   # reload it
#     pkill -f visit_model/run_job.sh                 # stop, and stay stopped
#
# The job subscribes to the lidar, so it runs inside navigation's ROS
# environment, and it runs niced: it files a batch of looks a minute and never
# competes with driving or the voice model for the processor.

DIR="$(cd "$(dirname "$0")" && pwd)"
STATE="$HOME/.ugv/world/visit_model"
LOG="$STATE/job.log"
RETRY=30
mkdir -p "$STATE"

stop() {
    echo "--- run_job.sh signalled at $(date -Is), stopping ---" >> "$LOG"
    kill "$child" 2>/dev/null
    exit 0
}
trap stop INT TERM

# The navigation environment: ROS and the DDS settings the rest of the stack uses.
. "$HOME/ugv/ros_nav/env.sh" > /dev/null 2>&1
[ -f "$HOME/ugv/ros_nav/dds.sh" ] && . "$HOME/ugv/ros_nav/dds.sh" > /dev/null 2>&1

while true; do
    echo "--- starting job.py at $(date -Is) ---" >> "$LOG"
    nice -n 15 python3 "$DIR/job.py" >> "$LOG" 2>&1 &
    child=$!
    wait "$child"
    echo "--- job.py exited with $? at $(date -Is); again in $RETRY s ---" >> "$LOG"
    sleep "$RETRY"
done
