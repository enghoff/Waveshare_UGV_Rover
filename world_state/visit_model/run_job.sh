#!/bin/sh
# Keep the world model's job up, and bring it back after a reboot: two
# processes, each restarted if it stops.
#
# - scan_log.py listens to the lidar and navigation's pose, so it runs inside
#   navigation's ROS environment (RoboStack's conda Python), and writes them to
#   ~/.ugv/world/visit_model/lidar/.
# - job.py files the store's new looks with version 4 on the system Python,
#   where the model was verified: the conda environment has its own numpy and
#   no PIL.
#
# Started the way every service on this rover is: a `@reboot` crontab entry
# for the rover user, which install_job.sh puts down.
#
#     @reboot /home/jetson/ugv/world_state/visit_model/run_job.sh
#
#     ~/ugv/world_state/visit_model/restart_job.sh   # reload them
#     pkill -f visit_model/run_job.sh                 # stop, and stay stopped
#
# Both run niced: the job files a batch of looks a minute and never competes
# with driving or the voice model for the processor.

DIR="$(cd "$(dirname "$0")" && pwd)"
STATE="$HOME/.ugv/world/visit_model"
LOG="$STATE/job.log"
RETRY=30
mkdir -p "$STATE"

logger_loop() {
    . "$HOME/ugv/ros_nav/env.sh" > /dev/null 2>&1
    [ -f "$HOME/ugv/ros_nav/dds.sh" ] && . "$HOME/ugv/ros_nav/dds.sh" > /dev/null 2>&1
    while true; do
        echo "--- starting scan_log.py at $(date -Is) ---" >> "$LOG"
        nice -n 15 python3 "$DIR/scan_log.py" >> "$LOG" 2>&1
        echo "--- scan_log.py exited with $? at $(date -Is); again in $RETRY s ---" >> "$LOG"
        sleep "$RETRY"
    done
}

stop() {
    echo "--- run_job.sh signalled at $(date -Is), stopping ---" >> "$LOG"
    pkill -P "$logger" 2>/dev/null
    kill "$logger" "$child" 2>/dev/null
    exit 0
}
trap stop INT TERM

logger_loop &
logger=$!

while true; do
    echo "--- starting job.py at $(date -Is) ---" >> "$LOG"
    nice -n 15 /usr/bin/python3 "$DIR/job.py" >> "$LOG" 2>&1 &
    child=$!
    wait "$child"
    echo "--- job.py exited with $? at $(date -Is); again in $RETRY s ---" >> "$LOG"
    sleep "$RETRY"
done
