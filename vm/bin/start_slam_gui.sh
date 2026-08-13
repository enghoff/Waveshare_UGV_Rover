#!/usr/bin/env bash
# What the desktop icon runs: start_slam.sh rviz, in a window worth reading.
#
# The icon could call start_slam.sh directly, but two of its behaviours make that
# a poor experience from a double-click. It takes up to a minute to reach RViz --
# it waits for the lidar's first /scan before starting it -- so a silent launch
# looks like a click that did nothing, and the natural response is to click again
# and stack a second stack on the first. And it refuses to start when a previous
# run survived cleanup, printing why and exiting 1; under Terminal=true the
# window closes the instant that happens, taking the only explanation with it.
#
# So: narrate the wait, and hold the window open on failure only.
set -eo pipefail

exec 2>&1   # interleave stderr into the terminal window, in order

echo "Starting SLAM with RViz."
echo "RViz waits for the lidar's first scan, so this takes up to a minute."
echo
echo "Keep the rover still until the map appears -- the first 15 seconds are"
echo "where the gyro's bias is measured."
echo

if bash "$HOME/ugv/bin/start_slam.sh" rviz; then
    echo
    echo "Logs: /tmp/slam.log, /tmp/rviz.log"
    echo "Stop the run with: bash ~/ugv/bin/stop.sh"
    sleep 6
    exit 0
fi

echo
echo "start_slam.sh failed -- nothing is running now."
echo "Tail of /tmp/slam.log, if the run got that far:"
tail -n 20 /tmp/slam.log 2>/dev/null || echo "(no log)"
echo
read -r -p "Press Enter to close this window. " _
exit 1
