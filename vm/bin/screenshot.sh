#!/usr/bin/env bash
# Screenshot the guest desktop.
#
# Must run from a file, not as an inline ssh command: pgrep/pkill -f match
# against full command lines, so an inline 'pkill -f xfce4-screensaver' matches
# the very shell running it and kills the SSH session. From a script the command
# line is just 'bash screenshot.sh', so there is nothing to self-match.
set -eo pipefail
export DISPLAY=:0

pkill -9 -f xfce4-screensaver 2>/dev/null || true
xset s off || true
xset s noblank || true
xset -dpms || true
sleep 2

scrot -o /tmp/screen.png
ls -l /tmp/screen.png

echo "=== process counts (accurate: script cmdline does not self-match) ==="
for p in ldlidar_stl_ros2_node rf2o_laser_odometry_node async_slam_toolbox_node rviz2 static_transform_publisher; do
    echo "  $p: $(pgrep -c -x "$p" 2>/dev/null || pgrep -cf "/$p" 2>/dev/null || echo 0)"
done
uptime
