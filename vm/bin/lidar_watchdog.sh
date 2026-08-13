#!/usr/bin/env bash
# Restart the lidar node when its serial port moves out from under it.
#
# ldlidar_stl_ros2_node opens the port once, at startup, and never reopens it.
# Its read loop treats a vanished device as an ordinary timeout -- demo.cpp logs
# "get ldlidar data is time out" on DATA_TIME_OUT and falls through to the next
# iteration -- so a node whose device has been unplugged does not exit, does not
# degrade, and does not stop. It spins at 10 Hz on a dead file descriptor at
# ~78% of a core, /scan stays advertised but silent, and rf2o and slam_toolbox
# sit downstream printing "Waiting for laser_scans...." forever. Nothing in the
# stack fails loudly enough to notice, which is how a run can be lost.
#
# Two ways the port moves:
#
#   * The device re-enumerates. Any USB disconnect gives it a new /dev/ttyACM*
#     and leaves the old inode deleted; the node keeps its handle on the corpse.
#     This is not hypothetical -- a failing USB audio dongle sharing the virtual
#     hub knocked the lidar off the bus 19 times in an hour before it was
#     removed (see setup/disable_usb_audio.sh).
#
#   * The node wins a race against udev at startup. start_slam.sh can open
#     /dev/rover-lidar while it still points at the previous ttyACM, moments
#     before udev repoints it at the one that just appeared.
#
# Both reduce to the same check, which is why this watches the port rather than
# the topic: compare the tty the node actually holds against the one
# /dev/rover-lidar resolves to now. A mismatch, or a handle marked (deleted),
# means the node can never recover on its own. Killing it is the fix -- the Node
# action carries respawn=True, so launch starts a fresh one that opens the
# symlink again and picks up wherever the device landed.
#
# Watching /scan instead would need a ROS subscription per check, and would
# confuse "the port went away" with "the lidar is spinning up", which is a
# normal several-second silence at every launch.
set -u

PORT="${1:-/dev/rover-lidar}"
INTERVAL="${2:-5}"

# A restarted node needs time to open the port before it is judged again. Two
# seconds of that is launch's respawn_delay.
GRACE=15

# The node holds exactly one tty. readlink prints "/dev/ttyACM0 (deleted)" once
# the device is gone, so the deleted case needs no separate test -- it simply
# never equals the symlink's current target.
node_tty() {
    local pid
    pid=$(pgrep -f '(^|/)ldlidar_stl_ros2_node' | head -1) || return 1
    [ -n "$pid" ] || return 1
    readlink /proc/"$pid"/fd/* 2>/dev/null |
        grep -m1 -E '^/dev/tty(ACM|USB)' || return 1
}

echo "lidar watchdog: following $PORT every ${INTERVAL}s"

misses=0
while true; do
    sleep "$INTERVAL"

    want=$(readlink -f "$PORT" 2>/dev/null) || want=""
    have=$(node_tty) || have=""

    # No node, or no device: nothing to heal. A missing node is launch's
    # business, and a missing device will not be fixed by restarting anything.
    if [ -z "$have" ] || [ -z "$want" ]; then
        misses=0
        continue
    fi

    if [ "$have" = "$want" ]; then
        misses=0
        continue
    fi

    # Require two consecutive mismatches. A single one is expected mid-replug,
    # in the moment after udev has moved the symlink and before the node has
    # been killed -- restarting on that would race the device's own recovery.
    misses=$((misses + 1))
    [ "$misses" -lt 2 ] && continue

    echo "$(date -Is) lidar holds '$have' but $PORT is '$want' -- restarting node"
    pkill -f '(^|/)ldlidar_stl_ros2_node'
    misses=0
    sleep "$GRACE"
done
