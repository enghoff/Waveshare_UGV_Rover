#!/bin/sh
# Keep the OAK awake as a depth camera, and bring it back after a reboot.
#
# A `@reboot` crontab entry for `admin`, exactly as run_daemon.sh is and for the
# same reason: a system unit would need a sudo password we do not have from a
# script, a user unit would need `loginctl enable-linger`, and cron needs neither.
#
#     @reboot /home/admin/ugv/run_daemon.sh --vision --board-bridge --ros-nav
#     @reboot /home/admin/ugv/oak_depth/run_oak_depth.sh
#
#     pkill -f oak_depth/depth_server.py   # reload; this restarts it
#     pkill -f run_oak_depth.sh            # stop, and stay stopped
#
# **The restart loop is the whole mechanism, not a safety net.** The Myriad X has
# no flash: it boots from its host over USB every time, and a booted device that
# stops hearing from that host kills itself on a 1500 ms watchdog. So a brownout
# on the shared 5 V rail, a cable knocked at the camera end, or this process
# dying all leave the same thing behind -- a camera in ROM bootloader state,
# waiting. Opening it again from scratch is the only repair, and that is what this
# does.
#
# It waits for the camera to appear rather than assuming it. On a cold boot the
# USB tree enumerates well after cron fires, and a first attempt that raced it
# would burn the retry interval for nothing.

DIR="$(cd "$(dirname "$0")" && pwd)"
LOG="$DIR/oak_depth.log"
# How long to wait before starting again, and which wait applies. A start that
# failed at once -- no device, a broken wheel -- would fail again at once, so it
# waits RETRY and the log takes a line every quarter minute rather than several a
# second. A server that had been RUNNING_S or more lost a camera that was
# working: on 2026-10-02 that was the OAK dropping off USB mid-stream, five
# times in an afternoon, and a flat RETRY on top of the 4-6 s firmware upload
# was twenty seconds without depth each time. That one only waits for the
# device to come back on the bus, which the kernel log put at about a second.
RETRY=15
RUNNING_S=30

# 03e7:2485 is the Myriad X in its ROM bootloader -- idle, waiting for a host,
# which is where it sits whenever nothing has booted it. f63b means something
# left it booted; depthai resets it on open, so that is fine too.
wait_for_camera() {    # $1 polls, $2 seconds apart
    i=0
    while ! lsusb | grep -qi '03e7:\(2485\|f63b\)' && [ $i -lt "$1" ]; do
        sleep "$2"
        i=$((i + 1))
    done
}
wait_for_camera 40 3

stop() {
    echo "--- run_oak_depth.sh signalled at $(date -Is), stopping ---" >> "$LOG"
    kill "$child" 2>/dev/null
    exit 0
}
trap stop INT TERM

echo "--- run_oak_depth.sh starting at $(date -Is) ---" >> "$LOG"
while true; do
    started=$(date +%s)
    python3 "$DIR/depth_server.py" "$@" >> "$LOG" 2>&1 &
    child=$!
    wait "$child"
    status=$?
    ran=$(( $(date +%s) - started ))
    if [ "$ran" -ge "$RUNNING_S" ]; then
        echo "--- depth_server exited $status at $(date -Is) after ${ran}s, restarting once the camera is back on USB ---" >> "$LOG"
        # Off the bus and back is what a drop looks like, so give it the second
        # it takes to leave before asking whether it has returned.
        sleep 1
        wait_for_camera 20 0.5
    else
        echo "--- depth_server exited $status at $(date -Is) after ${ran}s, restarting in ${RETRY}s ---" >> "$LOG"
        sleep $RETRY
    fi
done
