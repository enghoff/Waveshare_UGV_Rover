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
RETRY=15

# 03e7:2485 is the Myriad X in its ROM bootloader -- idle, waiting for a host,
# which is where it sits whenever nothing has booted it. f63b means something
# left it booted; depthai resets it on open, so that is fine too.
i=0
while ! lsusb | grep -qi '03e7:\(2485\|f63b\)' && [ $i -lt 40 ]; do
    sleep 3
    i=$((i + 1))
done

stop() {
    echo "--- run_oak_depth.sh signalled at $(date -Is), stopping ---" >> "$LOG"
    kill "$child" 2>/dev/null
    exit 0
}
trap stop INT TERM

echo "--- run_oak_depth.sh starting at $(date -Is) ---" >> "$LOG"
while true; do
    python3 "$DIR/depth_server.py" "$@" >> "$LOG" 2>&1 &
    child=$!
    wait "$child"
    status=$?
    echo "--- depth_server exited $status at $(date -Is), restarting in ${RETRY}s ---" >> "$LOG"
    sleep $RETRY
done
