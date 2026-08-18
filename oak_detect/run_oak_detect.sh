#!/bin/sh
# Keep the OAK face detector running, and bring it back after a reboot.
#
# A `@reboot` crontab entry for `admin`, exactly as run_daemon.sh is and for the
# same reason: a system unit would need a sudo password we do not have from a
# script, a user unit would need `loginctl enable-linger`, and cron needs
# neither. Installed alongside the daemon's own entry:
#
#     @reboot /home/admin/ugv/oak_detect/run_oak_detect.sh
#     @reboot /home/admin/ugv/run_daemon.sh --vision --lidar --service 127.0.0.1:8768
#
#     pkill -f oak_detect/server.py     # reload; this restarts it
#     pkill -f run_oak_detect.sh        # stop, and stay stopped
#
# The restart loop earns its keep here more than it does for the daemon. The VPU
# has no flash and boots from the host every time, so anything that interrupts
# the USB link -- a brownout on the shared 5 V rail, a cable knocked at the
# camera end -- leaves a dead device and a server that can no longer answer. The
# fix in every one of those cases is to open it again from scratch, which is what
# restarting this does. The rover meanwhile behaves exactly as it does when MEDIA
# is unreachable: face tracking reports that the detector is not there.
#
# It waits for the camera to appear rather than assuming it. On a cold boot the
# USB tree enumerates well after cron fires, and a first attempt that raced it
# would burn the retry interval for nothing.

DIR="$(cd "$(dirname "$0")" && pwd)"
LOG="$DIR/oak_detect.log"
RETRY=15

# 03e7:2485 is the Myriad X in its ROM bootloader -- idle, waiting for a host,
# which is the state it is in whenever nothing has booted it. f63b would mean a
# previous run left it booted; that is fine too, and mvnc will reset it.
i=0
while ! lsusb | grep -qi '03e7:\(2485\|f63b\)' && [ $i -lt 40 ]; do
    sleep 3
    i=$((i + 1))
done

stop() {
    echo "--- run_oak_detect.sh signalled at $(date -Is), stopping ---" >> "$LOG"
    kill "$child" 2>/dev/null
    exit 0
}
trap stop INT TERM

echo "--- run_oak_detect.sh starting at $(date -Is) ---" >> "$LOG"
while true; do
    python3 "$DIR/server.py" "$@" >> "$LOG" 2>&1 &
    child=$!
    wait "$child"
    status=$?
    echo "--- server exited $status at $(date -Is), restarting in ${RETRY}s ---" >> "$LOG"
    sleep $RETRY
done
