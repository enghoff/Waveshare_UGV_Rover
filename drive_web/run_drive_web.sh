#!/bin/sh
# Serve the drive console from the rover, and bring it back after a reboot.
#
# A `@reboot` crontab entry for `admin`, exactly as run_daemon.sh is and for the
# same reason: a system unit would need a sudo password we do not have from a
# script, a user unit would need `loginctl enable-linger`, and cron needs neither.
#
#     @reboot /home/admin/ugv/run_daemon.sh --vision --board-bridge --ros-nav
#     @reboot /home/admin/ugv/oak_depth/run_oak_depth.sh
#     @reboot /home/admin/ugv/drive_web/run_drive_web.sh
#
#     pkill -f drive_web/drive_web.py    # reload; this restarts it
#     pkill -f run_drive_web.sh          # stop, and stay stopped
#
# The page is http://<this host>:8771/ -- 8770 is already oak_depth. The daemon
# is on loopback, so a phone on the LAN talks HTTP to this process and this
# process talks the six tool connections to 127.0.0.1:8769. `--idle` is why a
# process that lives from boot is not a client of that daemon overnight.

DIR="$(cd "$(dirname "$0")" && pwd)"
LOG="$DIR/drive_web.log"
RETRY=15

stop() {
    echo "--- run_drive_web.sh signalled at $(date -Is), stopping ---" >> "$LOG"
    kill "$child" 2>/dev/null
    exit 0
}
trap stop INT TERM

echo "--- run_drive_web.sh starting at $(date -Is) ---" >> "$LOG"

# The certificate, before the console that serves it. This is idempotent and
# costs one openssl call when nothing has changed -- but this board is wifi-only
# and its address moves between three house networks, and a certificate is
# checked against the address that was typed. So the one moment worth re-checking
# it is exactly here, after a boot that may have landed somewhere new.
sh "$DIR/make_cert.sh" >> "$LOG" 2>&1

while true; do
    python3 -u "$DIR/drive_web.py" >> "$LOG" 2>&1 &
    child=$!
    wait "$child"
    status=$?
    echo "--- drive_web exited $status at $(date -Is), restarting in ${RETRY}s ---" >> "$LOG"
    sleep $RETRY
done
