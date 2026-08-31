#!/bin/sh
# Keep the rover daemon running, and bring it back after a reboot.
#
# Installed as a `@reboot` crontab entry for `admin` rather than a systemd unit,
# because a system unit needs a `sudo` password we do not have from a script and
# a *user* unit needs `loginctl enable-linger`, which needs the same. cron is
# already running on this box and needs neither.
#
#     crontab -l           # what is installed
#     crontab -r           # stop starting it at boot
#     pkill -f rover_daemon.py    # stop the one running now; this restarts it
#
# The restart loop is not belt-and-braces. The daemon exits deliberately when the
# driver board does not answer its startup probe, and at boot that is a race it
# can lose: the host is up and the ESP32 comes up on its own schedule, so
# a daemon that started first has nothing to talk to. Retrying is the whole point.

DIR="$(cd "$(dirname "$0")" && pwd)"
LOG="$DIR/rover_daemon.log"
RETRY=15

# Wait for the port to exist at all. On a cold boot this is the disk and the
# kernel, not the board -- there is nothing to retry against until it is here.
# One name per board for the same header pins: ttyTHS1 is UART1 on the Jetson
# Orin Nano, ttyS4 is UART4 on the Banana Pi M4 Zero, ttyAMA0 is the Pi 1. Kept
# in step with SERIAL_CANDIDATES in board_link.py.
i=0
while [ ! -e /dev/ttyTHS1 ] && [ ! -e /dev/ttyAMA0 ] && [ ! -e /dev/ttyS4 ]         && [ $i -lt 40 ]; do
    sleep 3
    i=$((i + 1))
done

# Stop only when *this* is signalled, never because the daemon was. The first
# version quit whenever the child exited 130 or 143, reasoning that a signal
# meant somebody had stopped it deliberately -- but the obvious way to reload the
# daemon is `pkill -f rover_daemon.py`, and that took the supervisor down with
# it, which is the opposite of what a supervisor is for.
#
#     pkill -f 'ugv/rover_daemon.py'   # reload: the child dies, this restarts it
#     pkill -f run_daemon.sh           # stop: this exits, and stays exited
stop() {
    echo "--- run_daemon.sh signalled at $(date -Is), stopping ---" >> "$LOG"
    kill "$child" 2>/dev/null
    exit 0
}
trap stop INT TERM

echo "--- run_daemon.sh starting at $(date -Is) ---" >> "$LOG"
while true; do
    python3 "$DIR/rover_daemon.py" "$@" >> "$LOG" 2>&1 &
    child=$!
    wait "$child"
    status=$?
    echo "--- daemon exited $status at $(date -Is), restarting in ${RETRY}s ---" >> "$LOG"
    sleep $RETRY
done
