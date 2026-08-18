#!/bin/sh
# Reload the face detector after deploying a new server.py or oak.py.
#
#     ssh rpi ~/ugv/oak_detect/restart.sh
#
# Separate from run_oak_detect.sh for the same reason ~/ugv/restart.sh is separate
# from run_daemon.sh, and it is worth repeating because it caught this directory
# too: a `pkill -f oak_detect/server.py` typed into an ssh command line *matches
# that ssh command*, because the pattern is in its own text. The session dies
# mid-sentence, the output vanishes, and it looks as though the service failed to
# come back. Here the pattern lives in a file instead, where nothing else can
# quote it.
#
# Only the server is killed, never the supervisor. The supervisor is what notices
# and starts the next one, and it is also where the arguments live -- exactly as
# for the daemon.
DIR="$(cd "$(dirname "$0")" && pwd)"

pkill -f "$DIR/server.py" || {
    echo "nothing was running; the supervisor will start one within 15 s" >&2
    exit 0
}

# The device is booted from scratch on every open, so this is not a process
# restart time -- it is a firmware upload and a graph upload, and it is the
# reason the daemon's own detector probe is bounded rather than instant.
i=0
while [ $i -lt 40 ]; do
    sleep 2
    if curl -fs -m 3 http://127.0.0.1:8768/health >/dev/null 2>&1; then
        curl -s -m 3 http://127.0.0.1:8768/health
        echo
        exit 0
    fi
    i=$((i + 1))
done

echo "the detector did not answer within 80 s -- see $DIR/oak_detect.log" >&2
exit 1
