#!/bin/sh
# Keep the perception sidecar up, and bring it back after a reboot.
#
# The arrangement every service on this rover uses: a `@reboot` crontab entry for
# the rover user, because a system unit would want a sudo password no script here
# has and a user unit would want `loginctl enable-linger`.
#
#     @reboot /home/jetson/ugv/world_state/run_perception.sh
#
#     ~/ugv/world_state/restart_perception.sh   # reload it
#     pkill -f world_state/run_perception.sh    # stop, and stay stopped
#
# **Not preloaded.** The models are opened on the first look rather than at
# startup, so a rover that is driving around and never inspecting anything pays
# neither the seven seconds nor the memory. The cost of that is that the first
# look after a reboot is seven seconds slower than the rest, which is worth it on
# a board where the language model next door already holds four gigabytes.

DIR="$(cd "$(dirname "$0")" && pwd)"
VENDOR="$DIR/vendor"
LOG="$DIR/perception.log"
RETRY=15
PORT=8776

if [ ! -f "$VENDOR/FastSAM-s.onnx" ]; then
    echo "--- $(date -Is): no perception models; run install_perception.sh" >> "$LOG"
    exit 1
fi

stop() {
    echo "--- run_perception.sh signalled at $(date -Is), stopping ---" >> "$LOG"
    kill "$child" 2>/dev/null
    exit 0
}
trap stop INT TERM

# onnxruntime, tokenizers and OpenCV are unpacked wheels rather than installed
# packages, because this host's Python is externally managed and sudo wants a
# password no deploy script has. perceive.py appends both directories to sys.path
# itself; this is here so that anything else the sidecar imports finds them too.
PYTHONPATH="$VENDOR:$(dirname "$DIR")/vendor:${PYTHONPATH:-}"
export PYTHONPATH
# Unbuffered, so a log tailed during a look shows the look rather than showing it
# when the buffer fills.
PYTHONUNBUFFERED=1
export PYTHONUNBUFFERED

echo "--- run_perception.sh starting at $(date -Is) ---" >> "$LOG"
while true; do
    python3 "$DIR/perception_server.py" --port "$PORT" >> "$LOG" 2>&1 &
    child=$!
    wait "$child"
    status=$?
    echo "--- perception_server.py exited $status at $(date -Is), restarting in ${RETRY}s ---" \
        >> "$LOG"
    sleep $RETRY
done
