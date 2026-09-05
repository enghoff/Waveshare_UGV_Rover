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

if [ ! -f "$VENDOR/yoloe-11s-seg-objectness.onnx" ]; then
    echo "--- $(date -Is): no perception models; run install_perception.sh" >> "$LOG"
    exit 1
fi

stop() {
    echo "--- run_perception.sh signalled at $(date -Is), stopping ---" >> "$LOG"
    kill "$child" 2>/dev/null
    exit 0
}
trap stop INT TERM

# **The board sometimes boots with no GPU**, and TensorRT does not survive meeting
# one: the server starts, answers /health, and dies with SIGSEGV the first time a
# look asks for an inference runtime, so the console reports a sidecar that is not
# answering and nobody is told about a GPU. Reloading the driver fixes it and
# needs root, which is why that half lives in `gpu_ctl.sh` behind the sudo rule
# `install_gpu_recovery.sh` puts down.
#
# It runs before every start rather than once at boot, because this restart loop
# is also what catches a GPU that goes away mid-life. It costs one stat when there
# is a GPU, and gpu_ctl.sh holds itself to one reload every five minutes when
# there is not, so a board that genuinely has no GPU logs a line now and then
# instead of reloading a driver every fifteen seconds.
gpu_check() {
    [ -e /dev/nvgpu/igpu0/ctrl ] && return 0
    if [ -x /usr/local/sbin/gpu_ctl.sh ]; then
        sudo -n /usr/local/sbin/gpu_ctl.sh recover >> "$LOG" 2>&1
    else
        echo "--- $(date -Is): no GPU, and /usr/local/sbin/gpu_ctl.sh is not" \
             "installed; run install_gpu_recovery.sh ---" >> "$LOG"
    fi
}

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
    gpu_check
    python3 "$DIR/perception_server.py" --port "$PORT" >> "$LOG" 2>&1 &
    child=$!
    wait "$child"
    status=$?
    echo "--- perception_server.py exited $status at $(date -Is), restarting in ${RETRY}s ---" \
        >> "$LOG"
    sleep $RETRY
done
