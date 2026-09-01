#!/bin/sh
# Keep the Cosmos sidecar up, and bring it back after a reboot.
#
# A `@reboot` crontab entry for the rover user, the same arrangement the daemon,
# the console and the depth camera all use and for the same reasons: a system unit
# would want a sudo password no script here has, a user unit would want
# `loginctl enable-linger`, and cron wants neither.
#
#     @reboot /home/jetson/ugv/world_state/run_cosmos.sh
#
#     ~/ugv/world_state/restart.sh              # reload the server
#     pkill -f world_state/run_cosmos.sh        # stop, and stay stopped
#
# **Loopback only, and that is the security argument in full.** This port
# authenticates nothing and takes arbitrary text and images; bound to 127.0.0.1 it
# grants exactly what an ssh session on this board already grants, and the only
# client is the daemon in the next process along.
#
# The flags are here rather than in the crontab because they are a property of the
# board rather than of a run. `-t 4` is the whole of this Jetson's CPU: nothing
# deployed on this rover uses its GPU, and the two things that would compete for
# these cores -- SLAM and the face tracker -- are the reason the context is small
# and one request is served at a time.

DIR="$(cd "$(dirname "$0")" && pwd)"
VENDOR="$DIR/vendor"
LOG="$DIR/cosmos.log"
RETRY=15
PORT=8775

MODEL="$VENDOR/Cosmos-Reason2-2B-Q4_K_M.gguf"
MMPROJ="$VENDOR/mmproj-Cosmos-Reason2-2B-F16.gguf"
SERVER="$VENDOR/llama/llama-server"

if [ ! -x "$SERVER" ] || [ ! -f "$MODEL" ] || [ ! -f "$MMPROJ" ]; then
    echo "--- $(date -Is): no model installed; run install.sh" >> "$LOG"
    exit 1
fi

stop() {
    echo "--- run_cosmos.sh signalled at $(date -Is), stopping ---" >> "$LOG"
    kill "$child" 2>/dev/null
    exit 0
}
trap stop INT TERM

# The release archive is flat: the shared libraries sit beside the binary
# rather than in a lib/ of their own, and nothing on this host has them.
LD_LIBRARY_PATH="$VENDOR/llama:${LD_LIBRARY_PATH:-}"
export LD_LIBRARY_PATH

echo "--- run_cosmos.sh starting at $(date -Is) ---" >> "$LOG"
while true; do
    # -c 4096: one picture and a page of prompt. A larger context on this board
    #   buys nothing and costs KV cache in the 8 GB the whole rover shares.
    # --parallel 1: one inspection at a time, which is also what the Inspector's
    #   own lock enforces at the other end.
    # --no-webui: nothing here is meant to be opened in a browser.
    "$SERVER" \
        --model "$MODEL" \
        --mmproj "$MMPROJ" \
        --alias cosmos-reason2-2b-q4_k_m \
        --host 127.0.0.1 --port "$PORT" \
        --ctx-size 4096 --parallel 1 --threads 4 \
        --no-webui \
        >> "$LOG" 2>&1 &
    child=$!
    wait "$child"
    status=$?
    echo "--- llama-server exited $status at $(date -Is), restarting in ${RETRY}s ---" \
        >> "$LOG"
    sleep $RETRY
done
