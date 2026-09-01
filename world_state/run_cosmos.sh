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
# board rather than of a run.
#
# **This is the one thing on the rover that uses the GPU.** There is no CUDA
# toolkit on this JetPack, so nothing here can reach the GPU that way -- but the
# Vulkan driver works, llama.cpp speaks Vulkan, and the released Vulkan build is
# 27 MB with no toolchain to install. Measured on the rover: an inspection went
# from 38 s to 9.5 s. `-t 4` still matters because the vision projector and the
# sampler stay on the CPU.

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
    # --n-gpu-layers 99: all of them. llama.cpp would rather size this itself and
    #   says so -- "failed to fit params to free device memory" -- because the
    #   GPU's memory here *is* the system's and it cannot tell what the rest of
    #   the rover is about to want. Setting it explicitly is the answer: 99 is
    #   "every layer", the model is 1.3 GB, and it was measured alongside the
    #   perception sidecar with 1.5 GB still free. **Set this to 0 to go back to
    #   the CPU** -- the same binary carries both backends, so there is nothing to
    #   reinstall if the driver ever misbehaves.
    # --cache-ram 0: **this one stops the rover running out of memory.** The
    #   default is 8192 MiB of prompt cache on a board with 7485 MiB in it and no
    #   swap, so the server grows by about a hundred megabytes an inspection and
    #   never gives any back. Measured: sixteen inspections took the board from
    #   1.5 GB free to 48 MB, at which point the out-of-memory killer is choosing
    #   between the language model and the process that owns STOP. With the cache
    #   off it settles at 3.2 GB after six and stays there, and an inspection is
    #   no slower -- every request here is a new picture with a new prompt, so
    #   there was never anything in that cache worth reusing.
    #
    #   This was not introduced by the switch to Vulkan; the CPU build leaks at
    #   the same rate and always did. The POC's note that "the sidecar holds
    #   about 4 GB" was this, caught halfway up.
    "$SERVER" \
        --model "$MODEL" \
        --mmproj "$MMPROJ" \
        --alias cosmos-reason2-2b-q4_k_m \
        --host 127.0.0.1 --port "$PORT" \
        --ctx-size 4096 --parallel 1 --threads 4 \
        --n-gpu-layers 99 \
        --cache-ram 0 \
        --no-webui \
        >> "$LOG" 2>&1 &
    child=$!
    wait "$child"
    status=$?
    echo "--- llama-server exited $status at $(date -Is), restarting in ${RETRY}s ---" \
        >> "$LOG"
    sleep $RETRY
done
