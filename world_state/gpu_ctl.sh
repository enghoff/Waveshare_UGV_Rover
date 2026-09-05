#!/bin/sh
# The privileged half of the perception sidecar: put the GPU back when the Orin
# has booted without one.
#
#     gpu_ctl.sh status    # is there a usable GPU on this board
#     gpu_ctl.sh recover   # reload the driver if there is not, and say what it did
#
# **This board sometimes comes up with no GPU at all** -- twice in three boots on
# 2026-09-05. The driver's probe finishes half done: `/dev/nvgpu/igpu0/` holds
# `power` and nothing else where a healthy boot has fifteen nodes, and the kernel
# logs `invalid mem acr_falcon2_sysmem_desc` at every attempt to use it. CUDA then
# reports no device, and `perception_server.py` dies with SIGSEGV inside TensorRT
# the first time a look asks for an inference runtime -- so the console says the
# sidecar is not answering and the world state stays empty, three layers away from
# the cause.
#
# **Reloading the module is the only thing observed to fix it.** It re-runs the
# probe that came up short, takes about four seconds, and worked both times the
# fault has been seen. A reboot does not: on 2026-09-05 the first one left the GPU
# exactly as dead and the second brought it up, which makes it a coin flip rather
# than a repair. What causes the bad probe is not known; nothing changed on the
# rover between the last boot that worked and the first that did not.
#
# It needs root and the sidecar runs as an ordinary account, so
# `install_gpu_recovery.sh` gives this one path a passwordless sudo rule. Nothing
# here takes an argument that reaches a command: the two words above are all it
# accepts, and everything else is refused.

set -u

# The node that says the driver got all the way up. A dead boot leaves the
# directory in place with only `power` in it, so the directory existing proves
# nothing and this asks for the one node CUDA opens first.
NODE=${NODE:-/dev/nvgpu/igpu0/ctrl}
MODULE=nvgpu

# Where the last attempt is remembered, and how long before another is allowed.
# The wrapper retries every fifteen seconds and would otherwise reload the driver
# every fifteen seconds for as long as a genuinely broken GPU stayed broken. On
# tmpfs, so a reboot starts with a clean slate -- which is right, because a reboot
# is exactly when the fault appears.
STAMP=${STAMP:-/run/gpu_ctl.stamp}
COOLDOWN=${COOLDOWN:-300}

# How long the driver is given to bring the node back before this reports failure.
SETTLE=${SETTLE:-15}

usage() {
    echo "usage: gpu_ctl.sh status|recover" >&2
    exit 2
}

up() {
    [ -e "$NODE" ]
}

# Seconds since the stamp was last written, or the cooldown itself when there is
# no stamp, which lets the first attempt of a boot through.
since_last() {
    if [ ! -e "$STAMP" ]; then
        echo "$COOLDOWN"
        return
    fi
    now=$(date +%s)
    then=$(date -r "$STAMP" +%s 2>/dev/null || echo 0)
    echo $((now - then))
}

recover() {
    if up; then
        echo "gpu_ctl: the GPU is up, nothing to do"
        return 0
    fi

    waited=$(since_last)
    if [ "$waited" -lt "$COOLDOWN" ]; then
        echo "gpu_ctl: no GPU, and the driver was reloaded ${waited}s ago;" \
             "leaving it alone for another $((COOLDOWN - waited))s"
        return 1
    fi

    if [ "$(id -u)" != 0 ]; then
        echo "gpu_ctl: no GPU, and reloading $MODULE needs root" >&2
        return 1
    fi

    : > "$STAMP"
    echo "gpu_ctl: no GPU ($NODE is missing); reloading $MODULE"
    if ! rmmod "$MODULE" 2>&1; then
        echo "gpu_ctl: $MODULE would not unload; something is holding it" >&2
        return 1
    fi
    if ! modprobe "$MODULE" 2>&1; then
        echo "gpu_ctl: $MODULE would not load again" >&2
        return 1
    fi

    # The nodes appear a moment after the module does, so this waits for the one
    # that matters rather than reporting on the module having loaded.
    waited=0
    while [ "$waited" -lt "$SETTLE" ]; do
        if up; then
            echo "gpu_ctl: the GPU is back after ${waited}s"
            return 0
        fi
        sleep 1
        waited=$((waited + 1))
    done
    echo "gpu_ctl: $MODULE reloaded and the GPU did not come back in ${SETTLE}s" >&2
    return 1
}

case ${1:-} in
    status)
        if up; then
            echo "gpu_ctl: the GPU is up"
            exit 0
        fi
        echo "gpu_ctl: no GPU ($NODE is missing)"
        exit 1
        ;;
    recover)
        recover
        exit $?
        ;;
    *)
        usage
        ;;
esac
