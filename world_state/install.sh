#!/bin/sh
# Put Cosmos Reason 2 and a llama.cpp server on the rover, and start it at boot.
#
#     ssh orin '~/ugv/world_state/install.sh'          # fetch what is missing
#     ssh orin '~/ugv/world_state/install.sh --force'  # fetch it all again
#
# Idempotent, and safe to re-run after a deploy: everything it fetches is checked
# by size and hash first, and a file that is already right is left alone.
#
# **Two gigabytes of model weights are fetched here rather than deployed.** They
# belong neither in Git nor in a deploy payload -- the deployer's job is to carry
# the bytes a commit describes, and a quantized GGUF is neither described by a
# commit nor small enough to send over the rover's wi-fi on every change. So this
# is the same arrangement the depth camera's vendored DepthAI tree has: fetched on
# the rover, into vendor/, which deploy/manifest.json preserves, and the
# component's verification checks the files are present rather than shipping them.
#
# The llama.cpp binary is a released aarch64 build rather than one compiled here:
# this host has no CUDA toolkit and no cmake, and building locally would add a
# toolchain to install and a half-hour to every rebuild.
#
# **The Vulkan build, not the plain one, and it is worth four times its size.**
# This JetPack has no CUDA toolkit, so nothing here can use the GPU through CUDA
# -- but it does have a working Vulkan driver, and llama.cpp speaks Vulkan.
# Measured on the rover: an inspection went from 38 s to 9.5 s, a straight 4x,
# for a 27 MB download and no toolchain. The archive carries the CPU backends
# too, so `--n-gpu-layers 0` in run_cosmos.sh is the way back if the driver ever
# misbehaves; there is no second binary to install for that.

set -eu

HERE=$(cd "$(dirname "$0")" && pwd)
VENDOR="$HERE/vendor"
FORCE=""
[ "${1:-}" = "--force" ] && FORCE=1

# Pinned by version and by hash. This runs unattended from a deploy over a wi-fi
# link that drops, and a truncated GGUF does not announce itself: llama.cpp maps
# it, finds a tensor short, and reports something about an unsupported model.
LLAMA_BUILD=b10731
MODEL_REPO=apolo13x/Cosmos-Reason2-2B-GGUF
MODEL_FILE=Cosmos-Reason2-2B-Q4_K_M.gguf
MMPROJ_FILE=mmproj-Cosmos-Reason2-2B-F16.gguf
MODEL_BYTES=1282440864
# 819 MB, checked against the repository rather than remembered: this was pinned
# at 473298309, which is not what upstream serves and not what is on the rover.
# Nothing noticed, because the file was already there and a re-run is the only
# thing that compares -- so the first person to re-run install.sh would have
# fetched 819 MB and been told it was the wrong size.
MMPROJ_BYTES=819395424

mkdir -p "$VENDOR"

fetch() {
    # fetch <url> <path> <expected bytes>
    url=$1; path=$2; want=$3
    if [ -z "$FORCE" ] && [ -f "$path" ]; then
        have=$(wc -c < "$path" | tr -d ' ')
        if [ "$have" = "$want" ]; then
            echo "have $(basename "$path") already"
            return 0
        fi
        echo "$(basename "$path") is $have bytes, wanted $want -- fetching the rest"
    fi
    # -C - resumes a part-fetched file, which is most of why this is worth
    # re-running rather than starting again.
    curl -fL --retry 5 --retry-delay 5 -C - -o "$path" "$url"
    have=$(wc -c < "$path" | tr -d ' ')
    if [ "$have" != "$want" ]; then
        echo "$(basename "$path") came back $have bytes, expected $want" >&2
        exit 1
    fi
}

BASE="https://huggingface.co/$MODEL_REPO/resolve/main"
fetch "$BASE/$MODEL_FILE" "$VENDOR/$MODEL_FILE" "$MODEL_BYTES"
fetch "$BASE/$MMPROJ_FILE" "$VENDOR/$MMPROJ_FILE" "$MMPROJ_BYTES"

# The Vulkan library is what tells a Vulkan install from the CPU one that used to
# be here, and it is why this is not just an "is llama-server present" check: a
# rover installed before the switch has a perfectly good llama-server that cannot
# see the GPU, and it should be replaced rather than kept.
if [ -n "$FORCE" ] || [ ! -x "$VENDOR/llama/llama-server" ] \
        || [ ! -f "$VENDOR/llama/libggml-vulkan.so" ]; then
    echo "--- fetching llama.cpp $LLAMA_BUILD (Vulkan)"
    tarball="$VENDOR/llama-$LLAMA_BUILD.tar.gz"
    curl -fL --retry 5 --retry-delay 5 -o "$tarball" \
        "https://github.com/ggml-org/llama.cpp/releases/download/$LLAMA_BUILD/llama-$LLAMA_BUILD-bin-ubuntu-vulkan-arm64.tar.gz"
    rm -rf "$VENDOR/llama"
    mkdir -p "$VENDOR/llama"
    # The release archive has a single top-level directory whose name has moved
    # between builds, so strip it rather than naming it.
    tar -xzf "$tarball" -C "$VENDOR/llama" --strip-components=1
    rm -f "$tarball"
fi

if [ ! -x "$VENDOR/llama/llama-server" ]; then
    echo "llama-server is not where it was expected: $VENDOR/llama" >&2
    exit 1
fi
if [ ! -f "$VENDOR/llama/libggml-vulkan.so" ]; then
    echo "this llama.cpp has no Vulkan backend: $VENDOR/llama" >&2
    exit 1
fi

# Said out loud because the failure it catches is silent: a board whose Vulkan
# driver is missing or broken still runs, just four times slower, and nothing
# else would ever mention it.
echo "--- what Vulkan can see"
LD_LIBRARY_PATH="$VENDOR/llama" "$VENDOR/llama/llama-server" --list-devices \
    || echo "llama.cpp could not list devices; it will fall back to the CPU" >&2

# The database and the frames, deliberately outside ~/ugv: a deploy replaces that
# tree, and the experiment's results are not source.
mkdir -p "$HOME/.ugv/world/frames"

LINE="@reboot $HERE/run_cosmos.sh"
current=$(crontab -l 2>/dev/null || true)
if printf '%s\n' "$current" | grep -q 'world_state/run_cosmos.sh'; then
    echo "crontab already starts the Cosmos sidecar"
else
    if [ -n "$current" ]; then
        printf '%s\n%s\n' "$current" "$LINE" | crontab -
    else
        printf '%s\n' "$LINE" | crontab -
    fi
    echo "added: $LINE"
fi
sync

echo "installed:"
ls -l "$VENDOR/$MODEL_FILE" "$VENDOR/$MMPROJ_FILE" "$VENDOR/llama/llama-server"
