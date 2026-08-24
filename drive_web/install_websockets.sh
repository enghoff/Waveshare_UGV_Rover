#!/bin/sh
# Put `websockets` where the console's microphone can find it.
#
#     ssh bpi-m4zero 'sh ~/ugv/drive_web/install_websockets.sh'
#     ssh bpi-m4zero 'sh ~/ugv/drive_web/install_websockets.sh --force'
#
# The rover holds its own conversation with Alibaba's realtime service now -- see
# [omni_bridge.py](omni_bridge.py) -- and the session that speaks the protocol is
# [session.py](../voice_chat/session.py), which needs this library. The board runs
# Debian's CPython 3.13 with **no pip and no python3-venv**, and `sudo` here wants
# a password no deploy script has, so this follows the same road as OpenCV and
# depthai before it: a wheel is a zip, and unpacking one next to the code that
# imports it needs no privileges at all.
#
# This one is easier than those two, because it is `py3-none-any` -- pure Python,
# no compiled speedups, one file for every interpreter and every architecture. The
# C extension it can optionally use is a masking routine, and masking a hundred
# audio frames a second on four Cortex-A53 cores is not the expensive part of
# anything here.
#
# Pinned by version and by hash for the reason install_opencv.sh is: this runs
# unattended from a deploy, over a wifi link that drops, and a truncated download
# unpacking into half a package reads as the rover having lost its voice rather
# than as a bad copy.
set -e

VERSION=15.0.1
WHEEL=websockets-$VERSION-py3-none-any.whl
SHA256=f7a866fbc1e97b5c617ee4116daaa09b722101d4a3c170c787450ba409f9736f
URL=https://files.pythonhosted.org/packages/fa/a8/5b41e0da817d64113292ab1f8247140aac61cbf6cfd085d6a0fa77f4984f/$WHEEL

DIR="$(cd "$(dirname "$0")" && pwd)"
VENDOR="$DIR/vendor"
FORCE=""
LOCAL=""
for arg in "$@"; do
    case "$arg" in
        --force) FORCE=1 ;;
        *) LOCAL="$arg" ;;
    esac
done

if [ -z "$FORCE" ] && PYTHONPATH="$VENDOR" python3 -c 'import websockets' 2>/dev/null; then
    have=$(PYTHONPATH="$VENDOR" python3 -c 'import websockets; print(websockets.__version__)')
    echo "websockets $have is already unpacked in $VENDOR"
    exit 0
fi

mkdir -p "$VENDOR"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

if [ -n "$LOCAL" ]; then
    cp "$LOCAL" "$TMP/$WHEEL"
else
    echo "fetching $WHEEL ..."
    curl -fsSL --retry 3 -o "$TMP/$WHEEL" "$URL"
fi

got="$(sha256sum "$TMP/$WHEEL" | cut -d' ' -f1)"
if [ "$got" != "$SHA256" ]; then
    echo "$WHEEL does not match the pinned hash." >&2
    echo "  expected $SHA256" >&2
    echo "  got      $got" >&2
    exit 1
fi

# `python3 -m zipfile` rather than unzip, which this board does not have.
python3 -m zipfile -e "$TMP/$WHEEL" "$VENDOR"

PYTHONPATH="$VENDOR" python3 -c 'import websockets; print("websockets", websockets.__version__, "ok")'
