#!/bin/sh
# Put depthai where depth_server.py can find it, and check the udev rule.
#
#     ssh orin '~/ugv/oak_depth/install.sh'                  # fetch it from PyPI
#     ssh orin '~/ugv/oak_depth/install.sh /tmp/depthai.whl' # or from a local wheel
#     ssh orin '~/ugv/oak_depth/install.sh --force'          # unpack again over the top
#
# **The version here is the camera's firmware version.** The Myriad X has no flash
# and the host uploads firmware out of this wheel on every open, so choosing the
# wheel is choosing the firmware. 2.32.0.0 is pinned because 3.x kills this
# camera's left mono sensor and therefore its stereo depth -- measured twice, on
# two hosts, see README.md and docs/depthai-version-pin.md. Do not move it without
# re-running selftest.py and reading both.
#
# Unpacked rather than installed for the same reason OpenCV is beside yunet.py:
# the rover's Debian has no pip and no python3-venv, and `sudo` there wants a
# password no deploy script has. A wheel is a zip, numpy is already present, and
# that is the whole dependency list. The Jetson has pip and could install this
# properly, but unpacking works on both and one code path is worth more than the
# convenience of a second.
#
# Pinned by hash as well as version. This runs unattended from a deploy and the
# file arrives over a wifi link that drops; a truncated download would unpack into
# a half-populated package and read as the camera having failed.
set -e

VERSION=2.32.0.0

# One wheel per interpreter, unlike OpenCV's abi3 one, so the file is chosen by
# whichever python3 is on this host rather than pinned to a single board: the
# Banana Pi runs Debian's CPython 3.13 and the Jetson runs Ubuntu 24.04's 3.12.
# Only the interpreter tag differs between these two -- the *version* is the
# camera's firmware and must not move, because 3.x kills this camera's left mono
# sensor and therefore its stereo depth.
PYTAG=$(python3 -c 'import sys; print("cp%d%d" % sys.version_info[:2])')
case "$PYTAG" in
    cp312)
        SHA256=b3192ffff904482254def4cd2b9aac0c4d082a0787303bdc980768da4368331c
        URLDIR=6a/ab/73a17bfd2ed5686350ad38012edc22e16d817ecf8bd1dd6aff200777dfc5
        ;;
    cp313)
        SHA256=13b1fc97cbbdd89557a99461287618df388a27d521b7cfb8d0b81636b8b3c437
        URLDIR=7d/45/3ff68807991dd1c18077d8c04e3c99137e7556be8d23c3688e52802e0ca1
        ;;
    *)
        echo "no depthai $VERSION wheel is pinned for $PYTAG. PyPI publishes cp39" >&2
        echo "through cp314 for this release: add that filename's sha256 and its" >&2
        echo "package path to the case above, or pass a wheel as an argument." >&2
        exit 1
        ;;
esac
WHEEL=depthai-$VERSION-$PYTAG-$PYTAG-manylinux_2_28_aarch64.whl
URL=https://files.pythonhosted.org/packages/$URLDIR/$WHEEL

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

if [ -z "$FORCE" ] && PYTHONPATH="$VENDOR" python3 -c 'import depthai' 2>/dev/null; then
    echo "depthai $(PYTHONPATH="$VENDOR" python3 -c 'import depthai; print(depthai.__version__)') is already at $VENDOR"
    exit 0
fi

if [ -n "$LOCAL" ]; then
    ARCHIVE="$LOCAL"
    KEEP=1
else
    ARCHIVE="$(mktemp -d)/$WHEEL"
    KEEP=""
    echo "fetching $WHEEL"
    python3 - "$URL" "$ARCHIVE" <<'PY'
import shutil, sys, urllib.request
with urllib.request.urlopen(sys.argv[1], timeout=180) as source, \
        open(sys.argv[2], "wb") as target:
    shutil.copyfileobj(source, target)
PY
fi

python3 - "$ARCHIVE" "$SHA256" "$VENDOR" <<'PY'
import hashlib, pathlib, sys, zipfile

archive, expected, vendor = sys.argv[1], sys.argv[2], pathlib.Path(sys.argv[3])
digest = hashlib.sha256(pathlib.Path(archive).read_bytes()).hexdigest()
if digest != expected:
    raise SystemExit(f"{archive} is not the pinned wheel: sha256 {digest}")
vendor.mkdir(parents=True, exist_ok=True)
zipfile.ZipFile(archive).extractall(vendor)
print(f"unpacked into {vendor}")
PY

[ -n "$KEEP" ] || rm -rf "$(dirname "$ARCHIVE")"

PYTHONPATH="$VENDOR" python3 -c "import depthai; print('depthai', depthai.__version__)"

# The udev rule is the thing that will catch you. /dev/bus/usb/* is root:root at
# 0664, so libusb cannot open the camera as `admin` and every call fails with
# LIBUSB_ERROR_ACCESS -- which from the library's side is indistinguishable from
# the camera not being plugged in. Intel's own rule grants group `users`, which
# `admin` is in, and it has to cover both product IDs: the device changes its ID
# when it boots, so a rule for the unbooted state alone grants access to upload
# the firmware and then loses it.
if [ -f /etc/udev/rules.d/97-myriad-usbboot.rules ]; then
    echo "udev rule for 03e7 is installed"
else
    echo "MISSING: /etc/udev/rules.d/97-myriad-usbboot.rules -- as root, run:" >&2
    echo "    cp $DIR/97-myriad-usbboot.rules /etc/udev/rules.d/" >&2
    echo "    udevadm control --reload && udevadm trigger" >&2
    exit 1
fi
