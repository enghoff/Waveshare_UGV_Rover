#!/bin/sh
# Put OpenCV where yunet.py can find it, on a host that has no package for it.
#
#     ssh orin '~/ugv/install_opencv.sh'                 # fetch it from PyPI
#     ssh orin '~/ugv/install_opencv.sh /tmp/opencv.whl' # or from a wheel already here
#     ssh orin '~/ugv/install_opencv.sh --force'         # unpack again over the top
#
# The rover's board runs Debian's CPython 3.13 with **no pip and no python3-venv**,
# and `sudo` here wants a password no deploy script has, so neither
# `apt install python3-opencv` nor `pip install` is available. A wheel is a zip,
# though, and this one is `cp37-abi3` -- one build for every CPython from 3.7 on --
# so unpacking it beside yunet.py and letting that add the directory to sys.path
# gets the same library with none of the privileges. numpy is already here from
# Debian, which is the only dependency it has.
#
# Pinned by version *and* by hash. The hash is not ceremony: this runs unattended
# from a deploy, the file arrives over the network, and the failure it prevents --
# a truncated download unpacking into a half-populated cv2 -- reads as the rover
# having lost its face detector rather than as a bad copy.
set -e

VERSION=4.12.0.88
WHEEL=opencv_python_headless-$VERSION-cp37-abi3-manylinux2014_aarch64.manylinux_2_17_aarch64.whl
SHA256=aeb4b13ecb8b4a0beb2668ea07928160ea7c2cd2d9b5ef571bbee6bafe9cc8d0
URL=https://files.pythonhosted.org/packages/69/4e/116720df7f1f7f3b59abc608ca30fbec9d2b3ae810afe4e4d26483d9dfa0/$WHEEL

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

# Already unpacked and importable is the ordinary case on a redeploy: the .py
# files are copied every time and this is 90 MB that does not change.
if [ -z "$FORCE" ] && PYTHONPATH="$VENDOR" python3 -c 'import cv2' 2>/dev/null; then
    echo "OpenCV $(PYTHONPATH="$VENDOR" python3 -c 'import cv2; print(cv2.__version__)') is already at $VENDOR"
    exit 0
fi

if [ -n "$LOCAL" ]; then
    ARCHIVE="$LOCAL"
    KEEP=1
else
    ARCHIVE="$(mktemp -d)/$WHEEL"
    KEEP=""
    echo "fetching $WHEEL"
    # urllib rather than curl or wget: python3 is the one thing this script can be
    # sure of, since it is what the caller is being installed for.
    python3 - "$URL" "$ARCHIVE" <<'PY'
import shutil, sys, urllib.request
with urllib.request.urlopen(sys.argv[1], timeout=120) as source, \
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

# Prove it imports here rather than leaving that to the daemon's next restart,
# and prove the detector loads its model too -- an OpenCV that imports and a
# YuNet that runs are not the same claim.
PYTHONPATH="$VENDOR" python3 -c "import cv2; print('OpenCV', cv2.__version__, 'threads', cv2.getNumThreads())"
python3 -c "
import sys; sys.path.insert(0, '$DIR')
from yunet import LocalDetector
print(LocalDetector().describe())
"
