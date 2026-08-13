#!/usr/bin/env bash
# Minimal depthai environment in the guest, pinned to 2.x to match the host-side
# tooling in oak_camera/ (see docs/depthai-version-pin.md).
set -euo pipefail

sudo apt-get update -qq
sudo apt-get install -y -qq libusb-1.0-0 udev

python3 -m venv "$HOME/venvs/oak"
"$HOME/venvs/oak/bin/pip" -q install --upgrade pip
"$HOME/venvs/oak/bin/pip" -q install 'depthai>=2.32,<3'
"$HOME/venvs/oak/bin/python" -c 'import depthai; print("depthai", depthai.__version__)'
