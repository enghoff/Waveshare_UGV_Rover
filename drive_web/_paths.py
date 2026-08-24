"""Put this directory and voice_chat/ on sys.path.

On the rover everything lands in ~/ugv/drive_web/, including the modules
this console still shares with the voice stack -- rover_tools.py, console_model.py,
session.py -- so the voice_chat sibling is missing and unused. In the
repository those stay in voice_chat/, because the tests import them there.
"""
from __future__ import annotations

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
VOICE = os.path.normpath(os.path.join(HERE, "..", "voice_chat"))
# Wheels unpacked rather than installed, because this board has no pip -- see
# install_websockets.sh. Last on the path, so anything Debian does have wins.
VENDOR = os.path.join(HERE, "vendor")
for path in (HERE, VOICE):
    if os.path.isdir(path) and path not in sys.path:
        sys.path.insert(0, path)
if os.path.isdir(VENDOR) and VENDOR not in sys.path:
    sys.path.append(VENDOR)
