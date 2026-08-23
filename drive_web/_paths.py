"""Put this directory and voice_chat/ on sys.path.

On the rover everything lands in ~/ugv/drive_web/, including the two modules
this console still shares with the voice client -- rover_tools.py and
console_model.py -- so the voice_chat sibling is missing and unused. In the
repository those two stay in voice_chat/, because talk.py and its tests import
them there.
"""
from __future__ import annotations

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
VOICE = os.path.normpath(os.path.join(HERE, "..", "voice_chat"))
for path in (HERE, VOICE):
    if os.path.isdir(path) and path not in sys.path:
        sys.path.insert(0, path)
