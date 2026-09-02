"""The fakes the daemon's offline checks share, and the paths they need.

`FakeLink` is a driver board that answers, or does not, and remembers what it was
told -- the stand-in for the one piece of hardware every tool call goes through.

Importing this module also puts the daemon's own directory, `face_tracking/` and
`lidar_slam/` on `sys.path`, which is what lets a check `import rover_daemon`,
`aiming` or `mapimg` in the repository as well as on the rover. The rover deploys
this component flat into `~/ugv` and the map renderer into `~/ugv/lidar_slam`, so
both layouts have to work; see the comments below.
"""
from __future__ import annotations

import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
# In the repo, aiming.py lives one directory over; on the rover everything is
# deployed flat into ~/ugv and this does nothing.
SIBLING = os.path.join(os.path.dirname(HERE), "face_tracking")
if os.path.isdir(SIBLING):
    sys.path.insert(0, SIBLING)
# The same for the map renderer, and it needs both layouts rather than one:
# `lidar_slam` is a sibling of this file in the repository and a *subdirectory*
# of the rover's ~/ugv, which is the two-candidate dance ros_navigator.py does
# for the same import. One check draws a real map and reads the picture back,
# and with only the repository layout it passes here and fails on the rover.
for RENDERER in (os.path.join(os.path.dirname(HERE), "lidar_slam"),
                 os.path.join(HERE, "lidar_slam")):
    if os.path.isdir(RENDERER):
        sys.path.insert(0, RENDERER)
        break


class FakeLink:
    """A driver board that answers, or does not, and remembers what it was told."""

    def __init__(self, works=True, volts=1153):
        self.sent = []
        self.works = works
        # What its telemetry says the pack is at, in hundredths of a volt, and how
        # many times that has been asked for. None is a board that says nothing
        # back, which is what an unpowered one and the wrong serial port both look
        # like from here.
        self.volts = volts
        self.reads = 0

    def send(self, command):
        self.sent.append(command)
        return self.works

    def telemetry(self):
        self.reads += 1
        return None if self.volts is None else {"T": 1001, "v": self.volts}

    def describe(self):
        return "fake"

    def close(self):
        pass
