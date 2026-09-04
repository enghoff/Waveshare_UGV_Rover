#!/usr/bin/env python3
"""Where the rover has been, and when a pose is worth writing down.

The track drawn on the console's map is this: the rover's position in the map
frame, sampled twice a second and thinned to one point every five centimetres.
Kept here rather than in the bridge because the interesting part is not the
sampling -- it is knowing that a pose belongs to the map frame the rest of the
track is drawn in, which is a rule with a fault behind it and no ROS in it, so
the selftest argues with this file rather than with a second copy of it.

**The fault, as the rover had it on 2026-09-04.** The map was cleared and the
track came back with a straight 5.37 m line at its head, from (2.267, -18.303) to
(4.186, -23.318), running out of the mapped room and across open grey. The rover
had not moved a centimetre: every one of the other 418 steps in that track was
0.34 m or less, which is a half-second of driving, and there were no points at
all in between. What moved was the coordinates.

**Why clearing the map moves the rover without driving it.** `map -> odom` is the
pose graph's correction on top of dead reckoning, and clearing throws the graph
away: the new graph is anchored on raw odometry, so the correction that had built
up over the session is discarded in one step and the rover's coordinates change by
exactly that much. It was 5.37 m that afternoon and 11.4 m and 35 degrees a few
hours later. It is not a small effect and it does not shrink: over the 46 clears
in the rover's own log the rover stood anywhere from 0 to 23.7 m from the origin
of its own new map, because that origin is odometry's -- where the ROS stack was
last started -- and not where the rover was standing when the map was cleared.

**And why the old coordinates go on being published for a while.** slam_toolbox
only folds a scan into the graph once the rover has travelled
`minimum_travel_distance`, so a parked rover re-anchors nothing: measured on the
rover, `map -> odom` was bit-for-bit identical over 35 seconds of standing still.
The correction lands on the first scan after the wheels turn. Between the reset
returning and that scan -- which is at least one 0.5 s sample, and is however long
the rover is left standing -- every pose read out of the transform tree is still
in the frame that has just been thrown away.

So the trail waits. `cleared` says which correction is being discarded, and
nothing is written down until the mapper has published a different one, which is
the moment the new graph exists. Waiting costs nothing, because `clear_map` is
refused while the rover is moving and the rover therefore has nowhere to have
been in the meantime.
"""

import math

#: How far the rover moves between kept points. Five centimetres is the map's own
#: resolution, so a finer track could not be drawn on it anyway.
STEP_M = 0.05

#: How many points are kept: 4000 of them at 5 cm is 200 m of pottering about.
MOST = 4000

#: How much `map -> odom` has to change before it counts as a different
#: correction rather than the same one republished. A centimetre and half a
#: degree are both far below anything the graph moves by when it re-anchors --
#: that is metres and tens of degrees -- and far above the float noise in a
#: transform that is being recomputed and restamped twenty times a second.
MOVED_M = 0.01
MOVED_RAD = math.radians(0.5)

#: How far the rover may drive with the correction still unchanged before the
#: trail gives up waiting and records anyway. slam_toolbox re-anchors on the first
#: scan after `minimum_travel_distance`, which is 0.2 m, so a rover that has
#: driven a metre without the correction moving is a mapper that is not going to
#: move it -- and a track with a false step in it beats no track at all.
GIVE_UP_M = 1.0


def _turned(a, b):
    """The smaller way round between two headings, in radians."""
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


class Trail:
    """The points, and the one rule about when they may be added to.

    `correction` throughout is `map -> odom` as `(x, y, yaw)`, and `odom` is the
    rover's dead-reckoned pose in the same shape. Both may be None, which is what
    a transform tree that has not said anything yet looks like.
    """

    def __init__(self):
        self.points = []
        # The correction being discarded, and where the rover stood in odom when
        # it was, or None when nothing is being waited for.
        self._hold = None

    def __len__(self):
        return len(self.points)

    def cleared(self, correction, odom):
        """The map has been thrown away: start again in the frame that replaces it.

        A correction that was already nothing is not worth waiting on: there is
        no jump to avoid, and the new graph will publish nothing too, so waiting
        for a change would be waiting for something that is not coming.
        """
        self.points = []
        self._hold = None
        if correction is None or not self._changed(correction, (0.0, 0.0, 0.0)):
            return
        self._hold = (correction, odom)

    def offer(self, where, correction=None, odom=None):
        """One pose, half a second after the last. True if it was written down."""
        if not self.anchored(correction, odom):
            return False
        if self.points:
            last = self.points[-1]
            if math.hypot(where[0] - last[0], where[1] - last[1]) < STEP_M:
                return False
        self.points.append((round(where[0], 3), round(where[1], 3)))
        if len(self.points) > MOST:
            del self.points[:len(self.points) - MOST]
        return True

    def anchored(self, correction, odom):
        """Is a pose read now in the map frame this track is drawn in?

        True whenever nothing is being waited for, which is every sample except
        the handful between a clear and the mapper's first scan of the new graph.
        """
        if self._hold is None:
            return True
        was, odom_was = self._hold
        if correction is not None and self._changed(correction, was):
            self._hold = None
            return True
        if (odom is not None and odom_was is not None
                and math.hypot(odom[0] - odom_was[0],
                               odom[1] - odom_was[1]) >= GIVE_UP_M):
            self._hold = None
            return True
        return False

    @staticmethod
    def _changed(now, was):
        return (math.hypot(now[0] - was[0], now[1] - was[1]) > MOVED_M
                or _turned(now[2], was[2]) > MOVED_RAD)
