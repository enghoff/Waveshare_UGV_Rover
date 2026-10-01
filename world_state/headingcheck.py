"""Whether a look's heading can be believed, checked against the map and never by moving.

**A look's bearing is only as good as the heading it was measured from, and after
the rover turns on the spot that heading is wrong.** Measured against a tape on
2026-10-01: slam_toolbox over-counts every turn by about 7%, so two circles left
the believed heading 43 degrees out, and on the morning's drive the looks taken
while the rover was turning missed their targets by a median 4.7 degrees against
2.6 for looks taken still. One scan matched against the map in a narrow window
finds the truth to within 2 degrees. See
docs/progress/2026-10-01-heading-after-turning.md.

So a still look asks the navigator where one scan says the rover is
(`ros_navigator.measure`, a read-only search of about a tenth of a second). It
keeps the pose if the scan agrees, corrects it by what the scan found if the scan
disagrees with confidence, and gives no direction if the scan cannot say. A moving
look cannot be checked, because a scan and a pose taken in motion do not describe
the same instant. It keeps its direction only while the last still check found
the heading right and the rover has turned less than `TURNED_SINCE_CHECK_DEG`
since. Straight-line driving keeps its bearings, which is what was won on
2026-09-03 (`inspector.TURNED_WHILE_LOOKING_DEG`), and looks taken during turns
give none until the rover stands still again.

**Nothing here moves the rover, corrects the navigator or waits on a move.** An
attempt that did all three was reverted on 2026-10-01: a refit inside every move
held the wheels for fifteen seconds and dropped the console's next target. The
navigator does not need the heading put right, because it puts it right itself
once the rover drives and Nav2 replans. Only the direction stamped on a look does,
and this is that.
"""

#: A look counts as still under these, across the shutter bracket. The bracket's
#: own resolution is a centimetre and a tenth of a degree, and anything under a
#: degree of turning sits inside the 1.5 degrees a bearing is allowed.
STILL_M = 0.03
STILL_DEG = 1.0

#: Rotation since a check found the heading right, beyond which a moving look gets
#: no direction. At 7% of a turn, fifteen degrees is about one degree of heading.
TURNED_SINCE_CHECK_DEG = 15.0


def _wrap(angle_deg):
    return (angle_deg + 180.0) % 360.0 - 180.0


class HeadingCheck(object):
    """The last check, the turning since it, and the verdict on each look."""

    def __init__(self, measure):
        #: `() -> dict | None`: the navigator's narrow scan-to-map measurement,
        #: shaped like `refit.Fit.as_dict` plus `was`, the pose it measured from.
        self.measure = measure
        self.good = False
        self.turned_deg = 0.0
        self.last = None
        self._heading = None

    def saw(self, pose):
        """A pose the inspector read, so turning between looks is counted."""
        if not isinstance(pose, dict) or pose.get("heading_deg") is None:
            return
        heading = float(pose["heading_deg"])
        if self._heading is not None:
            self.turned_deg += abs(_wrap(heading - self._heading))
        self._heading = heading

    def judge(self, where, moved, turned):
        """`(pose, note)` for a look: the pose to take its bearings from, or None
        for no direction, and a phrase for the look's diagnostics line, or None.
        """
        if where is None:
            return None, None
        if moved > STILL_M or turned > STILL_DEG:
            if self.good and self.turned_deg < TURNED_SINCE_CHECK_DEG:
                return where, None
            return None, ("the rover had turned %.0f deg since its heading was "
                          "last checked against the map, and a moving look "
                          "cannot be checked" % (self.turned_deg,))
        try:
            fit = self.measure()
        except Exception as error:             # a look survives a missing check
            fit = {"trusted": False, "why": "%s: %s"
                   % (type(error).__name__, error)}
        fit = fit or {"trusted": False, "why": "nothing answered"}
        self.last = fit
        if not fit.get("trusted"):
            self.good = False
            return None, ("the heading could not be checked against the map: %s"
                          % (fit.get("why") or "no reason given"))
        checked = {"off_deg": fit.get("turned_deg"), "off_m": fit.get("moved_m"),
                   "score": fit.get("score")}
        if fit.get("settled"):
            self.good, self.turned_deg = True, 0.0
            return dict(where, checked=checked), None
        # The scan places the rover somewhere a little different, confidently.
        # The look takes its bearings from there. The navigator is left to correct
        # itself, which it does once it drives, so moving looks wait for a check
        # that finds the heading right.
        self.good = False
        was = fit.get("was") or {}
        try:
            fixed = dict(
                where,
                x_m=round(where["x_m"] + fit["x_m"] - was["x_m"], 3),
                y_m=round(where["y_m"] + fit["y_m"] - was["y_m"], 3),
                heading_deg=round(_wrap(where["heading_deg"]
                                        + fit["heading_deg"]
                                        - was["heading_deg"]), 1),
                checked=dict(checked, corrected=True))
        except (KeyError, TypeError, ValueError):
            return None, "the map check answered without a pose to correct from"
        return fixed, ("heading corrected by %.1f deg against the map"
                       % (fit.get("turned_deg") or 0.0,))
