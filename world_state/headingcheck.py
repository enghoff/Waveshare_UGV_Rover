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
disagrees with confidence, and gives no direction if the scan fits nowhere near
where the rover believes it is -- which is what a carried rover looks like
(R-WS-16). **A moving look is checked too, since 2026-10-02**: the navigator matches
the scan from where the rover was half way through its sweep rather than from the
pose read beside it, which is what used to make a moving scan meaningless
(`nav_map.scan_moment`). A sweep turned faster than 30 degrees a second is still
refused, and a moving search that fits nowhere is taken as the motion's fault rather
than evidence of a carried rover. Where a moving look gets no check of its own and
the last check is fresh --
under `TURNED_SINCE_CHECK_DEG` of turning since, and, when that check had to
correct the heading, under `TRAVELLED_SINCE_CHECK_M` of travel -- it takes the
same correction that check found.

**Otherwise a look takes the heading the navigator believes, as every look did
before 2026-10-01.** The first version withheld it instead, and on a driven run
that withheld nearly everything: the check only runs when no move is in progress,
so on 2026-10-02 171 of 216 looks went without a direction, 17 of 906 regions got
one and five things were placed. Measured against the walls they stand on, the
two corrected looks of that run put the painting and the wardrobe within 7 cm of
their wall, where the navigator's heading would have put them 7 and 18 cm off; so
a correction is worth taking whenever there is one. But an unchecked heading is
what every drive before 2026-10-01 was placed from -- a median 3.5 degrees where
the rover drove to its stop -- and a look without a direction places nothing at
all. Only a check that searched and fitted nowhere withholds, and it goes on
withholding until a check fits again. A check that never ran -- a move in
progress, a busy map keeper, a bridge that did not answer -- has found nothing
wrong, and changes nothing.

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

#: Rotation since a check, beyond which a moving look no longer takes what that
#: check found and goes back to the navigator's heading. At 7% of a turn, fifteen
#: degrees is about one degree of heading.
TURNED_SINCE_CHECK_DEG = 15.0

#: Travel since a check that had to correct the heading, beyond which a moving
#: look no longer takes the correction. Driving folds scans into the map and may
#: correct the heading by itself, which would leave the check's correction stale.
#: Half a metre covers the shuffle between looks at one standing place.
TRAVELLED_SINCE_CHECK_M = 0.5


def _wrap(angle_deg):
    return (angle_deg + 180.0) % 360.0 - 180.0


class HeadingCheck(object):
    """The last check, the turning since it, and the verdict on each look."""

    def __init__(self, measure):
        #: `(around_offset) -> dict | None`: the navigator's narrow scan-to-map
        #: measurement, shaped like `refit.Fit.as_dict` plus `was`, the pose it
        #: measured from. Handed the last correction found, so the search starts
        #: where the heading has drifted to rather than where the rover thinks.
        self.measure = measure
        #: The correction the last trusted check found, `(dx, dy, dheading)`, and
        #: zero when it found the heading right. None when nothing trusted is
        #: in hand: before any check, or after one that could not say.
        self.offset = None
        #: Why the last check that searched found no fit, or None. While it is
        #: set no look gets a direction, still or moving, until a check fits
        #: again: the carried rover of R-WS-16. A check that never ran neither
        #: sets nor clears it.
        self.misplaced = None
        self.turned_deg = 0.0
        self.travelled_m = 0.0
        self.last = None
        self._checked = {}
        self._heading = None
        self._xy = None

    def saw(self, pose):
        """A pose the inspector read, so turning between looks is counted."""
        if not isinstance(pose, dict) or pose.get("heading_deg") is None:
            return
        heading = float(pose["heading_deg"])
        if self._heading is not None:
            self.turned_deg += abs(_wrap(heading - self._heading))
        self._heading = heading
        if pose.get("x_m") is not None and pose.get("y_m") is not None:
            xy = (float(pose["x_m"]), float(pose["y_m"]))
            if self._xy is not None:
                self.travelled_m += ((xy[0] - self._xy[0]) ** 2
                                     + (xy[1] - self._xy[1]) ** 2) ** 0.5
            self._xy = xy

    def judge(self, where, moved, turned):
        """`(pose, note)` for a look: the pose to take its bearings from, or None
        for no direction, and a phrase for the look's diagnostics line, or None.
        """
        if where is None:
            return None, None
        moving = moved > STILL_M or turned > STILL_DEG
        try:
            fit = self.measure(self.offset)
        except Exception as error:             # a look survives a missing check
            fit = {"trusted": False, "why": "%s: %s"
                   % (type(error).__name__, error)}
        fit = fit or {"trusted": False, "why": "nothing answered"}
        self.last = fit
        if not fit.get("trusted"):
            if moving or "score" not in fit:
                # Nothing was searched, so nothing was found wrong -- a sweep
                # turned too fast, the keeper writing, a bridge that did not
                # answer; a search that ran always scores. Or the search ran on
                # the move, where a scan that fits nowhere says more about the
                # motion than about where the rover is.
                return self._unchecked(where)
            self.offset = None
            self.misplaced = fit.get("why") or "no reason given"
            return None, ("the heading could not be checked against the map: %s"
                          % self.misplaced)
        if fit.get("settled"):
            offset = (0.0, 0.0, 0.0)
        else:
            # The scan places the rover somewhere a little different,
            # confidently. The look takes its bearings from there, and the
            # navigator is left to correct itself, which it does once it drives.
            was = fit.get("was") or {}
            try:
                offset = (fit["x_m"] - was["x_m"], fit["y_m"] - was["y_m"],
                          _wrap(fit["heading_deg"] - was["heading_deg"]))
            except (KeyError, TypeError, ValueError):
                self.offset = None
                return None, ("the map check answered without a pose to "
                              "correct from")
        self.offset = offset
        self.misplaced = None
        self.turned_deg = self.travelled_m = 0.0
        self._checked = {"off_deg": fit.get("turned_deg"),
                         "off_m": fit.get("moved_m"), "score": fit.get("score")}
        if moving:
            self._checked["moving"] = True
        if offset == (0.0, 0.0, 0.0):
            return self._corrected(where), None
        return self._corrected(where), ("heading corrected by %.1f deg against "
                                        "the map" % (offset[2],))

    def _unchecked(self, where):
        """`(pose, note)` for a look no check of its own speaks for: none while
        the rover is misplaced, the last check's correction while that is fresh,
        and otherwise the heading the navigator believes."""
        if self.misplaced is not None:
            return None, ("the last check against the map could not place the "
                          "rover, and none has since: %s" % self.misplaced)
        if self._fresh():
            return self._corrected(where, moving=True), None
        return where, None

    def _fresh(self):
        """Whether the last trusted check still speaks for a look taken now."""
        if self.offset is None or self.turned_deg >= TURNED_SINCE_CHECK_DEG:
            return False
        corrected = self.offset != (0.0, 0.0, 0.0)
        return not corrected or self.travelled_m < TRAVELLED_SINCE_CHECK_M

    def _corrected(self, where, moving=False):
        """`where`, moved by the last check's correction, with the check beside it."""
        dx, dy, dh = self.offset
        checked = dict(self._checked)
        if dx or dy or dh:
            checked["corrected"] = True
        if moving:
            checked["from_earlier_check"] = True
        return dict(where,
                    x_m=round(where["x_m"] + dx, 3),
                    y_m=round(where["y_m"] + dy, 3),
                    heading_deg=round(_wrap(where["heading_deg"] + dh), 1),
                    checked=checked)
