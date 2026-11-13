"""Whether a drive by hand has taken the rover off its map, and when it is back.

No ROS in here, so the selftest can run it on a workstation; `nav_manual.py`
feeds it every step `localizer_node.py` publishes on `localizer/step`.

**Why it exists.** Manual driving pauses the mapper and the world store and
gives them back when the controller lets go (nav_manual.py). Given back at
whatever pose the rover then believes -- and a person can drive it out of the
flat, onto a balcony or into a room the map has never seen. Out there the
localizer can place no scan on the map and keeps wheel odometry, which drifts;
and a controller that goes to sleep out there would hand the mapper a street to
draw onto the flat and the store a pose nobody can vouch for to file its looks
against.

**Leaving is noticed and latched.** LEAVE_STEPS searched scans running that the
localizer could not place on the map -- under LEAVE_SCORE, its own bar for
believing a match -- during a manual drive, is the rover off its map. Scans it
did not search -- turning too fast, too few returns -- say nothing either way.
**The score decides, not whether a correction was taken**: a parked rover's
searched scans come back "standing still and the map agrees", not accepted,
at 82-87% (on the rover, 2026-11-13), and counting those as misses latched a
rover that had only stopped. From then on the map is held, with the controller or
without it: the mapper paused, nothing saved, the world store recording
nothing, the change watch and autonomous runs refused (nav_manual.py says how).

**Coming back is shown, not assumed.** BACK_STEPS matched scans running at
BACK_SCORE or better -- the support a restore needs (nav_map.RESTORE_SUPPORT)
-- earn a check: the drift check's own wide search, 1.5 m and a full turn
around the tracked pose, which must find nowhere better (nav_map.check_drift's
test). Only that ends it. A rover that came back further out than the
localizer's 25 cm window cannot match well enough to earn the check; a refit
from the console puts it where the scan says, and the check follows. While the
rover is off its map a refit searches that same wide window, since a drive on
wheel odometry alone can come back more than a refit's usual metre out.
"""

#: Searched scans running that could not be placed, during a manual drive:
#: two seconds at the localizer's 5 Hz. A person walking past the lidar does
#: not hide the room for that long; a doorway into the unmapped does.
LEAVE_STEPS = 10
#: Under this share of the scan on the map's walls, a scan is not placed: the
#: localizer's own MIN_SCORE, under which it keeps odometry's prediction.
LEAVE_SCORE = 0.55
#: Matched scans running, at this share of the scan on the map's walls or
#: better, before the wide search is asked whether the rover is back.
BACK_STEPS = 10
BACK_SCORE = 0.75
#: Seconds between two wide searches (1.1 s of the Orin each, measured for the
#: drift check), so a rover that keeps nearly matching is not searched for
#: continuously.
CHECK_GAP_S = 10.0


class OffMap(object):
    """The latch. `step` is fed the localizer's steps; `checked` the verdicts."""

    def __init__(self):
        #: When the rover was found to have left (wall clock), or None.
        self.since = None
        #: The last pose matched on the map before it left: (x, y, heading deg).
        self.left_at = None
        self.misses = 0
        self.hits = 0
        self.checking = False
        self.checked_at = None
        self.checks = 0

    @property
    def off(self):
        return self.since is not None

    def step(self, manual, searched, score, pose, now, wall):
        """One localizer step. Says "left" when the rover has just been found
        off its map, "check" when it may be back and the wide search should be
        asked, and "" otherwise. `now` is monotonic, for the check gap; `wall`
        is the wall clock, which is what `since` is shown in."""
        if not searched:
            return ""
        if self.since is None:
            if score >= LEAVE_SCORE:
                self.misses = 0
                self.left_at = pose
                return ""
            if not manual:
                # Only a person's drive is held: every other drive maps as it
                # goes, so what it sees is the map rather than off it.
                self.misses = 0
                return ""
            self.misses += 1
            if self.misses < LEAVE_STEPS:
                return ""
            self.misses = self.hits = 0
            self.checked_at = None
            self.since = wall
            return "left"
        if score < BACK_SCORE:
            self.hits = 0
            return ""
        self.hits += 1
        if (self.hits < BACK_STEPS or self.checking
                or (self.checked_at is not None
                    and now - self.checked_at < CHECK_GAP_S)):
            return ""
        self.checking = True
        self.checked_at = now
        return "check"

    def checked(self, back):
        """The wide search's verdict on a "check": back on the map, or not."""
        self.checking = False
        self.checks += 1
        if back:
            self.clear()
        else:
            self.hits = 0

    def clear(self):
        self.since = None
        self.misses = self.hits = 0
        self.checking = False
        self.checked_at = None

    def as_dict(self):
        """For the bridge's status: None while the rover is on its map."""
        if self.since is None:
            return None
        where = self.left_at
        return {"since": round(self.since, 1),
                "left_at": None if where is None else {
                    "x_m": round(where[0], 3), "y_m": round(where[1], 3),
                    "heading_deg": round(where[2], 1)},
                "matched_running": self.hits, "checks": self.checks,
                "checking": self.checking}
