"""Whether the rover's place on the map has been checked since anything put it in doubt.

**Turning on the spot leaves the rover wrong about which way it faces, and it does
not know.** Measured on 2026-10-01 against a tape, with the rover only turning:
slam_toolbox over-counts every turn by about 7% in both directions, so two
circles left the believed heading 43 degrees out, and a look taken then puts its
bearing that far from the thing it saw. The same afternoon a scan-to-map refit
took three such headings from 22-24 degrees out to within 2 of the tape. See
docs/progress/2026-10-01-heading-after-turning.md.

So the remedy is a check, not a calibration: after the rover has turned more than
a few degrees, match one scan against the map in a narrow window, move the rover
onto it, and only then let the world state take a bearing from the pose again.
The window is what makes it safe to do unasked. A search half a metre and 45
degrees wide cannot carry the rover into another room, and `refit.fit` refuses
whenever the room fits two ways. This is a different act from the console's
"refit to map", which searches wide because a person says the rover was moved.

The same flag carries the drift check's verdict. A rover lifted and carried turns
no wheel, so nothing here sees it move. The drift check (`nav_map.check_drift`,
every five minutes) is what notices: on 2026-10-01 it found a carried rover
"nowhere near here" ninety seconds after the rover had recorded six bearings from
where it used to stand (R-WS-16). Until a check confirms the pose again, the
world state takes no bearings.

This module is the bookkeeping and nothing else, so the selftest can argue with
it without ROS: `NavMap` feeds it odometry, asks it whether a check is due, runs
the search, and tells it how the search went.
"""
import math
import threading

#: Rotation since the last good check that makes the pose unchecked. At 7% of
#: every turn, fifteen degrees is about one degree of heading error, inside the
#: 1.5 degrees a bearing is allowed (`world_state/locate.py`, BEARING_SIGMA_DEG).
#: Counted from odometry, which under-counts a turn by about 12%, so the true
#: rotation that trips it is nearer seventeen; still inside the allowance.
CHECK_AFTER_DEG = 15.0

#: The search a check makes, around where the rover thinks it is. Two half-turns
#: one way left the heading 24 degrees out on 2026-10-01, so 45 leaves room for a
#: move that turned a full circle before anything looked.
WINDOW_M = 0.5
WINDOW_DEG = 45.0

#: How long the rover must stand still before the keeper checks on its own. A
#: move checks as it finishes; this is for driving by hand from the console,
#: which ends with no move to end.
STILL_S = 1.5

#: Spacing between attempts after a check was refused, so a rover nothing can
#: place -- carried to another room, say -- is not searched every second for ever.
#: Any new motion clears it, because a rover that moved is worth asking again.
RETRY_S = 30.0

#: Where the rover thinks it is, a drift check whose scan lies on the walls less
#: than this says it is not there. The parked rover read 0.905-0.945 on
#: 2026-10-01 and the carried one 0.259. A room symmetric enough for the search
#: to refuse still scores well here, and must not count as doubt.
LOST_HERE_SCORE = 0.80

#: One odometry sample counts as motion above these.
MOVED_M = 0.01
MOVED_DEG = 0.3


def _wrap(angle_deg):
    return (angle_deg + 180.0) % 360.0 - 180.0


class PoseWatch(object):
    """The turning since the last good check, and any other reason for doubt."""

    def __init__(self):
        # Fed from the odometry thread, read and reset from the keeper's, the
        # bridge's and whoever asks for status.
        self._lock = threading.Lock()
        self.turned_deg = 0.0
        self.doubt = None
        self.last = None
        self._prev = None
        self._moved_at = None
        self._failed_at = None

    def feed(self, odom, now):
        """One odometry sample, `(x_m, y_m, heading_deg)` in the odom frame.

        Absolute rotation is summed rather than net, because a turn and its undo
        both put error in: on 2026-10-01 the two directions were not equal, and a
        rover that turned a circle one way and back the other was still 6 degrees
        out. A refit moves `map -> odom` and leaves odometry alone, so correcting
        the rover never reads as turning it.
        """
        with self._lock:
            if odom is None:
                return
            if self._prev is not None:
                turned = abs(_wrap(odom[2] - self._prev[2]))
                moved = math.hypot(odom[0] - self._prev[0], odom[1] - self._prev[1])
                self.turned_deg += turned
                if turned > MOVED_DEG or moved > MOVED_M:
                    self._moved_at = now
                    self._failed_at = None
            else:
                self._moved_at = now
            self._prev = odom

    def checked(self):
        """Whether a bearing may be taken from the pose now."""
        return self.turned_deg < CHECK_AFTER_DEG and self.doubt is None

    def due(self, now):
        """Whether the keeper should check now, of its own accord.

        Only when something is in doubt, the rover has stood still for
        `STILL_S`, and a refused check is not being retried too soon.
        """
        with self._lock:
            if self.checked():
                return False
            if self._moved_at is not None and now - self._moved_at < STILL_S:
                return False
            if self._failed_at is not None and now - self._failed_at < RETRY_S:
                return False
            return True

    def drifted(self, drift):
        """What the five-minute drift check found, as `nav_map` reports it.

        Doubt when the scan confidently places the rover somewhere else, or when
        it lies badly on the walls where the rover thinks it is. "Cannot say" in
        a room that fits two ways, with the scan sitting well where the rover is,
        is not doubt. That is the parked rover on 2026-09-07, 94% on the walls.
        """
        with self._lock:
            if not drift:
                return
            wrong = drift.get("trusted") and not drift.get("agrees")
            lost = (not drift.get("trusted")
                    and (drift.get("here_score") or 0.0) < LOST_HERE_SCORE)
            if wrong or lost:
                self.doubt = drift.get("why") or "the drift check disagreed"

    def outcome(self, now, fitted, verified):
        """How a check went, from the fit that may have moved the rover and a
        second look that measures where it ended up.

        Confirmed only when that second look is trusted and finds the rover
        settled where it now thinks it is. The fit's own report is not enough:
        when it is handed a correction, the mapper matches the next scan against
        its own recent nodes and can keep the rover where it was, which the
        console's refit reported on 2026-10-01 as a two-degree fit not applied.
        """
        with self._lock:
            fitted = fitted or {}
            verified = verified or {}
            confirmed = bool(verified.get("trusted") and verified.get("settled"))
            self.last = {
                "confirmed": confirmed,
                "at": round(now, 1),
                "turned_deg": round(self.turned_deg, 1),
                "found_deg": fitted.get("turned_deg"),
                "found_m": fitted.get("moved_m"),
                "moved": bool(fitted.get("fitted")),
                "left_deg": verified.get("turned_deg"),
                "left_m": verified.get("moved_m"),
                "why": (verified.get("why") if not confirmed
                        else fitted.get("why") or verified.get("why")),
            }
            if confirmed:
                self.turned_deg = 0.0
                self.doubt = None
                self._failed_at = None
            else:
                self._failed_at = now
            return self.last

    def refused(self, now, why):
        """A check that could not run at all -- no map, a pose nobody settled."""
        with self._lock:
            self._failed_at = now
            self.last = {"confirmed": False, "at": round(now, 1),
                         "turned_deg": round(self.turned_deg, 1), "why": why}
            return self.last

    def status(self):
        with self._lock:
            return {"pose_checked": self.checked(),
                    "turned_since_check_deg": round(self.turned_deg, 1),
                    "pose_doubt": self.doubt,
                    "pose_check": self.last}
