"""The driver board's protocol, the chassis as somebody last measured it, and
the two shapes a move reports itself in.

Everything else that used to be here went with the rover's own navigator, which
`ros_nav/` replaced. What is left is what both halves still need and neither owns:
`ros_nav/base_node.py` and `ros_nav/drive_mixer.py` read the board constants and
the turn curve, and `rover_daemon/ros_navigator.py` builds its replies out of
`Outcome` and `MoveReport` so that the daemon's tools answer in the same shape
they always did.

The file keeps its name, and the directory keeps its own, because a dozen deploy
scripts and `sys.path` fixups name them. Neither is a description of the contents
any more: there is no SLAM in `lidar_slam/` and no navigator behind `nav_types`.
"""
import collections
import threading
import time

# --- the driver board -------------------------------------------------------
CMD_PWM = 11               # CMD_PWM_INPUT: {"T":11,"L":..,"R":..}
CMD_HEARTBEAT = 136
MIN_PWM = 40               # below this the motors buzz and do not turn
TOP_PWM = 160
HEARTBEAT_MS = 500         # the board stops itself if it hears nothing for this long

# --- the chassis, as a fallback ---------------------------------------------
# These describe the *previous* rover. The live numbers come from
# `~/ugv/odometry.json`, which `ros_nav/calibrate_chassis.py` writes by driving
# this chassis on this floor; `drive_mixer.py` warns loudly when it has to fall
# back here, because a curve fitted to a different set of tracks is wrong in a way
# that looks like a control problem.
MAX_SPEED_MS = 0.35
# The ceiling the mixer clamps every rotation request to, and the same number as
# `max_vel_theta: 0.78` in `config/nav2.yaml` -- 45 deg/s is 0.785 rad/s, and
# `ros_nav/selftest.py` checks the two have not drifted apart. It was originally
# half of what the old scan matcher's search window could absorb in a revolution;
# that reason is gone with the matcher, and what keeps the number is that the
# rover tracks a path visibly worse above it.
MAX_TURN_DPS = 45.0
MIN_TURN_DPS = 12.0        # below this a turn stops being a move; take the risk

# Turn rate and coast against PWM, from timing fixed-PWM bursts against the gyro:
#
#     PWM 180 -> 170.0 deg/s, 9.0 deg of coast   (fits 0.105-0.119)
#     PWM  80 ->  31.6 deg/s, 2.0 deg of coast   (fits 0.054-0.080)
#
# Only the rate half is read now -- the coast mattered to open-loop bursts, and
# Nav2 closes the loop -- but the pair is kept because it is one measurement.
TURN_RATES = {180: (170.0, 9.0), 80: (31.6, 2.0)}


class Outcome:
    """Why a move ended. The model needs this more than it needs the pose: "I
    stopped after 40 cm because something was 32 cm ahead" is actionable, and
    "done" is not."""

    def __init__(self, reason, travelled, turned, detail=""):
        self.reason = reason
        self.travelled_m = travelled
        self.turned_deg = turned
        self.detail = detail

    def asdict(self):
        out = {"reason": self.reason,
               "travelled_m": round(self.travelled_m, 3),
               "turned_deg": round(self.turned_deg, 1)}
        if self.detail:
            out["detail"] = self.detail
        return out


class MoveReport:
    """A running commentary on the move that is happening now.

    A move here is one blocking call that can last a minute -- plan a route, drive
    a leg, lose the corridor, plan again, drive the rest -- and until this existed
    the only account of any of it was the `Outcome` that arrived once it was all
    over. Anything watching had a stopwatch and nothing else: a route the planner
    had refused outright looked exactly like a route still being driven, and a
    rover that had quietly replanned four times looked like one driving a long way
    slowly.

    So the move says what it is doing while it does it, and `status()` hands the
    sentence it is on to whoever is polling.

    `seq` is what makes that usable. The console asks three times a second, and
    without a counter it cannot tell a sentence it has not seen from the same one
    read again -- every line it wrote would land in its log thirty times over.

    Behind the current sentence is a short history, and that is not
    belt-and-braces: some phases are briefer than the poll. A replan lasts exactly
    as long as the planner takes, about 0.2 s on this Pi, and is then superseded by
    the route it produced -- so a watcher asking every 0.3 s could easily see the
    new route appear and never learn what provoked it, which is the one thing about
    a replan worth knowing. A caller that says which sentence it saw last gets the
    ones in between along with the current one. A caller that says nothing gets the
    current one alone, which is the right answer for a status line.
    """

    #: How many sentences to keep for a watcher that blinked. A move is over long
    #: before it could produce this many, so in practice nothing is ever lost; the
    #: bound is here so that a daemon nobody is watching cannot grow a list forever.
    HISTORY = 32

    def __init__(self):
        # Its own lock rather than the navigator's: that one is taken by the control
        # loop twenty times a second, and there is nothing here worth queueing
        # behind a PWM decision for.
        self._lock = threading.Lock()
        self._state = self._blank(0)
        self._at = time.monotonic()
        self._history = collections.deque(maxlen=self.HISTORY)

    @staticmethod
    def _blank(seq, kind=None, asked=None):
        return {"seq": seq, "phase": "idle", "kind": kind, "asked": asked,
                "why": "", "route_m": None, "waypoints": None, "replans": 0,
                "reason": None}

    def begin(self, kind, asked, phase):
        """A new move. Everything the last one said goes, except the counter."""
        with self._lock:
            self._history.append(self._state)
            self._state = self._blank(self._state["seq"] + 1, kind, asked)
            self._state["phase"] = phase
            self._at = time.monotonic()

    def say(self, phase, why="", **fields):
        """One turn in the move. `why` is cleared unless this phase gives a reason,
        because a reason left lying around from the previous phase is a lie about
        this one."""
        with self._lock:
            self._history.append(self._state)
            self._state = dict(self._state)
            self._state.update(fields)
            self._state["phase"] = phase
            self._state["why"] = why
            self._state["seq"] += 1
            self._at = time.monotonic()

    def finish(self, reason, why=""):
        self.say("ended", why, reason=reason)

    def snapshot(self, since_seq=None):
        """The sentence being said now, carrying an age rather than a clock reading
        -- this machine's monotonic clock means nothing on the machine asking.

        `since_seq` is the last sentence the caller saw. Anything said between then
        and now comes back under `missed`, oldest first, so a phase shorter than the
        gap between two polls is still accounted for. Left out, `missed` is empty
        and this is simply the latest -- which is what a caller who wants a status
        line rather than a narrative should ask for.
        """
        with self._lock:
            out = dict(self._state)
            out["age_s"] = round(time.monotonic() - self._at, 2)
            missed = []
            if since_seq is not None:
                missed = [dict(state) for state in self._history
                          if since_seq < state["seq"] < out["seq"]]
            out["missed"] = missed
        return out
