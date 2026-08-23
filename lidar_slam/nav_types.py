"""Tuning knobs, move commentary and the small helpers Navigator is built from.

Kept beside :mod:`navigator` rather than inside it so the drive controller
can stay a composition of mixins without the constants living in three places.
`MAX_GOTO_M` is read out of this file with `ast` by the mock rover.
"""
import collections
import functools
import glob
import math
import os
import threading
import time

import serial

LIDAR_BAUD = 230400
# A 10 Hz sensor that has said nothing for a second has missed ten revolutions, and
# whatever it last said is no longer a description of the room. Everything that
# could move the rover checks this first -- the alternative was observed and is
# worse than it sounds: the port vanished under a running daemon, the scan froze,
# and describe_surroundings went on confidently reporting the room as it had been.
LIDAR_STALE_S = 1.0
LIDAR_REOPEN_S = 2.0

# Getting a sensor that has stopped talking to start again, as a ladder: each rung
# is a bigger act than the one before it, and none of them is taken while the wheels
# are turning.
#
# The first is closing the port and opening it again, which fixes the case the by-id
# name was introduced for -- the adapter re-enumerated under a running daemon and
# left this holding a handle to a device that no longer exists. Six seconds is sixty
# missed revolutions, well past any hiccup.
LIDAR_SILENT_S = 6.0
# The second is resetting the USB device, and it is a different kind of act: when
# the branch the lidar is on fails to enumerate, there is no port to open and no
# amount of reopening will make one. Held back to half a minute because the reset
# that reaches a wedged port is a reset of the hub above it, and that takes the
# camera and the OAK down with it for a few seconds -- see usbreset.py. Half a
# minute of blindness is already far past anything recoverable by waiting.
LIDAR_RESET_AFTER_S = 30.0
# How long to leave it before trying that again, and how far that backs off. A lidar
# that is genuinely unplugged cannot be helped by any of this, and resetting the hub
# every minute for the rest of the afternoon would knock the camera out each time
# for nothing. So: a minute, then two, then four, up to a quarter of an hour, and
# back to the start the moment a revolution arrives.
LIDAR_RESET_COOLDOWN_S = 60.0
LIDAR_RESET_MAX_COOLDOWN_S = 900.0

# --- what "do not hit anything" means -------------------------------------------
STANDOFF_M = 0.30          # the rule: never closer than this to anything seen
# Decide earlier than that. One revolution is 100 ms of sweep, slam2d only completes
# it when the next one starts, and then the motors take time to stop -- measured
# end to end this chain is over 200 ms, which at 0.35 m/s is 7 cm, and spin-down
# adds more. Braking from the standoff itself would arrive late every time.
REACT_MARGIN_M = 0.15
CORRIDOR_MARGIN_M = 0.06   # each side of the rover's own width: the hard limit

# What the planner has to leave around an obstacle. This is a sideways gap, not
# the along-track brake: inflating by STANDOFF_M + REACT_MARGIN_M (0.45 m) asked
# for a 90 cm opening, and a pinch the chassis fits through -- 85 cm, live scan
# still 4 m clear down the middle -- was refused as "no clear route" with the
# rover sitting still. 0.25 m is half of that; the live corridor still enforces
# the 30 cm standoff and 15 cm reaction along the path it actually follows.
PLAN_INFLATE_M = 0.25
# A route allowed to touch that ring hugs it, and a hugged corner is passed at
# exactly the distance the follower brakes at -- an ordinary pose error turns a
# legal route into a stop. The first attempt therefore inflates by that brake
# distance. Only if there is no such route does planning fall back to
# PLAN_INFLATE_M: a corner is given room whenever there is room, and a pinch is
# still taken when there is not. A soft toll was not enough; two extra cells of
# path is a cheap price to scrape a corner when going around costs metres.
PLAN_PREFERRED_M = STANDOFF_M + REACT_MARGIN_M
# Soft extra beyond whichever keep-out was used, so even a fallback route
# prefers the middle of a gap to its edge.
PLAN_COMFORT_M = 0.55

# And the soft one. Steering scores itself on a corridor this much wider than the
# rover, so that a route with room to spare beats one that merely fits. Without it
# the rover has no reason to prefer open space until something is already inside its
# safety margin: a wall approached at a shallow angle scored exactly the same as
# clear floor right up to the moment it blocked, at which point the rover was too
# close to turn away and simply stopped, wedged. Steering keeps clear; the tight
# corridor above still decides what is actually safe.
COMFORT_MARGIN_M = 0.28
LOOKAHEAD_M = 2.5          # no point scoring clearance further off than this
DECEL_MS2 = 0.45           # what the tracks can actually do on a hard floor

# --- speeds ---------------------------------------------------------------------
MAX_SPEED_MS = 0.35
CRAWL_SPEED_MS = 0.12      # when something ahead is unknown rather than clear
# The scan matcher's coarse window spans +/-9 degrees a revolution, i.e. 90 deg/s,
# and past that a 100 ms sweep smears the scan across more heading change than the
# match can absorb -- and, worse, the reported rotation saturates at the edge of the
# window instead of failing visibly. Staying well under it is not politeness, it is
# what keeps the measurement honest while the rover turns.
MAX_TURN_DPS = 45.0
# ...but 45 only holds if a revolution really is 100 ms. Measured over five
# recorded journeys it is not: the loop delivers a *matched* revolution every
# 138 ms at the median and 236 ms at the ninetieth percentile, because the Pi is
# oversubscribed while driving and drops roughly four revolutions in ten. What
# the window cares about is degrees per match, not degrees per second, so 45
# deg/s is 6.2 degrees at the median gap and 10.6 at the ninetieth -- and the
# coarse pass only spans 9. Past that the true pose is outside the lattice, the
# winner can only land on its rim, and the pose steps sideways: in those
# recordings every single jump over 6 cm happened on a revolution flagged
# `match_edge`, and 166 of the 178 window overruns were rotation rather than
# travel.
#
# So the cap is computed per revolution from the window and the interval the
# loop is actually managing, rather than assumed once at 10 Hz.
#
# The fraction is half the window, which is not a new judgement: 45 deg/s *is*
# half of the 90 the window allows at 100 ms, so this reproduces the existing
# limit exactly when the loop is keeping up and only bites when it is not. The
# headroom is there because the rover turns a little further than it is asked to
# -- momentum, and a turn scale that is itself being learned -- so the window has
# to cover the overshoot as well as the command. Raising this is the lever if
# turns feel slow, but the cheaper one by far is the drop rate: at a true 10 Hz
# this allows 45 deg/s, and at the 6 Hz the recordings measured it allows 33.
TURN_WINDOW_USE = 0.5
MIN_TURN_DPS = 12.0        # below this a turn stops being a move; take the risk
MAX_MATCH_GAP_S = 0.5      # a stale interval means the loop has stopped, not slowed
# How fast the commanded rate may rise. Without a ramp the first command of a turn
# is full differential, and the rover is briefly past the window before the rate
# loop has seen anything at all -- which is the moment tracking is most likely to be
# lost, and it happens before any feedback exists to prevent it.
TURN_RAMP_DPS_PER_S = 60.0
TURN_TOLERANCE_DEG = 2.0

# --- turning open-loop -----------------------------------------------------------
# Turning is dead reckoned, not servoed. Closing the loop on the scan matcher held
# the rate down to what the matcher could follow -- 30 deg/s, so six seconds for a
# quarter turn -- and it depended on a sensor that browns out mid-turn anyway. Open
# loop is both far faster and immune to the dropout, at the price of needing these
# numbers measured rather than assumed.
#
# Measured by calibrate_turn.py --deadreckon, which times fixed-PWM bursts and reads
# the angle back off the lidar profile, so nothing here is inferred from the thing
# being characterised. Angle is rate * seconds + coast; bursts of three lengths
# separate the two.
#
#     PWM 180 -> 170.0 deg/s, 9.0 deg of coast   (fits 0.105-0.119)
#     PWM  80 ->  31.6 deg/s, 2.0 deg of coast   (fits 0.054-0.080)
#
# Re-measure after anything that changes the drag: a different floor, worn tracks, a
# flat battery. The signature of stale numbers is a consistent over- or under-shoot
# in the same direction on every turn.
TURN_RATES = {180: (170.0, 9.0), 80: (31.6, 2.0)}
TURN_FAST_PWM = 180
TURN_FINE_PWM = 80
# Below this the fast burst is mostly its own coast, so the fine PWM does the whole
# turn: 9 degrees of coast is a third of a 25 degree turn and none of a 90.
TURN_FAST_MIN_DEG = 30.0
# A burst is dead reckoned, so its error grows with its size -- a 90 that physically
# managed 42 was 48 degrees out. Cutting a long turn into bursts this size and
# measuring between them keeps each error inside what the recovery search can undo,
# and costs only the settling time of the extra bursts.
TURN_BURST_MAX_DEG = 60.0
TURN_MAX_BURSTS = 6        # 180 degrees plus corrections, and a floor under a loop
                           # that would otherwise depend on the turn converging
# The burst coasts after the PWM stops -- 9 degrees of it at the fast rate -- and a
# revolution taken during that coast is smeared across the rotation. Waiting this
# out before believing anything costs a third of a second and saves judging the
# re-seed on the one scan guaranteed to be worst.
TURN_SETTLE_S = 0.35
TURN_REFIND_S = 2.5        # how long to let the matcher confirm where the re-seed put it
# What the matcher has to say before a re-seeded heading is believed and the map is
# written again. Three separate questions, because the score answers only the first:
# a scan that has snapped onto the wrong-but-consistent alignment scores *high* --
# scoring high is why that pose won -- so the score alone cannot tell a fix from a
# confident mistake. See slam2d.h.
RESEED_MIN_SCORE = 0.35    # does the scan fit here at all; same number as
                           # slam2d min_write_score, and the C core is what
                           # actually refuses to write below it
RESEED_MAX_AMBIGUITY = 0.60  # was there an equally good answer somewhere else.
                           # 0.90 never fired: a 90-degree-out fit in a
                           # rectangle sits in the 0.7-0.9 band. A turn still
                           # comes back lost at this; mapping does not stay
                           # held -- it takes the winner once the pose holds
                           # still, because freezing the map in a rectangle
                           # is how a rover stops adding walls for good.
RESEED_CONFIRM_M = 0.05    # pose must stay put this close across the pair
RESEED_CONFIRM_DEG = 5.0   # of confirming revolutions, wrap-aware
REACQUIRE_GOOD_SCANS = 2   # consecutive agreeing matches before the map is trusted again
# Revolutions of ordinary tracking to try before falling back on the +/-60 degree
# sweep, when the pose was lost while driving rather than to a dead-reckoned turn.
#
# The sweep is wide in heading and deliberately *narrow* in translation -- +/-5 cm
# against the tracking window's +/-10 -- because it was sized for a rover standing
# still after a turn, where the error is tens of degrees and a centimetre or two of
# position. A rover doing 0.25 m/s covers 5 cm in one revolution, so asking for the
# sweep while driving lands the winner on the rim, which holds the map, which asks
# for another sweep. Over five recorded drives an ordinary match outran its
# translation window on 0% of revolutions and hit the rim on 11%; the sweep outran
# its own on 49% and hit the rim on 80%, and mapping was held for 39-67% of every
# move as a result. So: try the window that fits a driving rover first, and widen
# only once it has failed often enough to mean the rover really is tens of degrees
# out rather than a few.
WIDEN_AFTER_LOST = 4
SLAM_EVENTS = 20           # how much of the recent history of all this to keep

# --- the gyro, which is the only witness here that is not the scan matcher -------
# What counts as standing still, for the purpose of learning what the resting gyro
# reads. Loose rather than tight: these are the *matcher's* numbers, so they carry
# its own few-millimetre noise, and a bar below that noise would never be met and
# the witness would never get a threshold at all. Anything the rover does on
# purpose is an order of magnitude above both.
REST_MAX_DPS = 2.0
REST_MAX_MS = 0.02
# A drive that has swung further round than this was not a drive in a direction,
# and whatever the wheels did over it is not a measurement of anything. Generous,
# because what used to be the worry here -- that a curved drive measures the wheels
# against a chord -- is now handled properly by measuring against the path the
# matcher actually traced rather than the straight line between its ends. This
# chassis wanders 23 degrees over a metre and a half, so a tight bar here refused
# every drive there was.
CALIBRATE_MAX_DRIVE_TURN_DEG = 60.0

# Turning is never refused. It used to be, whenever something sat inside the
# chassis' circumscribed radius, and that was the wrong call: a rover that has got
# closer to something than its own turning circle can then neither drive nor turn,
# so the one refusal that was meant to protect it left it wedged with no move
# available at all. Rotating is how it gets out. What proximity buys now is caution
# rather than a veto -- inside this radius the whole turn runs at the fine PWM,
# and the outcome says so.
TURN_CAREFUL_M = 0.24

# --- PWM ------------------------------------------------------------------------
CMD_PWM = 11               # CMD_PWM_INPUT: {"T":11,"L":..,"R":..}
CMD_HEARTBEAT = 136
MIN_PWM = 40               # below this the motors buzz and do not turn
TOP_PWM = 160
HEARTBEAT_MS = 500         # the board stops itself if it hears nothing for this long
KEEPALIVE_S = HEARTBEAT_MS / 3000.0
STOP_REPEATS = 3           # a dropped stop is the one packet that matters

# --- limits on a single request --------------------------------------------------
# The voice service gives a tool 12 s, all in. A bounded move has to finish inside
# that or the model is told nothing at all, which is worse than a short move.
MAX_MOVE_S = 8.0
# A route to a tap is not one 8 s hop: it is a handful of segments and the turns
# between them, and it is allowed to take the time that actually takes. The voice
# service will still cut a spoken call short; the console waits.
#
# The two numbers belong together and the distance is the one that moved. It was
# 8 m, which was the old grid's honest limit -- 10 m of reach from where the rover
# started, less the standoff -- and on a 40 m grid it was refusing places the map
# can show: a tap 11.2 m away came back "capped at 8 m" with the room it pointed at
# plainly drawn. 15 m is a route across a floor of a house rather than across a
# room, and still well inside the 20 m the grid reaches in any direction.
#
# The time follows from it. 15 m at the default 0.22 m/s is 68 s before a single
# corner is turned, and a route is a polyline rather than a straight line, so this
# is the same allowance the per-leg limit below makes -- a little over twice what
# the distance would take, which at the far end of the cap is a little over three
# minutes. It is a backstop and not the thing that gives up: a move that has gone
# wrong is ended by that per-leg limit, or by MAX_REPLANS, long before this.
MAX_GOTO_S = 200.0
MAX_GOTO_M = 15.0
MAX_REPLANS = 8
GOTO_ARRIVE_M = 0.16       # close enough; the pose is not a millimetre thing
GOTO_CORRIDOR_M = 0.55     # off the polyline by more than this means replan
GOTO_LOOKAHEAD_M = 0.80    # carrot along the current segment, not past the corner
# ...but never at a point nearer than this, at a corner gentle enough to drive
# through. A carrot that has collapsed onto the vertex it is clamped to turns a
# centimetre of cross-track error into tens of degrees of heading error, and the
# rover stops and spins a hand's breadth short of a corner it was tracking
# cleanly. Past this the aim point runs on along the line of the leg being
# driven, which cannot bend towards the inside of the corner -- see
# planner.carrot_at, which also says why a corner past GOTO_TURN_DEG keeps the
# collapsing carrot instead.
#
# 0.30 m is the standoff, and it is enough: the tracking error to steer out is a
# few centimetres of pose wobble on top of maybe 10 cm of drift off the line, and
# 10 cm at 30 cm is 18 degrees -- well inside GOTO_TURN_DEG, where the same error
# at 5 cm is 63 and stops the rover dead. Simulated over 253 trips, 0.25, 0.30
# and 0.40 m were indistinguishable in heading swing and arrivals, so this is the
# smallest of them that does the job and the least departure from aiming at the
# vertex itself.
GOTO_CARROT_MIN_M = 0.30
GOTO_SLACK_M = 0.35        # progress may slide back this far without rewinding
GOTO_TURN_DEG = 40.0       # more heading error than this: stop and turn
GOTO_ALIGN_DEG = 12.0      # then drive once the nose is this close
GOTO_RECHECK_S = 1.5       # how often to notice the map along the route changing
# When the way on is blocked, turning is the only move that changes anything -- see
# _step_goto. This is how long to spend turning to look for room before admitting it
# has not worked, and what has to have changed before asking the planner again can
# come back with a different answer at all.
GOTO_UNSTICK_S = 2.5
GOTO_REPLAN_MIN_S = 1.0
GOTO_REPLAN_MIN_MOVE_M = 0.10
# ...or this much turned on the spot, which is the same question asked of the one
# move a cornered rover always still has. The planner reads the heading -- a nose
# pointing into the keep-out starts the route with a hop off it -- so a rover that
# has only turned is not asking the same question again. 20 degrees swings the
# 0.45 m the heading test looks along by 15 cm, three cells, which is enough for it
# to answer differently.
GOTO_REPLAN_MIN_TURN_DEG = 20.0
UNKNOWN_AHEAD_SECTORS = 3  # +/-30 degrees at 36 sectors
# How many recent revolutions the "is anything touching us" test looks back over.
# One is not enough: a thin or dark object near the sensor's 0.12 m floor comes and
# goes between scans, and testing only the newest let a turn start beside something
# 0.13 m away and run for nearly four seconds before a scan happened to see it
# again. Taking the closest thing seen in the last half second instead means a
# return only has to appear once to be believed.
NEAR_HISTORY = 5


def _pose_close(a, b, max_m=RESEED_CONFIRM_M, max_deg=RESEED_CONFIRM_DEG):
    """True if two (x, y, theta) poses agree closely enough to be the same answer.

    The confirming pair after a re-find has to pass this, not just both look
    healthy: two recovery sweeps of a rectangle can lock at +38 deg and -25 deg
    in successive revolutions, both scoring beautifully, and that is two answers
    that contradict each other rather than one pose to write from.
    """
    dx, dy = a[0] - b[0], a[1] - b[1]
    if dx * dx + dy * dy > max_m * max_m:
        return False
    dth = abs((math.degrees(a[2] - b[2]) + 180.0) % 360.0 - 180.0)
    return dth <= max_deg


def _why_lost(health):
    """The reason a re-seed was not believed, as something a person can act on.

    Worth spelling out rather than printing three numbers: the fixes are different.
    An answer against the rim of the window means the turn went further wrong than
    the search can undo, and the rover needs to be told where it is or the map
    cleared. A rival peak means the room genuinely looks the same two ways round,
    and driving somewhere less symmetric fixes it. A low score means the scan does
    not fit anywhere near here at all, which is usually something in the way.
    """
    if health.get("reason"):
        return health["reason"]
    if health.get("edge"):
        if health.get("recovery"):
            return ("the best fit was against the edge of even the wide search, so "
                    "the rover ended up further round than that search could reach")
        return ("the best fit was against the edge of the tracking window, so the "
                "rover moved further than one revolution's search covers")
    if health.get("ambiguity", 0.0) >= RESEED_MAX_AMBIGUITY:
        return (f"another heading fitted the room just as well "
                f"({health['ambiguity']:.2f} of the best), so the room looks the "
                f"same from two directions and the scan cannot say which is right")
    return (f"the scan does not fit the map anywhere near there "
            f"(best match {health.get('score', 0.0):.2f})")


def _clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def find_lidar(preferred=None):
    """The lidar's serial port, preferring a name that survives a replug.

    `/dev/ttyACM0` is not that name. This CH343 came back as `ttyACM1` after
    re-enumerating under a running daemon, which left it holding a dead handle and
    reporting a frozen scan as though it were current. The by-id symlink carries the
    adapter's serial number, so it names the same device whatever number the kernel
    hands out this time.
    """
    if preferred and os.path.exists(preferred):
        return preferred
    for pattern in ("/dev/serial/by-id/*1a86*", "/dev/serial/by-id/*10c4*",
                    "/dev/ttyACM*"):
        found = sorted(glob.glob(pattern))
        if found:
            return found[0]
    return None


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


def _one_move_at_a_time(method):
    """The wheels have one owner at a time.

    The daemon calls tools from whichever connection thread they arrived on, so
    without this a turn_in_place and a drive_to arriving together would interleave
    their PWM -- the burst turn sends directly, bypassing the goal that guards
    _run_goal, so the goal check alone does not cover it. Refusing the second
    caller with "busy" is the honest answer; queueing it would drive the rover
    somewhere the first caller has since made wrong.

    stop() and clear_estop() deliberately do not take this lock: a stop must
    always get through, most of all while a move holds the lock.
    """
    @functools.wraps(method)
    def wrapper(self, *args, **kwargs):
        if not self._move_mutex.acquire(blocking=False):
            return Outcome("busy", 0.0, 0.0, "a move is already running")
        try:
            return method(self, *args, **kwargs)
        finally:
            self._move_mutex.release()
    return wrapper
