#!/usr/bin/env python3
"""Driving the rover with the lidar in the loop.

This owns the lidar port, the SLAM core and a 10 Hz control loop, and it turns a
request like "forward 1.5 m" into motor PWM that will not drive into anything. It
does not own the driver board: the caller passes in something with `.send(dict)`,
which on the Pi is the daemon's SerialLink, so there is still exactly one owner of
the UART.

Three things about this rover shape the whole design.

**There are no wheel encoders.** Driving is open-loop PWM in +/-255, equal PWM is
not equal speed, and below MIN_PWM the motors only buzz. So a speed in metres per
second is meaningless unless something measures it -- and the scan matcher does,
every 100 ms. SLAM is the encoder this rover does not have, which is why speed and
distance here close on the match and not on the motors.

**Obstacle avoidance reads the live scan, never the map.** The map drifts and holds
geometry that has since moved. The current revolution is 100 ms old and costs
nothing to consult, and it is still right when the pose estimate is not.

**The lidar sees one horizontal slice at its own height.** It cannot see a step, a
drop, a low sill or a table top. Thirty centimetres from a wall is safe; thirty
centimetres from a table edge is not, and no tuning here changes that.
"""
import collections
import functools
import glob
import math
import os
import threading
import time

import serial

import journey
import planner
import usbreset
from odometry import Odometry
from slam2d import Slam2D, default_config

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


class Navigator:
    """The lidar, the SLAM core and the control loop, as one owned thing."""

    def __init__(self, link, lidar_port=None, config=None,
                 on_drive_start=None, on_drive_end=None):
        self.link = link
        self.slam = Slam2D(config or default_config())
        #: The driver board's gyro and wheel counts, which until now nothing read.
        #: Two jobs: it centres the scan matcher's search window on where the rover
        #: thinks it went, and it is the one witness on this rover that is not the
        #: scan matcher and can therefore contradict it. See odometry.py -- the
        #: first of those waits on a scale factor, the second does not.
        self._odom = Odometry(link)
        #: The board's account of the interval the newest revolution covers, kept
        #: because _note_match and _on_scan both want it and only _loop is in a
        #: position to take it -- a span may only be consumed once.
        self._span = None
        #: Held across a whole drive, the way a turn's mark is, so the wheel counts
        #: can be measured against the distance the matcher says was covered.
        self._drive_mark = None
        #: Revolutions whose match was rejected outright -- the scan fitted nothing
        #: anywhere -- counted since the daemon started. A move that saw one is a
        #: move whose distance is fiction, which is the bar a wheel measurement has
        #: to clear. Deliberately not the same bar as the map's: a winner against
        #: the rim of the window is a pose a few centimetres short, not a lost one,
        #: and refusing to measure over one of those refuses every drive, since
        #: stopping is when the window is outrun.
        self._rejects = 0
        self._edges = 0
        self._drive_marked_at = None
        #: Distance the matcher says the rover has travelled along its own heading,
        #: accumulated a revolution at a time and never reset. This is what the
        #: wheel counts are measured against, and it has to be the path rather than
        #: the straight line between the ends of a move: the wheels roll every
        #: centimetre of a wander and a chord does not. Signed along the heading, so
        #: a revolution's worth of match noise cancels instead of accumulating --
        #: taking the absolute value of each step would rectify that noise into
        #: centimetres of travel that never happened.
        self._path_m = 0.0
        # Opened by the loop rather than here, and reopened whenever it goes away.
        # At boot this matters: the lidar enumerates 93 s after the kernel starts on
        # this Pi, long after cron has run the daemon, so constructing this used to
        # throw and the rover came up permanently without its driving tools.
        self._lidar_pref = lidar_port
        self.lidar = None
        self.lidar_path = None
        self._reopen_at = 0.0
        # Two clocks, because "the sensor is there" and "we know where we are" are
        # different questions and a dead-reckoned turn makes them disagree. The
        # matched clock stops during a burst because the map is suspended; the packet
        # clock keeps running, because the sensor is still spinning and still
        # reporting whether or not anything is being done with what it says.
        self._last_scan_at = None       # last revolution that matched and was mapped
        self._last_packet_at = None     # last revolution parsed at all
        #: Where the lidar's adapter sits on the USB bus, as a sysfs name, kept from
        #: the last time the port opened. Kept because it is the one thing that
        #: cannot be looked up once the device has gone: a reset then has to be
        #: aimed at the hub the device *was* under, and nothing on the bus still
        #: remembers that.
        self._lidar_usb = ""
        #: The recovery ladder's state: when the loop started caring, when the next
        #: reset is allowed, how long to wait after this one, how many have been
        #: issued, and what the last one said. The last two are reported in
        #: nav_status, because a rover that has reset its own lidar four times in an
        #: hour has a cable problem and nobody would otherwise find out.
        self._lidar_watch_from = None
        self._reset_at = 0.0
        self._reset_wait = LIDAR_RESET_COOLDOWN_S
        self._resets = 0
        self._reset_note = ""
        #: How far up the bus the next reset reaches. Counted rather than decided,
        #: because a reset can succeed and change nothing -- the ioctl returns fine
        #: against a device that is enumerated but dead -- and without this the
        #: recovery would spend all afternoon resetting the one thing that has
        #: already been shown not to answer. Nothing came back, so reach higher.
        self._reset_rung = 0

        #: Called with no arguments just before the wheels first move, and again
        #: once they have stopped. The daemon uses these to put face tracking down
        #: and pick it back up, because the camera and SLAM cannot both have the core.
        self.on_drive_start = on_drive_start
        self.on_drive_end = on_drive_end

        self._lock = threading.Lock()
        self._move_mutex = threading.Lock()   # see _one_move_at_a_time
        self._run = threading.Event()
        self._thread = None

        # The current request, under _lock.
        self._want_speed = 0.0      # m/s, forward only
        self._want_turn = 0.0       # deg/s, ccw positive
        self._goal = None           # dict for a bounded move, or None
        self._estop = False
        self._driving = False

        #: What the move currently running is doing, for anything polling status().
        #: See MoveReport -- a move is one call that lasts a minute, and this is the
        #: only account of it that arrives before it is over.
        self.report = MoveReport()

        # Telemetry for status and for the speed loop.
        self._measured_speed = 0.0
        self._measured_turn = 0.0
        self._pwm_scale = 1.0       # closes the loop on speed with no encoders
        self._turn_scale = 1.0      # and on turn rate, which was worse: nobody has
                                    # measured what PWM 133 does in degrees a second
        self._commanded_turn = 0.0  # after the ramp, for reporting and for the ramp
        self._trim = 0.0            # left/right imbalance, so straight is straight
        self._last_pose = None
        self._last_at = None
        #: Seconds between matched revolutions, smoothed. The turn cap is derived
        #: from it -- see MAX_TURN_DPS. Starts at the sensor's nominal period so
        #: the first move is not throttled by a figure nothing has measured yet.
        self._match_gap = 0.1
        # Heading that accumulates instead of wrapping. The pose's own heading is
        # kept in (-pi, pi], which is right for reporting and wrong for counting a
        # turn: overshoot 180 and the difference flips sign, so "10 degrees left to
        # go" reads as "350 degrees left to go" and the rover spins right round.
        self._heading_accum = 0.0
        self._last_sent = None
        self._last_send_at = 0.0
        self._clearance = None
        self._chosen_deg = 0.0
        self._scans = 0
        self._dropped = 0
        self._near_history = []
        self._suspend_slam = False
        # Mapping suspended while nothing has confirmed where the pose is. Distinct
        # from _suspend_slam, which stops the matching too: this one keeps matching
        # so the rover can find its way back, and only holds off writing.
        self._map_paused = False
        self._need_recovery = False  # search until the first healthy match, and
        self._wide_recovery = False  # whether that search is the +/-60 deg sweep
                                     # rather than the ordinary tracking window
        self._lost_run = 0           # consecutive unhealthy revolutions while paused
        self._confirm_pose = None    # first of the confirming pair, or None
        self._recovery_this_scan = False
        self._hold_confirm = False   # True while a burst is still coasting
        self._min_write_score = float(self.slam.config.min_write_score)
        self._good_run = 0           # consecutive agreeing matches while paused
        self._health = {}            # how the last match was won, for status
        self._events = []            # a short history of losing and regaining the pose
        self._trail = []
        #: The journey being recorded, or None. Armed by the presence of a
        #: directory rather than a flag -- see journey.py. Nothing in the control
        #: loop may depend on it, and nothing it does may reach the wheels.
        self._journey = None
        self._heartbeat_set = False

    # --- lifecycle ------------------------------------------------------------
    def start(self):
        if self._thread is not None:
            return
        self._run.set()
        self._thread = threading.Thread(target=self._loop, daemon=True,
                                        name="navigator")
        self._thread.start()

    def close(self):
        self._run.clear()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None
        self._halt()
        try:
            if self.lidar is not None:
                self.lidar.close()
        finally:
            self.slam.close()

    # --- commands -------------------------------------------------------------
    # Each of the three is a thin wrapper that opens and closes the commentary in
    # `report`, around a private body that is the move itself. Wrapping rather than
    # threading report calls through every `return` keeps the bodies readable: this
    # one has eight ways to end and the route has more, and each of them would
    # otherwise have to remember to have the last word.
    def _ended(self, outcome):
        """Close the commentary on a move, and hand the outcome back untouched."""
        self.report.finish(outcome.reason, outcome.detail)
        return outcome

    @_one_move_at_a_time
    def drive(self, distance_m=None, speed_ms=None, seconds=None):
        """Go forward until the distance is covered or something is in the way.
        Blocks until it is done and says why it stopped. Avoidance may steer
        around obstacles, and will say so.
        """
        asked = {"distance_m": distance_m, "speed_ms": speed_ms}
        self.report.begin("drive", asked, "driving")
        # Recorded like the other two. This one has no route at all -- it aims at
        # whatever is roomiest within 40 degrees of the nose, every revolution,
        # and that becomes the new nose -- so if a wandering trail came from here
        # then no amount of planner work was ever going to straighten it, and the
        # recording is the only thing that says which tool drew the line.
        self._journey = journey.Recorder.if_armed()
        if self._journey:
            self._journey.begin("drive", asked, self.slam.pose)
        self._drive_mark = self._odom.mark()
        self._drive_marked_at = (self._rejects, self._edges, self._path_m)
        try:
            outcome = self._ended(self._drive(distance_m, speed_ms, seconds))
        finally:
            recording, self._journey = self._journey, None
        # Every straight drive the rover makes is also a measurement of its wheels,
        # for the same reason every turn measures its gyro: the scan matcher is the
        # ruler neither of them has on board.
        self._calibrate_drive(outcome)
        if recording:
            recording.end(outcome.reason, outcome.detail)
        return outcome

    def _drive(self, distance_m, speed_ms, seconds):
        if self._estop:
            return Outcome("stopped", 0.0, 0.0,
                           "the emergency stop is latched; clear it first")
        speed = _clamp(float(speed_ms if speed_ms is not None else 0.22),
                       0.05, MAX_SPEED_MS)
        distance = None if distance_m is None else abs(float(distance_m))
        if distance is None and seconds is None:
            seconds = 2.0
        limit = min(MAX_MOVE_S, float(seconds) if seconds else MAX_MOVE_S)
        if distance is not None:
            # Plus a margin for getting up to speed; the distance still decides.
            limit = min(MAX_MOVE_S, distance / speed * 1.8 + 1.5)

        return self._run_goal({"kind": "drive", "distance": distance,
                               "speed": speed}, limit)

    @_one_move_at_a_time
    def drive_to(self, ahead_m, left_m, speed_ms=None):
        """Go to a place relative to where the rover is now, around obstacles.

        Plans on the occupancy grid, thins the route to a few waypoints, and
        follows that polyline by looking ahead along it rather than by chasing
        every cell. A large heading change is a turn on the spot; a small one is
        steered through while driving. If the live scan cannot proceed, or the
        rover has left the corridor, or the map along the remaining route has
        grown a wall, the route is thrown away and another is planned from here.

        This is the one move where the answer is worth having before the end of it.
        It reports through `report`: the plan being drawn, the route it came back
        with or the reason there is none, and every replan with what provoked it.
        """
        asked = {"ahead_m": round(float(ahead_m), 2),
                 "left_m": round(float(left_m), 2)}
        self.report.begin("drive_to", asked, "planning")
        self._journey = journey.Recorder.if_armed()
        if self._journey:
            self._journey.begin("drive_to", asked, self.slam.pose)
        try:
            outcome = self._ended(self._drive_to(ahead_m, left_m, speed_ms))
        finally:
            recording, self._journey = self._journey, None
        if recording:
            recording.end(outcome.reason, outcome.detail)
        return outcome

    def _drive_to(self, ahead_m, left_m, speed_ms):
        if self._estop:
            return Outcome("stopped", 0.0, 0.0,
                           "the emergency stop is latched; clear it first")
        ahead_m, left_m = float(ahead_m), float(left_m)
        range_m = math.hypot(ahead_m, left_m)
        if range_m < 0.08:
            return Outcome("arrived", 0.0, 0.0, "already there")
        if range_m > MAX_GOTO_M:
            return Outcome("blocked", 0.0, 0.0,
                           f"that is {range_m:.1f} m away and a single route is "
                           f"capped at {MAX_GOTO_M:.0f} m")
        why = self._preflight("goto")
        if why:
            return Outcome("blocked", 0.0, 0.0, why)

        speed = _clamp(float(speed_ms if speed_ms is not None else 0.22),
                       0.05, MAX_SPEED_MS)
        x, y, th = self.slam.pose
        target = (x + ahead_m * math.cos(th) - left_m * math.sin(th),
                  y + ahead_m * math.sin(th) + left_m * math.cos(th))
        replans = 0
        path, last_why = self._plan_route(target)
        if not path:
            return Outcome("blocked", 0.0, 0.0, last_why)
        self._say_route(path, replans)

        started = time.monotonic()
        travelled_before = 0.0
        turned_before = 0.0

        self._begin_driving()
        try:
            while time.monotonic() - started < MAX_GOTO_S:
                if self._estop:
                    return Outcome("stopped", travelled_before, turned_before,
                                   "the emergency stop was latched")
                if path is None:
                    path, last_why = self._plan_route(target)
                    if not path:
                        return Outcome("blocked", travelled_before, turned_before,
                                       last_why)
                    self._say_route(path, replans)
                remaining = planner.length(path)
                limit = min(MAX_GOTO_S - (time.monotonic() - started),
                            remaining / speed * 2.2 + 4.0)
                if limit < 1.0:
                    break
                outcome = self._run_goal({
                    "kind": "goto",
                    "speed": speed,
                    "path": path,
                    "target": target,
                    "progress": 0.0,
                    "travelled_before": travelled_before,
                    "last_check": time.monotonic(),
                }, limit)
                path = None
                travelled_before = outcome.travelled_m
                turned_before = outcome.turned_deg
                if outcome.reason == "arrived":
                    extra = (f"replanned {replans} time"
                             f"{'' if replans == 1 else 's'}" if replans else "")
                    return Outcome("arrived", outcome.travelled_m, outcome.turned_deg,
                                   extra)
                if outcome.reason == "replan":
                    replans += 1
                    if replans > MAX_REPLANS:
                        return Outcome("blocked", outcome.travelled_m,
                                       outcome.turned_deg,
                                       "gave up replanning: " + (outcome.detail
                                                                 or last_why))
                    # Said before the planner is asked rather than after it answers,
                    # because what provoked the replan is the interesting half and
                    # planning on this host is not instant.
                    self.report.say("replanning", outcome.detail or last_why,
                                    replans=replans, route_m=None, waypoints=None)
                    continue
                if outcome.reason == "stopped":
                    return outcome
                return Outcome(outcome.reason, outcome.travelled_m, outcome.turned_deg,
                               outcome.detail or last_why)
            return Outcome("timed out", travelled_before, turned_before,
                           "the route ran out of its time budget")
        finally:
            self._halt()
            self._end_driving()

    def _say_route(self, path, replans):
        """The planner came back with a route. How long it is and how many corners
        it has is what says whether it went the way you meant it to."""
        self.report.say("driving", route_m=round(planner.length(path), 2),
                        waypoints=len(path), replans=replans)

    def _plan_route(self, target_xy):
        """A polyline from here to `target_xy`, or (None, why).

        Tries the follower's brake distance as keep-out first, then the gap the
        chassis still fits, and starts with a turn when the heading looks into
        the keep-out -- turning is always legal even when the nose is not.
        """
        import numpy as np

        with self.slam.lock:
            grid = np.array(self.slam.grid(), copy=True)
            pose = self.slam.pose
            res = self.slam.config.resolution_m
            occupied_at = self.slam.config.occupied_at
        began = time.monotonic()
        path, why = planner.plan(grid, res, occupied_at, (pose[0], pose[1]),
                                 target_xy,
                                 inflate_m=PLAN_INFLATE_M,
                                 preferred_m=PLAN_PREFERRED_M,
                                 comfort_m=PLAN_COMFORT_M,
                                 start_yaw=pose[2])
        if self._journey:
            # The grid is handed over rather than copied again: this is the copy
            # taken above, and nothing writes to it after the planner has read it.
            self._journey.plan(grid, pose, target_xy, PLAN_INFLATE_M,
                               PLAN_PREFERRED_M, PLAN_COMFORT_M, path, why,
                               time.monotonic() - began)
        return path, why

    @_one_move_at_a_time
    def turn_in_place(self, angle_deg, speed_dps=None):
        """Rotate by this many degrees, counter-clockwise positive.

        Dead reckoned in bursts of fixed PWM, using the rates in TURN_RATES. That is
        roughly six times faster than servoing on the scan match was, and it keeps
        working when the lidar browns out during the turn -- which it does, because
        the motors and the lidar share one 5 V rail.

        The matcher cannot follow 170 deg/s (its window is 90), so matching is
        suspended for each burst and the heading re-seeded from the dead reckoning
        afterwards. That re-seed is a guess, and it has been wrong by 48 degrees --
        five times the window the matcher can search -- so nothing is written to the
        map until a wide search has agreed with a tracking revolution on the same
        pose. Until that happens the pose is the only thing at risk; the map, which
        cannot be repaired because there is no loop closure, is not. A room that
        looks the same two ways round still makes the turn come back lost, but
        mapping takes the winner rather than staying held for good.

        Long turns go in bursts of TURN_BURST_MAX_DEG with a measurement between
        them, because a dead-reckoned error is a fraction of the burst it came from
        and a whole 180 guessed in one go can land outside what the search can undo.

        If the lidar is not reporting, the commanded figure is returned and said to
        be dead-reckoned rather than passed off as a measurement.
        """
        asked = {"angle_deg": round(float(angle_deg), 1)}
        self.report.begin("turn_in_place", asked, "turning")
        # Recorded like a route is, because a turn is where the pose is most
        # likely to move without the rover having gone anywhere: each burst runs
        # with matching suspended and ends in a re-seed that is a guess. A trail
        # cannot be read without knowing which of its kinks were those.
        self._journey = journey.Recorder.if_armed()
        if self._journey:
            self._journey.begin("turn_in_place", asked, self.slam.pose)
        try:
            outcome = self._ended(self._turn_in_place(angle_deg, speed_dps))
        finally:
            recording, self._journey = self._journey, None
        if recording:
            recording.end(outcome.reason, outcome.detail)
        return outcome

    def _turn_in_place(self, angle_deg, speed_dps):
        if self._estop:
            return Outcome("stopped", 0.0, 0.0,
                           "the emergency stop is latched; clear it first")
        angle = _clamp(float(angle_deg), -180.0, 180.0)
        if abs(angle) < 1.0:
            return Outcome("arrived", 0.0, 0.0, "already facing that way")

        careful = self._nearest_recent()
        gentle = careful is not None and careful < TURN_CAREFUL_M

        self._begin_driving()
        try:
            # Accumulated rather than wrapped, because a turn is counted and not
            # merely reported: a measured 181 against a requested 180 wraps to -179
            # and reads as a rover that has gone the wrong way round.
            start_accum = self._heading_accum
            # Held across the whole turn, bursts and re-finds and all, because what
            # calibrates the gyro is one long rotation measured two ways -- and the
            # running mark underneath is being consumed a revolution at a time by
            # the prior. See Odometry.mark.
            odo_mark = self._odom.mark()
            had_lidar = self.lidar_live()
            done = 0.0            # what dead reckoning was told to do, in degrees
            remaining = angle
            blind = not had_lidar
            lost = None

            for _ in range(TURN_MAX_BURSTS):
                if self._estop:
                    break
                # Splitting a turn buys a measurement between the pieces, so with
                # nothing measuring there is nothing to buy: a blind 180 done sixty
                # degrees at a time is only three chances to be interrupted.
                piece = (remaining if blind else
                         _clamp(remaining, -TURN_BURST_MAX_DEG, TURN_BURST_MAX_DEG))
                # Fast for the bulk of it, fine for the last of it or when something
                # is close. Turning is still never refused.
                pwm = (TURN_FINE_PWM if gentle or abs(piece) < TURN_FAST_MIN_DEG
                       else TURN_FAST_PWM)
                stepped = self._burst_turn(pwm, piece)
                if stepped == 0.0:
                    break         # what is left is smaller than the burst's own coast
                done += stepped
                if blind:
                    break         # nothing measured that, and nothing is going to

                ok, health = self._refind()
                if health.get("lidar_gone"):
                    # It was reporting when this burst began and is not now. Finish
                    # on what the last measurement said, less the burst just
                    # commanded -- the best estimate there is once nothing measures.
                    blind = True
                    remaining -= stepped
                    continue
                if not ok:
                    lost = health
                    break
                measured = self._heading_change(start_accum)
                if self._journey:
                    # What the burst was told to do against what the matcher says
                    # it did. Bursts beyond ceil(angle / TURN_BURST_MAX_DEG) are
                    # corrections, and how many there are is the honest measure of
                    # whether TURN_RATES still describes this floor and battery.
                    self._journey.event(
                        "burst",
                        f"asked {piece:+.0f} deg at PWM {pwm}, reckoned "
                        f"{stepped:+.0f}, matcher says {measured:+.0f} of "
                        f"{angle:+.0f} so far")
                remaining = angle - measured
                if abs(remaining) <= TURN_TOLERANCE_DEG:
                    break

            # Three outcomes, and they are worth telling apart. The sensor may have
            # stopped reporting, in which case the commanded turn is all there is to
            # report and it must not be dressed up as a measurement. Or it is
            # reporting but nothing it sees fits the map anywhere near where the turn
            # was supposed to end, which means the rover did not go where dead
            # reckoning thinks -- jammed against something, most likely. Or it fits,
            # and the number handed back is a measurement.
            if blind:
                return Outcome("arrived", 0.0, done,
                               "dead reckoned; the lidar stopped reporting, so this "
                               "is the commanded turn and not a measured one")
            if lost is not None:
                map_bit = (
                    "The map is not being written meanwhile, so nothing is being "
                    "spoiled by it" if self._map_paused else
                    "The map is being written from the heading the matcher kept "
                    "picking -- the better of the answers the room offered -- but "
                    "that heading is not a measurement")
                return Outcome("lost", 0.0, done,
                               f"the turn was dead reckoned as {done:.0f} degrees but "
                               f"{_why_lost(lost)}, so the rover is not where it "
                               f"thinks it is and its heading is not to be trusted "
                               f"until it sees something it recognises. {map_bit}")

            turned = self._heading_change(start_accum)
            # Every burst of this turn was confirmed -- a recovery sweep and the
            # tracking revolution after it landed within 5 degrees and 5 cm of each
            # other, or the loop above would have come back `lost`. That is what
            # makes `turned` a measurement rather than a re-seed repeating itself,
            # and it is the whole reason a scale factor may be fitted to it. How
            # far the turn fell short of what was *asked* is a different question
            # and does not bear on this one.
            self._calibrate_turn(turned, odo_mark)
            error = angle - turned
            detail = ""
            if gentle:
                detail = f"turned slowly because something was {careful:.2f} m away"
            if abs(error) > TURN_TOLERANCE_DEG * 2:
                detail = (((detail + "; ") if detail else "")
                          + f"still {error:.0f} degrees out after correcting")
            return Outcome("arrived", 0.0, turned, detail)
        finally:
            self._halt()
            self._end_driving()

    def _burst_turn(self, pwm, angle):
        """One open-loop burst, ending with the pose moved to match. Returns degrees.

        Map updates are suspended throughout: at these rates the matcher cannot keep
        up, and a scan folded in at a pose a quarter turn out damages the map for
        good. The pose is moved to where dead reckoning says the rover ended up
        *before* the suspension is lifted, so there is no revolution between the two
        that could be integrated at the heading the turn started from.

        Blind time is bounded by the arithmetic: at the fast rate even a half turn is
        just over a second, and the worst case in the file is a 180 in close quarters,
        which runs at the fine rate for five and a half seconds. Rotating on the spot
        cannot carry the rover into anything it was not already touching, which is
        what makes that acceptable where the same blindness while driving would not
        be.

        Re-seeding is a loaded gun. It *tells* the matcher where it is, and the
        ordinary search window is only about 9 degrees, so if the dead reckoning was
        well out the matcher cannot climb back on its own and will simply agree with
        the wrong answer. Observed: a turn that physically managed 42 degrees of a
        requested 90 was reported as 90, because that is what it had been told.

        So this leaves two things behind for the caller to clean up, and _refind is
        the caller doing it. Mapping is suspended, so the wrong answer cannot be
        written into the map and become the thing the next revolution matches
        against. And a recovery search is asked for, which sweeps far wider than the
        tracking window and can therefore find an answer the re-seed missed by tens
        of degrees. Neither costs anything if the re-seed was right.
        """
        rate, coast = TURN_RATES[pwm]
        hold = (abs(angle) - coast) / rate
        if hold <= 0.0:
            return 0.0
        sign = 1 if angle > 0 else -1
        turned = sign * (rate * hold + coast)
        self._suspend_slam = True
        try:
            self.link.send({"T": CMD_HEARTBEAT, "cmd": HEARTBEAT_MS})
            end = time.monotonic() + hold
            while time.monotonic() < end and not self._estop:
                # Counter-clockwise is left track back, right track forward.
                self._send(-sign * pwm, sign * pwm)
                time.sleep(0.05)
            self._halt()
            with self.slam.lock:
                x, y, th = self.slam.pose
                # Dead reckoning's answer, and only ever the starting point for the
                # search that follows. An estop part way through the burst lands here
                # too, with a `turned` the rover never completed -- which the recovery
                # search undoes like any other bad guess.
                self.slam.pose = (x, y, th + math.radians(turned))
            # The board has been reporting throughout the burst, and the running
            # span has not been consumed since nothing matched. Left alone, the
            # first revolution after this would be handed a prior covering the
            # whole turn. The turn's own span is kept separately by the caller --
            # see _turn_in_place -- so nothing is lost by starting again here.
            self._odom.reset()
            self._pause_mapping("a turn was dead reckoned and nothing has "
                                "confirmed where it ended yet")
        finally:
            self._suspend_slam = False
        return turned

    # --- knowing whether the pose is worth believing --------------------------

    def _match_health(self):
        """How the last match was won, and whether that is a pose to build on.

        `map_ok` is the one bar computed here: the scan fits, the winner was not
        jammed against the rim of the window, and the match was not rejected.
        Mapping takes that and a pose that holds still, even when the room is
        ambiguous -- freezing the map forever in a rectangle is worse than picking
        the better of the two answers. A turn needs more: no rival heading worth
        the winner. That bar is deliberately not computed here, because the
        ambiguity worth judging it by is the recovery sweep's and this runs every
        revolution -- a tracking match reports 0.0 however symmetric the room is.
        _refind keeps the sweep's figure and applies it.

        The score on its own is not the bar. A scan that has snapped onto the
        wrong-but-self-consistent alignment scores *high* -- scoring high is
        exactly why that pose beat the others. See slam2d.h.
        """
        with self.slam.lock:
            health = {"score": round(self.slam.score, 3),
                      "edge": self.slam.match_edge,
                      "ambiguity": round(self.slam.ambiguity, 3),
                      "rejected": self.slam.rejected,
                      "pose": self.slam.pose,
                      "recovery": self._recovery_this_scan}
        health["map_ok"] = (not health["rejected"]
                            and health["score"] >= self._min_write_score
                            and not health["edge"])
        return health

    def _refind(self, within_s=TURN_REFIND_S):
        """Wait for the matcher to agree with a re-seeded pose, or give up on it.

        Per revolution rather than after a fixed wait, which is the difference that
        matters: the old code slept seven revolutions and then read the score once,
        by which time seven scans had already been folded into the map at whatever
        heading the re-seed invented. The C core will not write below min_write_score
        or from an edge match, and mapping stays paused here until the pose holds
        still across a confirming pair -- so the cost of being wrong is revolutions
        of pose, not a second copy of the room.

        A turn is only believed if the *recovery* sweep had no rival. The confirming
        revolution is a tracking match, which cannot see 90 degrees away and so
        reports ambiguity 0.0 even when the room has two answers; the figure from
        the wide search is the one that counts.

        Returns (ok, health) -- see _match_health.
        """
        # The burst coasts, and a scan taken mid-coast is smeared across the
        # rotation. Judging the re-seed on it would fail turns that were fine, and
        # would also let _note_match resume mapping from those smeared poses.
        self._hold_confirm = True
        try:
            time.sleep(TURN_SETTLE_S)
        finally:
            self._hold_confirm = False
        self._good_run = 0
        self._confirm_pose = None
        self._need_recovery = True
        self._wide_recovery = True     # a burst is the error the sweep is for
        self._lost_run = 0
        deadline = time.monotonic() + within_s
        seen = self._scans
        good = 0
        confirm = None
        recovery_amb = None
        health = dict(self._health)
        while time.monotonic() < deadline:
            if not self.lidar_live():
                # Not the same failure as a re-seed that could not be confirmed, and
                # the caller has to tell them apart: one means the rover is lost,
                # the other only that nothing is looking.
                return False, {"lidar_gone": True,
                               "reason": "the lidar stopped reporting"}
            if self._scans == seen:
                time.sleep(0.02)
                continue
            seen = self._scans
            # _note_match already ran on this revolution; its health is the record.
            health = dict(self._health)
            pose = health.get("pose")
            if health.get("recovery"):
                recovery_amb = health.get("ambiguity")
            if not health.get("map_ok") or pose is None:
                good = 0
                confirm = None
                continue
            if confirm is None or not _pose_close(pose, confirm):
                confirm = pose
                good = 1
                continue
            good += 1
            if good < REACQUIRE_GOOD_SCANS:
                continue
            amb = (recovery_amb if recovery_amb is not None
                   else health.get("ambiguity", 0.0))
            if amb < RESEED_MAX_AMBIGUITY:
                return True, health
            # The pair has agreed, and no more wide sweeps are coming that could
            # retract the rival -- the verdict cannot improve, so hand it back
            # now rather than after the rest of the deadline.
            break
        # The tracking confirmation reports ambiguity 0.0, which is a window too
        # narrow to hold a rival, not a clean bill of health. The figure from the
        # recovery sweep is the one that made the turn lost.
        if recovery_amb is not None:
            health = dict(health)
            health["ambiguity"] = recovery_amb
        return False, health

    def _pause_mapping(self, why, wide=True):
        """Match, but write nothing, until something confirms where the rover is.

        `wide` is whether getting the pose back needs the +/-60 degree sweep. A
        dead-reckoned turn does: the heading can be tens of degrees out and the
        tracking window spans nine. A revolution that merely landed on the rim while
        driving does not, and asking for the sweep there makes matters worse rather
        than better -- see WIDEN_AFTER_LOST for what it costs. It widens on its own
        if ordinary tracking turns out not to find the pose either.
        """
        if not self._map_paused:
            self._log_event("map held", why)
        with self.slam.lock:
            self.slam.mapping = False
        self._map_paused = True
        self._good_run = 0
        self._confirm_pose = None
        self._need_recovery = True
        self._wide_recovery = wide
        self._lost_run = 0

    def _resume_mapping(self, why):
        if self._map_paused:
            self._log_event("map resumed", why,
                            score=self._health.get("score"),
                            edge=self._health.get("edge"),
                            ambiguity=self._health.get("ambiguity"))
        with self.slam.lock:
            self.slam.mapping = True
        self._map_paused = False
        self._good_run = 0
        self._confirm_pose = None
        self._need_recovery = False
        self._wide_recovery = False
        self._lost_run = 0
        # Whatever the pose did while the map was held -- a recovery sweep, a
        # re-seed, tens of degrees of legitimate correction -- is not the matcher
        # creeping away from a still chassis, and carrying it into the creep test
        # would accuse a rover that has just proved itself. See odometry.py.
        self._odom.forget_quiet()

    def _log_event(self, what, why, **fields):
        """A short history of losing and regaining the pose, for whoever asks later.

        The point of keeping it is that the two ways this goes wrong need telling
        apart and neither is visible after the fact: a rover that could not keep up
        shows dropped revolutions and an answer against the rim of the window, while
        a room that looks the same two ways round shows a rival peak and no drops at
        all. Both used to end as the same misaligned map with nothing to say why.
        """
        event = {"at": time.monotonic(), "what": what, "why": why,
                 "dropped_total": self._dropped}
        event.update(fields)
        self._events.append(event)
        if len(self._events) > SLAM_EVENTS:
            del self._events[0]
        # This list is a short ring for status(); a journey keeps the lot, because
        # the interesting question afterwards is which re-seed the pose jumped on.
        if self._journey:
            self._journey.event(what, why)

    def _note_match(self):
        """Record how the last match went, and hold or heal the map from it.

        One untrustworthy revolution is enough to pause: C will not stamp it, but
        the next one will not get a wide search unless mapping is held. While held,
        the first healthy match drops the wide search so the confirming revolution
        is an ordinary tracking match that has to land in the same place -- two
        independent +/-60 degree answers agreeing is not the same as the tracker
        agreeing with the recovery. Mapping resumes once that pair agrees, even if
        the room looked the same two ways round: the matcher already picked a
        winner, and a frozen map in a rectangle never grows again.
        """
        health = self._match_health()
        self._health = health
        if health["rejected"]:
            self._rejects += 1
        if health["edge"]:
            self._edges += 1
        if self._hold_confirm:
            return
        pose = health.get("pose")
        # The seed scan has score 0 and is not a failure -- it *is* the map.
        matched = health["score"] > 0.0 or health["rejected"] or health["edge"]

        if not self._map_paused:
            self._good_run = 0
            self._confirm_pose = None
            if matched and not health["map_ok"]:
                # Not the dead-reckoned case: the rover is driving and the matcher
                # was tracking it a revolution ago, so it is a few degrees out, not
                # tens. The ordinary window reaches that and the sweep would not.
                self._pause_mapping(_why_lost(health), wide=False)
                return
            # Everything above judges the match by the search that produced it, and
            # there is one failure that search cannot see: a scan that has snapped
            # onto a wrong-but-self-consistent alignment scores *high*, because
            # scoring high is why that pose won. The gyro is the only thing here
            # that is not the matcher, so it is the only thing that can disagree.
            #
            # Only in this branch. A recovery sweep legitimately moves the heading
            # tens of degrees with the chassis standing still, and so does the
            # re-seed after a dead-reckoned turn; both live on the paused-map side
            # of this test, where the machinery is already doing the right thing.
            self._witness(health)
            return

        if not health["map_ok"] or pose is None:
            self._good_run = 0
            self._confirm_pose = None
            self._need_recovery = True
            self._lost_run += 1
            if self._lost_run >= WIDEN_AFTER_LOST and not self._wide_recovery:
                # Tracking has had its go and cannot find it, so this is the error
                # the sweep exists for after all.
                self._wide_recovery = True
                self._log_event("searching wide",
                                f"the ordinary window has not found the pose in "
                                f"{self._lost_run} revolutions, so the search "
                                f"widens to +/-60 degrees")
            return

        if self._confirm_pose is None or not _pose_close(pose, self._confirm_pose):
            self._confirm_pose = pose
            self._good_run = 1
            self._lost_run = 0
            # Next revolution tracks rather than searching +/-60 deg again, so a
            # flip to a rival peak 90 deg away cannot masquerade as confirmation.
            self._need_recovery = False
            return

        self._good_run += 1
        if self._good_run >= REACQUIRE_GOOD_SCANS:
            self._resume_mapping("the scan fits the map again")

    def _calibrate_turn(self, degrees, mark):
        """Fit the gyro's scale to a turn the matcher has confirmed.

        The pleasing part of this arrangement is that it costs no manoeuvre of its
        own. Every `turn_in_place` the rover makes for its own reasons is also a
        measurement, because the matcher's heading is the absolute reference the
        gyro has never had -- so the scale factor arrives by driving rather than by
        ceremony, and goes on being refined for as long as the rover turns.
        """
        span = self._odom.between(mark, self._odom.mark())
        taken, why = self._odom.note_turn(degrees, span)
        state = self._odom.status()
        if taken:
            self._log_event("gyro measured",
                            f"a confirmed {degrees:+.0f} degree turn against what "
                            f"the gyro integrated over it",
                            gyro_lsb_per_dps=state["gyro_lsb_per_dps"],
                            turns_measured=state["turns_measured"])
        else:
            self._log_event("gyro not measured", why,
                            turns_measured=state["turns_measured"])

    def _calibrate_drive(self, outcome):
        """Fit the wheels' scale to a drive the matcher has confirmed.

        Straight ones only. The wheel counts measure the arc the wheels rolled and
        `travelled` is the straight line between the ends, so a drive that steered
        round something is measuring two different lengths and would fit the scale
        short.
        """
        mark, since = self._drive_mark, self._drive_marked_at
        self._drive_mark = self._drive_marked_at = None
        if mark is None or outcome.reason != "arrived":
            return                      # not a drive that measured anything
        if abs(outcome.turned_deg) > CALIBRATE_MAX_DRIVE_TURN_DEG:
            self._log_event("wheels not measured",
                            f"the drive curved {outcome.turned_deg:+.0f} degrees, so "
                            f"the wheels rolled an arc and the matcher measured the "
                            f"chord")
            return
        rejected = 0 if since is None else self._rejects - since[0]
        edges = 0 if since is None else self._edges - since[1]
        if rejected:
            self._log_event("wheels not measured",
                            f"{rejected} revolutions of that drive fitted the map "
                            f"nowhere, so the distance it reports is not a "
                            f"measurement")
            return
        # The path the matcher traced, not the straight line between the ends. On a
        # drive that wandered 23 degrees those differ by more than the measurement
        # is worth, and the wheels rolled the path.
        path = self._path_m - since[2]
        span = self._odom.between(mark, self._odom.mark())
        taken, why = self._odom.note_drive(path, span)
        state = self._odom.status()
        if taken:
            self._log_event("wheels measured",
                            f"a confirmed {path:.2f} m of travel -- "
                            f"{outcome.travelled_m:.2f} m of it in a straight line "
                            f"-- against the wheel counts over it",
                            # The two raw sides of the fit, so a scale factor that
                            # disagrees with what the prior does per revolution can
                            # be taken apart rather than argued about.
                            path_m=round(path, 3),
                            ticks=None if span is None else span.ticks,
                            seconds=None if span is None else round(span.dt, 2),
                            ticks_per_metre=state["ticks_per_metre"],
                            drives_measured=state["drives_measured"],
                            # Each of these is a revolution whose pose came back a
                            # few centimetres short, so a drive with several of them
                            # measures the wheels slightly long. Reported rather
                            # than refused, and the fit's own spread is the check.
                            window_overruns=edges)
        else:
            self._log_event("wheels not measured", why,
                            drives_measured=state["drives_measured"])

    def _witness(self, health):
        """Let the gyro contradict the scan match, and hold the map if it does.

        The response is deliberately the same one a weak match gets -- pause the
        map and go looking -- rather than anything new. A disagreement does not say
        which of the two is wrong, only that they cannot both be right, and the
        safe reading of that is the one this code already has a mechanism for:
        stop drawing, keep matching, and resume when two revolutions agree.

        Cheap enough to run every revolution: it is a subtraction and a compare
        against a threshold the resting rover measured for itself.
        """
        pose = health.get("pose")
        if pose is None or self._last_pose is None or self._span is None:
            return
        # _measure has not run for this revolution yet -- _loop calls this first --
        # so _last_pose is still the previous revolution's and this is the step.
        dth = (pose[2] - self._last_pose[2] + math.pi) % (2 * math.pi) - math.pi
        why = self._odom.disagreement(self._span, dth)
        if why:
            self._pause_mapping(why, wide=False)

    def _heading_change(self, since_accum):
        """Degrees turned since a mark taken from the accumulating heading."""
        return math.degrees(self._heading_accum - since_accum)

    def stop(self, latch=False):
        """Stop now. `latch` makes it stick until cleared, so a caller that has lost
        confidence can guarantee stillness rather than hope the next command is a
        stop."""
        with self._lock:
            self._goal = None
            self._want_speed = self._want_turn = 0.0
            if latch:
                self._estop = True
        self._halt()
        # Only while something is actually moving. A stop pressed on a still rover
        # is a reasonable thing to do and would otherwise wipe the last move's
        # ending off the screen of whoever pressed it.
        if self._driving:
            self.report.say("stopping", "a stop was asked for"
                                        + (" and latched" if latch else ""))
        return {"stopped": True, "latched": self._estop}

    def clear_estop(self):
        with self._lock:
            self._estop = False
        return {"latched": False}

    def _run_goal(self, goal, limit_s):
        why = self._preflight(goal["kind"])
        if why:
            return Outcome("blocked", 0.0, 0.0, why)
        with self._lock:
            if self._goal is not None:
                return Outcome("busy", 0.0, 0.0, "a move is already running")
            start_pose = self.slam.pose
            goal["start_accum"] = self._heading_accum
            goal["started_at"] = time.monotonic()
            goal["deadline"] = goal["started_at"] + limit_s
            goal["start_pose"] = start_pose
            goal["done"] = None
            goal["travelled"] = 0.0
            goal["turned"] = 0.0
            self._goal = goal

        if not self._driving:
            self._begin_driving()
            owned = True
        else:
            owned = False
        # A backstop on the wait itself, not only on the move. Every deadline below
        # this one is checked inside a per-scan step, so if scans stop arriving
        # nothing checks anything and this loop waits for ever -- which is what
        # happened when the lidar port vanished: the goal never finished, and every
        # later move was refused as "a move is already running" until the daemon was
        # restarted. A tool call must always return.
        hard_stop = goal["deadline"] + 2.0
        try:
            while True:
                with self._lock:
                    g = self._goal
                    if g is None or g["done"] is not None:
                        break
                    if time.monotonic() > hard_stop:
                        g["done"] = ("stopped",
                                     "the control loop stopped responding, so the "
                                     "move was abandoned and the rover halted")
                        break
                time.sleep(0.02)
        finally:
            with self._lock:
                g, self._goal = self._goal, None
                self._want_speed = self._want_turn = 0.0
            self._halt()
            if owned:
                self._end_driving()

        if g is None:
            return Outcome("stopped", 0.0, 0.0, "cancelled")
        reason, detail = g["done"] or ("stopped", "")
        if self._journey:
            self._journey.event(reason, detail)
        return Outcome(reason, g["travelled"], g["turned"], detail)

    def _begin_driving(self):
        self._driving = True
        if self.on_drive_start:
            try:
                self.on_drive_start()
            except Exception:
                pass
        # Before anything moves: this is what stops the rover if this process dies
        # or the link drops. Gimbal commands deliberately do not feed it, so aiming
        # the camera can never be mistaken for driving.
        self.link.send({"T": CMD_HEARTBEAT, "cmd": HEARTBEAT_MS})
        self._heartbeat_set = True

    def _end_driving(self):
        self._driving = False
        if self.on_drive_end:
            try:
                self.on_drive_end()
            except Exception:
                pass

    def _halt(self):
        for _ in range(STOP_REPEATS):
            self.link.send({"T": CMD_PWM, "L": 0, "R": 0})
        self._last_sent = (0, 0)

    # --- the loop -------------------------------------------------------------
    def _open_lidar(self):
        """Open the port if it is not open, no more often than every LIDAR_REOPEN_S."""
        if self.lidar is not None:
            return True
        now = time.monotonic()
        if now < self._reopen_at:
            return False
        self._reopen_at = now + LIDAR_REOPEN_S
        path = find_lidar(self._lidar_pref)
        if not path:
            return False
        try:
            self.lidar = serial.Serial(path, LIDAR_BAUD, timeout=0.05)
        except (OSError, serial.SerialException):
            self.lidar = None
            return False
        self.lidar_path = path
        # Where it is on the bus, for the recovery that has to work after it is no
        # longer anywhere. Best-effort: a port that is not a USB device at all --
        # somebody testing over a pty -- simply leaves this empty and the ladder
        # stops one rung short, which it says.
        self._lidar_usb = usbreset.usb_path_for(path) or self._lidar_usb
        # The sensor has been spinning the whole time. The first wrap we see is a
        # remnant of the revolution we joined in the middle of; without this it
        # was stamped as the seed and later full scans would not match it, so
        # mapping froze on a wedge of room until the map was cleared.
        self.slam.resync()
        return True

    def _drop_lidar(self):
        try:
            if self.lidar is not None:
                self.lidar.close()
        except Exception:
            pass
        self.lidar = None
        self.lidar_path = None

    def quiet_for(self):
        """Seconds since the sensor last said anything at all, or None if it is not
        being waited for yet.

        Measured from the loop starting rather than from the first packet, so that a
        rover whose lidar was already missing when the daemon came up is not treated
        as a rover whose lidar has never been due. That is the case the recovery is
        most needed in: a Pi that rebooted with the port already wedged.
        """
        since = self._last_packet_at or self._lidar_watch_from
        return None if since is None else time.monotonic() - since

    def _mind_the_lidar(self, now):
        """Get the sensor talking again, escalating as far as it takes.

        Called from the read loop, which is the only place that knows how long it
        has been since anything arrived. Never during a move and never during a
        dead-reckoned turn: the first because resetting a hub mid-drive takes the
        camera with it and the move is already being stopped by the watchdog for
        the same silence, the second because a suspended map is silence by design.
        """
        if self._driving or self._suspend_slam:
            return
        quiet = self.quiet_for()
        if quiet is None or quiet < LIDAR_SILENT_S:
            return

        # Rung one: an open port that has gone quiet is a handle to something that
        # is no longer there often enough to be worth trying first, and it costs a
        # reopen.
        if self.lidar is not None and quiet < LIDAR_RESET_AFTER_S:
            self._log_event("lidar silent",
                            f"nothing for {quiet:.0f} s, reopening the port")
            self._drop_lidar()
            return

        # Rung two: the device, or the nearest hub above where it was.
        if quiet < LIDAR_RESET_AFTER_S or now < self._reset_at:
            return
        self._drop_lidar()
        attempt = usbreset.revive(self._lidar_usb, self._reset_rung)
        self._resets += 1
        self._reset_note = attempt.why
        self._log_event("lidar reset" if attempt.ok else "lidar reset refused",
                        f"silent for {quiet:.0f} s: {attempt.why}")
        # Escalate first, back off second. While there is something bigger left to
        # try, the next attempt comes at the same interval and reaches one rung
        # higher -- resetting the device that did not answer a second time is not a
        # second attempt at anything. Once the ladder is out, the wait doubles, so a
        # lidar that is genuinely unplugged is not knocking the camera out every
        # minute for the rest of the afternoon.
        self._reset_at = now + self._reset_wait
        if attempt.more:
            self._reset_rung += 1
        else:
            self._reset_wait = min(LIDAR_RESET_MAX_COOLDOWN_S, self._reset_wait * 2)
        # Straight away, rather than after the ordinary reopen wait: the device is
        # a second or two from re-enumerating and there is nothing else to do until
        # it does.
        self._reopen_at = 0.0

    def reset_lidar(self):
        """Reset the lidar's USB device now, because somebody asked.

        The same act the ladder above reaches on its own, exposed so that a person
        watching a scan age climb does not have to wait out the cooldown -- and so
        that the thing which fixes it is in the console rather than in an ssh
        session. Refused while driving, for the reason the ladder does not do it
        either: the reset takes the camera down with it.
        """
        if self._driving:
            return {"ok": False, "error": "not while the rover is driving"}
        self._drop_lidar()
        attempt = usbreset.revive(self._lidar_usb, self._reset_rung)
        self._resets += 1
        self._reset_note = attempt.why
        self._log_event("lidar reset" if attempt.ok else "lidar reset refused",
                        f"asked for: {attempt.why}")
        if attempt.more:
            self._reset_rung += 1
        self._reopen_at = 0.0
        # The wait starts over: an asked-for reset is a fresh judgement that this is
        # worth doing, and it should not be followed by a quarter of an hour of the
        # automatic ladder declining to.
        self._reset_wait = LIDAR_RESET_COOLDOWN_S
        self._reset_at = time.monotonic() + LIDAR_RESET_COOLDOWN_S
        return {"ok": attempt.ok, "reset": attempt.what, "reason": attempt.why,
                "resets": self._resets, "more_to_try": attempt.more}

    def scan_age(self):
        """Seconds since the last revolution that matched and was mapped, or None if
        there has been none at all."""
        if self._last_scan_at is None:
            return None
        return time.monotonic() - self._last_scan_at

    def lidar_ok(self):
        """Is there a current position to drive on? What the driving path asks."""
        age = self.scan_age()
        return age is not None and age <= LIDAR_STALE_S

    def lidar_live(self):
        """Is the sensor still turning and reporting, whatever is done with it?

        Different from :meth:`lidar_ok`, and the difference matters exactly once: a
        dead-reckoned turn suspends the map, so no revolution is matched for the
        length of the burst and `lidar_ok` goes false on a rover whose lidar is
        perfectly healthy. Reporting that as "the lidar stopped reporting" would send
        somebody looking for an electrical fault that is not there -- and the rover
        does have one of those, which is precisely why the two must not be confused.
        """
        if self._last_packet_at is None:
            return False
        return time.monotonic() - self._last_packet_at <= LIDAR_STALE_S

    def _loop(self):
        # When this started waiting to hear from the sensor. Not the same as the
        # first packet, which on a rover whose lidar is already missing never comes.
        self._lidar_watch_from = time.monotonic()
        while self._run.is_set():
            if not self._open_lidar():
                self._watchdog()
                self._mind_the_lidar(time.monotonic())
                time.sleep(0.05)
                continue
            try:
                waiting = self.lidar.in_waiting
                chunk = self.lidar.read(waiting if waiting > 0 else 1)
            except (OSError, serial.SerialException):
                # The port went away under us -- a replug, or the adapter
                # re-enumerating. Drop it and let _open_lidar find it again under
                # whatever name it comes back as.
                self._drop_lidar()
                continue
            if chunk:
                revolutions = self.slam.feed(chunk)
                if revolutions:
                    self._last_packet_at = time.monotonic()
                    # The sensor is talking, so whatever it took to get it talking
                    # worked: the next silence starts from patience again, and from
                    # the gentlest rung of the ladder rather than the one that
                    # happened to be reached this time.
                    self._reset_wait = LIDAR_RESET_COOLDOWN_S
                    self._reset_rung = 0
                if revolutions and not self._suspend_slam:
                    self._dropped += revolutions - 1
                    self._recovery_this_scan = False
                    if (self._map_paused and self._need_recovery
                            and self._wide_recovery):
                        # Nothing is being written, so the pose is all there is to
                        # get back, and the ordinary window cannot reach it from
                        # tens of degrees out. Asked for until the first healthy
                        # match, then the confirming revolution tracks -- a second
                        # wide sweep is free to pick a different peak.
                        self.slam.request_recovery()
                        self._recovery_this_scan = True
                    # The prior goes in before the match, because centring the
                    # search window is the whole of what it does. Zero until the
                    # scale factors have been measured, which is a legitimate
                    # prior rather than a fallback -- see odometry.py.
                    # Drained here rather than by a thread of its own. The
                    # board's stream has to be read at something like its own
                    # rate for the gyro's timing to mean anything, and a read
                    # folded into a loop that was going to run anyway costs no
                    # wakeup -- which on this one core is the whole cost. See
                    # TELEMETRY_POLL_S in rover_daemon.py.
                    pump = getattr(self.link, "pump", None)
                    if pump is not None:
                        pump()
                    self._span = self._odom.span()
                    self.slam.set_prior(*self._odom.prior(self._span))
                    if self.slam.update():
                        self._last_scan_at = time.monotonic()
                        self._note_match()
                        self._scans += 1
                        self._on_scan()
            self._watchdog()
            self._mind_the_lidar(time.monotonic())
            # Keep the board's heartbeat fed even when the PWM has not changed, or
            # it stops the base mid-move.
            if (self._driving and self._last_sent
                    and time.monotonic() - self._last_send_at > KEEPALIVE_S):
                self._send(*self._last_sent)

    def _watchdog(self):
        """Stop a move the moment the sensor stops reporting.

        The per-scan checks cannot cover this on their own: if the scans stop
        arriving, nothing calls them, and the move would coast on the last PWM until
        its deadline. The board's own heartbeat would eventually catch it, but half a
        second of blind driving is exactly what the standoff exists to prevent.
        """
        if self._suspend_slam:
            # A dead-reckoned turn is deliberately blind and suspends the map, which
            # by itself makes the matched clock look stale after a second. Stopping
            # on that would be this loop fighting the burst it is supposed to be
            # letting happen: three zero-PWM packets from here against every one of
            # the burst's own, at a far higher rate, all the way to the end. That is
            # what made turning slow, jerky and short of what was asked -- a turn
            # long enough to cross the staleness threshold got shut down a second in
            # and stuttered through whatever was left, which is every turn in close
            # quarters and any turn past about 170 degrees. A burst is bounded by its
            # own clock; that is the safety here, and the lidar is not part of it.
            return
        if not self._driving or self.lidar_ok():
            return
        with self._lock:
            goal = self._goal
            if goal is not None and goal["done"] is None:
                goal["done"] = ("lost the lidar",
                                "the lidar stopped reporting mid-move, so the rover "
                                "stopped rather than drive on what it last saw")
            self._want_speed = self._want_turn = 0.0
        self._halt()

    def _on_scan(self):
        now = time.monotonic()
        pose = self.slam.pose
        self._measure(pose, now)

        # A gyro's zero drifts with temperature, and the witness's threshold is a
        # spread around that zero -- so both have to be learnt from the rover
        # standing still, and re-learnt as the afternoon goes on. Standing still is
        # most of what this rover does, so there is no ceremony in it. Both tests
        # matter: `_driving` is false while a script waits or a conversation runs,
        # and the matcher's own numbers catch the rover being pushed, which would
        # otherwise teach the gyro to expect that push.
        if (not self._driving and self._span is not None
                and abs(self._measured_turn) < REST_MAX_DPS
                and abs(self._measured_speed) < REST_MAX_MS):
            self._odom.learn_rest(self._span)

        # Kept every revolution, driving or not, so a move that is about to start
        # already has half a second of history to be cautious with.
        self._near_history.append(self._nearest())
        if len(self._near_history) > NEAR_HISTORY:
            del self._near_history[0]

        with self._lock:
            goal = self._goal
            estop = self._estop

        if self._journey:
            self._journey.tick(pose, self._match_health(),
                               (self._measured_speed, self._measured_turn),
                               (self._want_speed, self._want_turn),
                               self._chosen_deg, self._clearance,
                               (goal or {}).get("progress", 0.0),
                               (goal or {}).get("cross", 0.0),
                               self._dropped)

        if estop or goal is None:
            if self._last_sent not in (None, (0, 0)):
                self._halt()
            return

        # Bound once: clear_map swaps a fresh list in from another thread, and
        # reading self._trail twice across that swap is how a length check passes
        # on the old list and the index that follows it lands on the empty new one.
        trail = self._trail
        if len(trail) < 4000 and (
                not trail
                or math.hypot(pose[0] - trail[-1][0],
                              pose[1] - trail[-1][1]) > 0.05):
            trail.append((pose[0], pose[1]))

        if goal["kind"] == "goto":
            self._step_goto(goal, pose, now)
        else:
            self._step_drive(goal, pose, now)


    def _measure(self, pose, now):
        """Speed and turn rate from the scan matcher, since nothing else measures
        them on this rover."""
        if self._last_pose is not None and self._last_at is not None:
            dt = now - self._last_at
            if 1e-3 < dt <= MAX_MATCH_GAP_S:
                # Smoothed hard, because the cap it feeds should follow how the
                # loop is doing over a second or so and not flinch at one late
                # revolution. Intervals longer than MAX_MATCH_GAP_S are the loop
                # having stopped rather than slowed, and are not evidence of a
                # rate to plan around.
                self._match_gap += (dt - self._match_gap) * 0.2
            if dt > 1e-3:
                dx, dy = pose[0] - self._last_pose[0], pose[1] - self._last_pose[1]
                # Signed along the heading we had, so reversing reads negative
                # rather than as forward motion.
                heading = self._last_pose[2]
                along = dx * math.cos(heading) + dy * math.sin(heading)
                dth = (pose[2] - self._last_pose[2] + math.pi) % (2 * math.pi) - math.pi
                # Safe to accumulate a wrapped step: one revolution's rotation is a
                # few degrees, nowhere near the half turn that would be ambiguous.
                self._heading_accum += dth
                # Lightly smoothed: one revolution of match noise is a few
                # millimetres and would otherwise fight the speed loop.
                self._path_m += along
                self._measured_speed += 0.5 * (along / dt - self._measured_speed)
                self._measured_turn += 0.5 * (math.degrees(dth) / dt
                                              - self._measured_turn)
        self._last_pose, self._last_at = pose, now

    def _turn_limit(self):
        """The fastest turn the scan matcher can still follow, right now.

        The coarse pass searches a fixed number of degrees either side of where
        the rover was, so what it can tolerate is a rotation *per matched
        revolution*, not per second. Divide the window by the interval the loop
        is actually delivering and that becomes a rate -- one that rises when the
        Pi is keeping up and falls when it is not, instead of a constant that was
        only ever right at 10 Hz.

        Floored at MIN_TURN_DPS: if the loop has fallen so far behind that even a
        crawl would outrun the window, refusing to turn is worse than turning
        badly, because turning is the move a cornered rover always still has.
        """
        window = float(self.slam.config.coarse_ang_deg
                       * self.slam.config.coarse_ang_steps)
        gap = min(max(self._match_gap, 1e-3), MAX_MATCH_GAP_S)
        return _clamp(window * TURN_WINDOW_USE / gap, MIN_TURN_DPS, MAX_TURN_DPS)

    def _headroom(self, curvature, comfort=False):
        half = self.slam.config.rover_width_m * 0.5 + (
            COMFORT_MARGIN_M if comfort else CORRIDOR_MARGIN_M)
        return self.slam.arc_clearance(curvature, half, LOOKAHEAD_M + STANDOFF_M)

    def _choose_heading(self, want_deg):
        """Follow-the-gap: the heading with the most room, penalised for departing
        from the one asked for.

        Scored on two corridors, and that is the part that matters. The wide one
        decides where to go, so a route that passes things with clearance beats one
        that merely squeezes past; the tight one is carried along so the caller can
        brake on what is actually unsafe rather than on what is merely snug. Scoring
        on the tight corridor alone is what made the rover graze walls and wedge
        itself into corners -- it had no reason to prefer space it was not yet
        touching.

        A wall met at a shallow angle produces a clearance gradient in the wide
        corridor while it is still a comfortable distance off, so wall-following
        falls out of this rather than being a special case with a threshold to tune.
        """
        best, best_score, best_clear = want_deg, -1e9, 0.0
        for offset in range(-40, 41, 5):
            heading = want_deg + offset
            if abs(heading) > 55:
                continue
            # Curvature that swings the nose by `heading` over the lookahead.
            curvature = 2.0 * math.sin(math.radians(heading)) / LOOKAHEAD_M
            roomy = min(self._headroom(curvature, comfort=True), LOOKAHEAD_M)
            tight = min(self._headroom(curvature), LOOKAHEAD_M)
            # A degree of detour is worth about a centimetre of room: enough to
            # prefer the open side, not enough to spin on the spot at the first
            # thing it sees. The tight corridor gets a small say so that a heading
            # which is merely passable is still better than one that is blocked.
            score = (roomy + 0.3 * tight
                     - 0.010 * abs(heading - want_deg) - 0.004 * abs(heading))
            if score > best_score:
                best, best_score, best_clear = heading, score, tight
        return best, best_clear

    def _speed_limit(self, clear):
        """What is safe given how far it can see, so it can always stop by the
        standoff."""
        usable = clear - STANDOFF_M - REACT_MARGIN_M
        if usable <= 0.0:
            return 0.0
        return min(MAX_SPEED_MS, math.sqrt(2.0 * DECEL_MS2 * usable))

    def _unknown_ahead(self):
        sectors = self.slam.sectors(36)
        n = UNKNOWN_AHEAD_SECTORS
        ahead = [sectors[i % 36] for i in range(-n, n + 1)]
        return sum(1 for v in ahead if v is None)

    def _nearest(self):
        """Closest thing in any direction this revolution, or None if nothing came
        back at all."""
        known = [v for v in self.slam.sectors(36) if v is not None]
        return min(known) if known else None

    def _nearest_recent(self):
        """The closest thing seen in the last few revolutions -- see NEAR_HISTORY.

        Deliberately pessimistic. Something that shows up in one scan out of five is
        still there in the other four; the sensor just did not get an echo back.
        """
        seen = [v for v in self._near_history if v is not None]
        if seen:
            return min(seen)
        return self._nearest()

    def _preflight(self, kind):
        """Whether this move can start, checked before anything else happens.

        Before, specifically, face tracking is put down: a request that was never
        going to move should not cost the camera a stop and a restart, and on this
        host restarting it costs v4l2-ctl's start-up all over again.
        """
        if self._estop:
            return "the emergency stop is latched; clear it first"
        if self.slam.scans < 3:
            return "the lidar has not produced a complete scan yet"
        age = self.scan_age()
        if age is None or age > LIDAR_STALE_S:
            # Refusing is the whole point. The scan matcher keeps its last revolution
            # for ever, so without this check every query below would answer from a
            # picture of the room that may be minutes old and the rover would drive
            # into whatever has changed since.
            return ("the lidar has stopped reporting"
                    + (f" ({age:.0f}s ago)" if age else "")
                    + ", so the rover has no current picture of what is around it")
        if kind in ("turn", "goto"):
            # Turning is never refused, however close anything is. Going to a place
            # that is not straight ahead starts with a turn, so refusing goto for a
            # blocked nose would refuse every destination that is not already in
            # front of a clear run. Turning is also the only move a wedged rover
            # has left -- taking it away is what turned "too close to drive" into
            # "stuck for good".
            return None
        chosen, clear = self._choose_heading(0.0)
        if self._speed_limit(clear) <= 0.0:
            return (f"the way ahead is {clear:.2f} m and the rover keeps "
                    f"{STANDOFF_M:.2f} m from anything it can see")
        return None

    def _step_drive(self, goal, pose, now):
        sx, sy, _ = goal["start_pose"]
        goal["travelled"] = math.hypot(pose[0] - sx, pose[1] - sy)
        goal["turned"] = self._heading_change(goal["start_accum"])

        if goal["distance"] is not None and goal["travelled"] >= goal["distance"]:
            goal["done"] = ("arrived", "")
            return
        if now >= goal["deadline"]:
            goal["done"] = ("timed out", "the move ran out of its time budget")
            return

        chosen, clear = self._choose_heading(0.0)
        self._chosen_deg, self._clearance = chosen, clear

        limit = self._speed_limit(clear)
        if limit <= 0.0:
            goal["done"] = ("blocked",
                            f"the way ahead is {clear:.2f} m and the rover keeps "
                            f"{STANDOFF_M:.2f} m from anything it can see")
            return

        unknown = self._unknown_ahead()
        if unknown:
            # Nothing came back from straight ahead. That is a matt black or glass
            # surface as readily as it is open space, so crawl rather than commit.
            limit = min(limit, CRAWL_SPEED_MS)

        speed = min(goal["speed"], limit)
        # Turn towards the chosen heading; the divisor is how many seconds it should
        # take to get the nose there.
        cap = self._turn_limit()
        turn = _clamp(chosen / 0.8, -cap, cap)
        self._drive_pwm(speed, turn)

    def _step_goto(self, goal, pose, now):
        """Follow a planned polyline by looking ahead along it.

        Progress is the closest point on the path, allowed to slide back a little
        so a weave beside the line is not a rewind. The carrot stays on the current
        segment: looking past a vertex onto the next leg is how a route that gave a
        corner room still drove the chord and arrived at the brake distance. At a
        gentle corner it does run on along the line of the leg being driven once
        the vertex is too near to steer at, which is a different thing -- see
        planner.carrot_at. A sharp corner is a turn on the spot, which is the move
        this rover already has; the rest is the same follow-the-gap drive as a
        straight move, aimed at the carrot. Replan rather than fight when the line
        is blocked, the rover has left the corridor, or the map has grown a wall on
        the remaining route.
        """
        path = goal["path"]
        tx, ty = goal["target"]
        x, y, th = pose
        goal["turned"] = self._heading_change(goal["start_accum"])
        remaining = math.hypot(tx - x, ty - y)

        if remaining <= GOTO_ARRIVE_M:
            goal["travelled"] = (goal.get("travelled_before", 0.0)
                                 + goal.get("progress", 0.0))
            goal["done"] = ("arrived", "")
            return
        if now >= goal["deadline"]:
            goal["travelled"] = (goal.get("travelled_before", 0.0)
                                 + goal.get("progress", 0.0))
            goal["done"] = ("timed out", "the move ran out of its time budget")
            return

        s, cross = planner.project(path, (x, y), goal["progress"], GOTO_SLACK_M)
        goal["progress"] = s
        goal["cross"] = cross
        goal["travelled"] = goal.get("travelled_before", 0.0) + s
        if cross > GOTO_CORRIDOR_M and self._replan_could_differ(goal, now):
            goal["done"] = ("replan",
                            f"drifted {cross:.2f} m off the route, so planning "
                            f"again from here")
            self._drive_pwm(0.0, 0.0)
            return

        if now - goal.get("last_check", now) >= GOTO_RECHECK_S:
            goal["last_check"] = now
            if (self._route_blocked_on_map(path, s)
                    and self._replan_could_differ(goal, now)):
                goal["done"] = ("replan",
                                "the map along the remaining route is no longer "
                                "clear")
                self._drive_pwm(0.0, 0.0)
                return

        carrot = planner.carrot_at(path, s, GOTO_LOOKAHEAD_M, GOTO_CARROT_MIN_M,
                                   GOTO_TURN_DEG)
        want = math.degrees(math.atan2(carrot[1] - y, carrot[0] - x) - th)
        want = (want + 180.0) % 360.0 - 180.0

        # A sharp corner is a turn, not a curve. Turning-over-the-move is how
        # the matcher used to lose the room; stopping and spinning is the move
        # this rover already has.
        if abs(want) > GOTO_TURN_DEG:
            cap = self._turn_limit()
            turn = _clamp(want / 0.6, -cap, cap)
            self._drive_pwm(0.0, turn)
            self._chosen_deg, self._clearance = want, None
            return

        chosen, clear = self._choose_heading(want)
        self._chosen_deg, self._clearance = chosen, clear
        limit = self._speed_limit(clear)
        if limit <= 0.0:
            # Blocked is not a reason to replan. The planner reads the pose and the
            # map, and a rover that has stopped has changed neither, so it draws the
            # same route and the loop refuses it again on the next revolution.
            # Turning is the move that changes something -- and it is the move a
            # cornered rover always still has, which is why turn_in_place refuses
            # nothing. So rotate towards whatever heading has the most room and let
            # the next revolution look again, giving up only once that has been
            # tried for a while and found nothing.
            stuck_since = goal.setdefault("stuck_since", now)
            if now - stuck_since < GOTO_UNSTICK_S:
                best, best_clear = self._roomiest()
                self._chosen_deg, self._clearance = best, best_clear
                cap = self._turn_limit()
                self._drive_pwm(0.0, _clamp(best / 0.6, -cap, cap))
                return
            if self._replan_could_differ(goal, now):
                goal["done"] = ("replan",
                                f"only {clear:.2f} m of room and turning has not "
                                f"found more, so planning again from here")
            else:
                goal["done"] = ("blocked",
                                f"the way on is {clear:.2f} m, the rover keeps "
                                f"{STANDOFF_M:.2f} m from anything it can see, and "
                                f"nothing it can turn to is clearer")
            self._drive_pwm(0.0, 0.0)
            return
        goal.pop("stuck_since", None)

        if self._unknown_ahead():
            limit = min(limit, CRAWL_SPEED_MS)
        if abs(want) > GOTO_ALIGN_DEG:
            limit = min(limit, CRAWL_SPEED_MS)
        speed = min(goal["speed"], limit)
        cap = self._turn_limit()
        turn = _clamp(chosen / 0.8, -cap, cap)
        self._drive_pwm(speed, turn)

    def _replan_could_differ(self, goal, now):
        """Whether asking the planner again could possibly come back with anything new.

        It plans from the pose, on the map. A rover that has not moved since the last
        route was drawn, over a map that has not been rebuilt under it, gets handed
        the same polyline and refuses it again one revolution later -- which is how
        eight replans and a "gave up" used to fit inside nine tenths of a second,
        with the rover standing still throughout and the caller told it had been
        tried eight times.

        **Heading counts as having moved.** It did not use to, and it did not need
        to: when this guard was written the planner took a position and nothing
        else, so a rover that had only turned really would get the same polyline
        back. Then `start_yaw` became an input -- a heading that looks into the
        keep-out starts the route with a hop off it -- and this went on reading
        position alone. The cost of that lands exactly where it hurts: the move
        that gets a cornered rover out is a turn on the spot, `_step_goto` spends
        GOTO_UNSTICK_S doing precisely that, and then asks here whether to replan.
        Turning changes no coordinate, so the answer was no, and the rover reported
        itself blocked from a heading it had just spent two and a half seconds
        changing. What it should do -- what a person would do -- is turn until it
        is pointing somewhere that has room, and ask again from there.

        The guard against replan storms is still the clock. Each replan restarts
        the leg, so GOTO_REPLAN_MIN_S puts a second between them however much the
        rover has turned, and eight of them can no longer fit in under a second.
        """
        if now - goal["started_at"] < GOTO_REPLAN_MIN_S:
            return False
        sx, sy, sth = goal["start_pose"]
        x, y, th = self.slam.pose
        if math.hypot(x - sx, y - sy) >= GOTO_REPLAN_MIN_MOVE_M:
            return True
        turned = abs(math.degrees((th - sth + math.pi) % (2 * math.pi) - math.pi))
        return turned >= GOTO_REPLAN_MIN_TURN_DEG

    def _roomiest(self):
        """The heading with the most room anywhere the rover can turn to.

        _choose_heading searches either side of where the route wants to go, which is
        the right question while the route is drivable and the wrong one once there
        is no room in that direction at all: the way out is wherever it happens to
        be, not near the way in.
        """
        best, best_score, best_clear = 0.0, -1e9, 0.0
        for heading in range(-90, 91, 5):
            curvature = 2.0 * math.sin(math.radians(heading)) / LOOKAHEAD_M
            roomy = min(self._headroom(curvature, comfort=True), LOOKAHEAD_M)
            tight = min(self._headroom(curvature), LOOKAHEAD_M)
            score = roomy + 0.3 * tight - 0.004 * abs(heading)
            if score > best_score:
                best, best_score, best_clear = float(heading), score, tight
        return best, best_clear

    def _route_blocked_on_map(self, path, progress):
        """True if the remaining polyline now sits on a solid cell."""
        import numpy as np

        with self.slam.lock:
            grid = np.array(self.slam.grid(), copy=True)
            res = self.slam.config.resolution_m
            occupied_at = self.slam.config.occupied_at
        return planner.cells_occupied_along(grid, res, occupied_at, path, progress)

    def _drive_pwm(self, speed_ms, turn_dps):
        """Wanted speed and turn rate -> the PWM pair, closing what loop it can.

        With no encoders the only feedback is the scan matcher, so the speed loop is
        a single scale factor nudged by the error. It is deliberately slow and
        tightly clamped: at 10 Hz against a match that resolves a couple of
        centimetres, anything eager oscillates.
        """
        with self._lock:
            self._want_speed, self._want_turn = speed_ms, turn_dps

        if speed_ms > 0.01:
            error = speed_ms - self._measured_speed
            self._pwm_scale = _clamp(self._pwm_scale + 0.05 * error / MAX_SPEED_MS,
                                     0.6, 1.8)
        throttle = _clamp(speed_ms / MAX_SPEED_MS * self._pwm_scale, 0.0, 1.0)

        # Ramp the commanded rate rather than stepping to it, so a turn does not
        # begin with a lurch past the matcher's window.
        now = time.monotonic()
        step = TURN_RAMP_DPS_PER_S * max(0.0, min(0.5, now - self._last_send_at))
        if turn_dps > self._commanded_turn:
            self._commanded_turn = min(turn_dps, self._commanded_turn + step)
        else:
            self._commanded_turn = max(turn_dps, self._commanded_turn - step)
        wanted = self._commanded_turn

        # Servo the rate on what the matcher measures, because the open-loop map from
        # PWM to degrees a second was never calibrated and was wrong by enough to
        # overshoot every turn by about 40%. The feedback converges from above even
        # when the measurement is saturating: a saturated reading is still far higher
        # than the target, so the correction is still downwards.
        if abs(wanted) > 1.0:
            error = abs(wanted) - abs(self._measured_turn)
            self._turn_scale = _clamp(self._turn_scale + 0.06 * error / MAX_TURN_DPS,
                                      0.30, 2.0)
        magnitude = _clamp(abs(wanted) / MAX_TURN_DPS * self._turn_scale, 0.0, 1.0)
        steer = -math.copysign(magnitude, wanted) if abs(wanted) > 1e-6 else 0.0
        # Positive steer turns right in the firmware's terms (left = throttle +
        # steer), and this module's turn rate is counter-clockwise positive, hence
        # the sign flip above.

        # Equal PWM is not equal speed on this chassis, so hold a straight line by
        # trimming on the turn rate the matcher actually sees.
        if abs(turn_dps) < 2.0 and speed_ms > 0.05:
            self._trim = _clamp(self._trim - 0.004 * self._measured_turn, -0.25, 0.25)
            steer -= self._trim

        left, right = throttle + steer, throttle - steer
        peak = max(abs(left), abs(right))
        if peak > 1.0:
            left, right = left / peak, right / peak
        self._send(self._to_pwm(left), self._to_pwm(right))

    @staticmethod
    def _to_pwm(value):
        if abs(value) < 1e-3:
            return 0
        magnitude = MIN_PWM + abs(value) * (TOP_PWM - MIN_PWM)
        return int(round(magnitude if value > 0 else -magnitude))

    def _send(self, left, right):
        self.link.send({"T": CMD_PWM, "L": left, "R": right})
        self._last_sent = (left, right)
        self._last_send_at = time.monotonic()

    # --- reporting ------------------------------------------------------------
    def status(self, since_seq=None):
        """`since_seq` is passed straight to the move commentary: give it the last
        sentence you saw and the reply carries anything said since. See
        MoveReport.snapshot."""
        x, y, th = self.slam.pose
        remaining = None
        with self._lock:
            goal = self._goal
            if goal is not None and goal.get("kind") == "goto" and goal.get("path"):
                remaining = max(0.0, planner.length(goal["path"])
                                - goal.get("progress", 0.0))
        return {
            "driving": self._driving,
            "estop": self._estop,
            "pose": {"x_m": round(x, 3), "y_m": round(y, 3),
                     "heading_deg": round(math.degrees(th), 1)},
            "speed_ms": round(self._measured_speed, 3),
            "turn_dps": round(self._measured_turn, 1),
            "clearance_m": None if self._clearance is None
                           else round(self._clearance, 2),
            "steering_deg": round(self._chosen_deg, 1),
            "remaining_m": None if remaining is None else round(remaining, 2),
            # What the move says it is doing, as opposed to what the wheels are
            # doing. The numbers above are the state of the rover; this is the state
            # of the request, and a plan being refused shows up here and nowhere
            # else until the call itself returns. See MoveReport.
            "move": self.report.snapshot(since_seq),
            "match_score": round(self.slam.score, 3),
            "position_trusted": not self.slam.rejected,
            # How the match was won, which the score alone does not say: a winner
            # against the rim of the search window means the answer was probably
            # outside it, and a rival near 1.0 means some other heading fitted just
            # as well. Both read false and 0.0 on a healthy revolution.
            "match_edge": self._health.get("edge", False),
            "heading_ambiguity": self._health.get("ambiguity", 0.0),
            # False while the pose is not trusted enough to write the map from. The
            # rover goes on driving and avoiding things -- that reads the live scan,
            # never the map -- but the map stops growing until two revolutions have
            # agreed on where it is. A room with two answers still resumes, on the
            # heading the matcher kept picking.
            "mapping": not self._map_paused,
            "slam_events": [dict(e, age_s=round(time.monotonic() - e["at"], 1))
                            for e in self._events[-6:]],
            # The gyro and the wheels: what the board's own sensors are being
            # believed for. `witness` says whether the gyro yet has a resting
            # threshold to judge rotation by -- it learns one within a few seconds
            # of standing still -- and `prior` whether either scale factor has been
            # measured well enough to centre the search window with.
            "odometry": self._odom.status(),
            "scans": self._scans,
            "dropped_scans": self._dropped,
            # Running totals since the daemon started, which is what makes them
            # useful: a rate is what says whether the search window is the right
            # size for how fast the rover is being driven, and one revolution's
            # `match_edge` above cannot. An overrun is a pose that came back short
            # because the rover moved further than one revolution's search covers;
            # a rejection is a scan that fitted the map nowhere at all.
            "window_overruns": self._edges,
            "rejected_matches": self._rejects,
            "pwm": self._last_sent,
            "lidar_ok": self.lidar_ok(),
            # Both, because they disagree during a dead-reckoned turn and the pair is
            # what tells a healthy suspended map apart from a sensor that has died.
            "lidar_live": self.lidar_live(),
            "lidar_port": self.lidar_path,
            "lidar_usb": self._lidar_usb,
            # How many times this rover has had to reset its own sensor, and what
            # came of the last one. Reported rather than merely logged because a
            # count that climbs over an afternoon is a cable working loose, and the
            # console is where somebody would notice.
            "lidar_resets": self._resets,
            "lidar_reset_note": self._reset_note,
            "scan_age_s": None if self.scan_age() is None
                          else round(self.scan_age(), 2),
        }

    def describe(self):
        out = self.slam.describe()
        out["driving"] = self._driving
        out["estop"] = self._estop
        age = self.scan_age()
        out["lidar_ok"] = self.lidar_ok()
        out["scan_age_s"] = None if age is None else round(age, 2)
        if not out["lidar_ok"]:
            # Said first and said plainly, because everything after it is a
            # description of a room that may no longer be there.
            out["text"] = ("The lidar is not reporting, so nothing here is current "
                           "and the rover will not drive. " + out["text"])
        return out

    def clear_map(self):
        """Throw the map away and start again from where the rover is standing.

        There is no loop closure here and never will be -- see slam2d.h -- so drift
        is permanent: a room that has come out a few degrees out of true with itself,
        or a corridor stamped in twice from two passes, will stay that way for as
        long as the daemon runs. Once that has happened the map is worse than no map,
        because the planner routes on it and refuses gaps that are really there. An
        empty map is at least true, and this rover fills one back in within a
        revolution or two of standing still.

        Refused while a move is running, and refused rather than queued. The route a
        move is following is a list of places in the very frame this is about to
        throw away, so clearing underneath one would have the rover drive to
        coordinates that no longer mean anything -- and it is holding the lidar's
        own picture of the room at the time. Stop first; stopping is never refused.
        """
        if not self._move_mutex.acquire(blocking=False):
            return {"cleared": False,
                    "reason": "the rover is moving, and the route it is following is "
                              "in the frame this would throw away -- stop it first"}
        try:
            with self.slam.lock:
                self.slam.reset()
            # The track is where the rover has been in the old frame, so drawing it
            # over the new map would put an invented history across an empty room.
            self._trail = []
            # Speed and turn rate are differences between one pose and the next, and
            # the pose has just moved without the rover moving. Re-seed rather than
            # measure the jump, which would otherwise be reported as several metres
            # a second for one revolution and would reach the speed loop as fact.
            self._last_pose = None
            self._last_at = None
            self._measured_speed = 0.0
            self._measured_turn = 0.0
            # A map asked for from scratch is one to write, so whatever hold a bad
            # turn left behind goes with the map it was protecting. slam2d_reset
            # does the same on its side; this is the flag that mirrors it.
            self._map_paused = False
            self._good_run = 0
            self._confirm_pose = None
            self._need_recovery = False
            self._wide_recovery = False
            self._lost_run = 0
            self._hold_confirm = False
            self._log_event("map resumed", "the map was cleared and rebuilt")
            return {"cleared": True,
                    "reason": "the map is empty and the rover is at its origin"}
        finally:
            self._move_mutex.release()

    def map_png(self, half_extent_m=3.0, scale=3, rover_up=False, camera=None):
        """`camera` is `(bearing_deg, fov_deg)` for the gimbal's cone, or None.

        Passed straight through and not worked out here, because this owns the lidar
        and knows nothing whatever about the camera -- the daemon owns that, and it
        is the daemon that has to turn a pan into a bearing in this frame.
        """
        import mapimg
        return mapimg.render(self.slam, half_extent_m, scale, tuple(self._trail),
                             rover_up=rover_up, camera=camera)


def _selftest():
    """The commentary, which is the one thing in this file that can be checked
    without a lidar, a driver board or a floor.

    Worth checking on its own because the failure mode is quiet: a report that
    keeps a stale field, or that fails to move its counter, does not break a move
    -- it makes the window watching one describe something that is not happening.
    """
    report = MoveReport()
    assert report.snapshot()["phase"] == "idle", "a fresh report claims a move"

    origin = (0.0, 0.0, 0.0)
    assert _pose_close(origin, (0.01, 0.0, math.radians(1))), "a centimetre is the same pose"
    assert not _pose_close(origin, (0.0, 0.0, math.radians(20))), "twenty degrees is not"
    assert _pose_close((0.0, 0.0, math.pi - 0.02), (0.0, 0.0, -math.pi + 0.02)), \
        "heading wrap is still the same pose"

    report.begin("drive_to", {"ahead_m": 1.2, "left_m": -0.4}, "planning")
    first = report.snapshot()
    assert first["phase"] == "planning" and first["kind"] == "drive_to", first
    assert first["asked"] == {"ahead_m": 1.2, "left_m": -0.4}, first

    report.say("driving", route_m=1.86, waypoints=4, replans=0)
    accepted = report.snapshot()
    assert accepted["seq"] > first["seq"], "the counter did not move"
    assert accepted["route_m"] == 1.86 and accepted["waypoints"] == 4, accepted
    assert accepted["asked"] == first["asked"], "the request was forgotten mid-move"

    # A reason belongs to the phase that gave it. Left lying around it becomes a
    # claim about the next phase, which is how a route that planned cleanly ends up
    # captioned with the drift that provoked the replan before it.
    report.say("replanning", "drifted 0.61 m off the route", replans=1,
               route_m=None, waypoints=None)
    assert report.snapshot()["route_m"] is None, "the old route outlived the replan"
    report.say("driving", route_m=1.2, waypoints=3, replans=1)
    assert report.snapshot()["why"] == "", "the replan's reason outlived the replan"

    report.finish("arrived", "")
    ended = report.snapshot()
    assert ended["phase"] == "ended" and ended["reason"] == "arrived", ended
    assert ended["replans"] == 1, "the replans were not counted"
    assert ended["missed"] == [], "asked for no history and got some anyway"

    # A watcher that blinked. Everything said between the sentence it last saw and
    # the one being said now comes back with it, oldest first -- a replan lasts
    # about as long as the planner takes and is easily shorter than a poll.
    caught_up = report.snapshot(since_seq=first["seq"])
    phases = [state["phase"] for state in caught_up["missed"]]
    assert phases == ["driving", "replanning", "driving"], phases
    assert [state["seq"] for state in caught_up["missed"]] == sorted(
        state["seq"] for state in caught_up["missed"]), "history is out of order"
    assert caught_up["missed"][1]["why"].startswith("drifted"), (
        "the history lost the reason with the phase it belonged to")
    assert report.snapshot(since_seq=ended["seq"])["missed"] == [], (
        "a caller already up to date was handed history anyway")

    # A new move starts clean, except for the counter -- which must never go
    # backwards, or a poller decides it has already seen what it is looking at.
    report.begin("turn_in_place", {"angle_deg": -90.0}, "turning")
    fresh = report.snapshot()
    assert fresh["seq"] > ended["seq"], "the counter went backwards"
    assert fresh["reason"] is None and fresh["replans"] == 0, fresh
    assert fresh["route_m"] is None and fresh["why"] == "", fresh

    # A rover that has spent GOTO_UNSTICK_S turning on the spot has changed the
    # one thing the planner reads besides the map, so asking again can come back
    # with something new. Reading position alone is how a turn-to-get-free ended
    # in "blocked" instead of a route -- see _replan_could_differ.
    class _Standing:
        """Just enough Navigator to ask the replan question of."""
        def __init__(self, pose):
            self.slam = type("S", (), {"pose": pose})()

    goal = {"start_pose": (1.0, 2.0, 0.0), "started_at": 0.0}
    at_once = _Standing((1.0, 2.0, 0.0))
    assert not Navigator._replan_could_differ(at_once, goal, 0.5), (
        "replanned before the leg was a second old")
    assert not Navigator._replan_could_differ(at_once, goal, 2.0), (
        "replanned having neither moved nor turned")

    shuffled = _Standing((1.15, 2.0, 0.0))
    assert Navigator._replan_could_differ(shuffled, goal, 2.0), (
        "15 cm of travel is a new question and was refused")

    turned = _Standing((1.0, 2.0, math.radians(35)))
    assert Navigator._replan_could_differ(turned, goal, 2.0), (
        "the rover turned 35 degrees on the spot and was still told nothing "
        "could have changed -- this is the blocked-instead-of-turning bug")

    nudged = _Standing((1.0, 2.0, math.radians(5)))
    assert not Navigator._replan_could_differ(nudged, goal, 2.0), (
        "five degrees of pose wobble is not a turn and must not spend a replan")

    wrapped = _Standing((1.0, 2.0, math.radians(-179)))
    goal_near_pi = {"start_pose": (1.0, 2.0, math.radians(179)), "started_at": 0.0}
    assert not Navigator._replan_could_differ(wrapped, goal_near_pi, 2.0), (
        "two degrees across the heading wrap read as most of a revolution")

    # The turn cap is a rotation the matcher can follow, expressed as a rate --
    # so it has to move with the interval the loop is actually delivering. The
    # recordings that motivated it measured 138 ms at the median and 236 at the
    # ninetieth percentile, against a coarse window of 3 deg x 3 steps.
    class _Paced:
        """Just enough Navigator to ask what turn rate it would allow."""
        def __init__(self, gap):
            self._match_gap = gap
            self.slam = type("S", (), {"config": type("C", (), {
                "coarse_ang_deg": 3.0, "coarse_ang_steps": 3})()})()

    window = 3.0 * 3
    nominal = Navigator._turn_limit(_Paced(0.100))
    assert abs(nominal - MAX_TURN_DPS) < 1e-6, (
        f"at the sensor's own 10 Hz this must come out at the limit that was "
        f"there before, and it came out {nominal:.1f}")

    at_median = Navigator._turn_limit(_Paced(0.138))
    at_p90 = Navigator._turn_limit(_Paced(0.236))
    assert at_median > at_p90, "a slower loop must not permit a faster turn"
    assert at_p90 * 0.236 <= window + 1e-6, (
        f"{at_p90:.0f} deg/s over a 236 ms gap is {at_p90 * 0.236:.1f} deg, past "
        f"the {window:.0f} the coarse pass can search")

    # Never above the PWM ceiling, and never so low that turning stops being a
    # move -- a cornered rover has nothing else left.
    assert Navigator._turn_limit(_Paced(0.001)) == MAX_TURN_DPS
    assert Navigator._turn_limit(_Paced(9.9)) == MIN_TURN_DPS, (
        "a loop that has stopped must still leave the rover able to turn")

    # Losing the pose while driving is not the same accident as losing it to a
    # dead-reckoned turn, and the search that finds it again is not the same
    # search. The sweep gives up translation to buy heading -- +/-5 cm against the
    # tracking window's +/-10 -- which suits a rover standing still and is less
    # than a driving one covers in a revolution. Asking for it there is what turned
    # one bad revolution into fifteen seconds of held map.
    from odometry import _Board

    class _Tracking:
        """Just enough Navigator to run the map-hold state machine on scripted
        revolutions, without a lidar or a floor."""

        def __init__(self):
            self._map_paused = False
            self._need_recovery = False
            self._wide_recovery = False
            self._lost_run = 0
            self._good_run = 0
            self._confirm_pose = None
            self._hold_confirm = False
            self._health = {}
            self._events = []
            self._dropped = 0
            self._journey = None
            self.health = {}
            # A blind odometry by default: no source, so the witness has nothing to
            # say and every assertion below is about the matcher alone, the way it
            # was before the gyro was read at all.
            self._odom = Odometry(object(), load=False)
            self._span = None
            self._last_pose = None
            self._drive_mark = None
            self._drive_marked_at = None
            self._rejects = 0
            self._edges = 0
            self._path_m = 0.0
            self.slam = type("S", (), {"lock": threading.Lock(),
                                       "mapping": True})()

        def _match_health(self):
            return dict(self.health)

        _log_event = Navigator._log_event
        _pause_mapping = Navigator._pause_mapping
        _resume_mapping = Navigator._resume_mapping
        _note_match = Navigator._note_match
        _witness = Navigator._witness
        _calibrate_turn = Navigator._calibrate_turn
        _calibrate_drive = Navigator._calibrate_drive

    def _rev(nav, ok, pose=(0.0, 0.0, 0.0)):
        nav.health = {"score": 0.9 if ok else 0.9, "edge": 0 if ok else 1,
                      "ambiguity": 0.0, "rejected": False, "pose": pose,
                      "recovery": False, "map_ok": ok}
        nav._note_match()

    nav = _Tracking()
    _rev(nav, True)
    assert not nav._map_paused, "a healthy revolution held the map"

    _rev(nav, False)
    assert nav._map_paused and nav._need_recovery, "a rim hit did not hold the map"
    assert not nav._wide_recovery, (
        "a rim hit while driving asked for the +/-60 degree sweep, whose "
        "translation window is half the one the rover just outran")

    _rev(nav, False)
    assert not nav._wide_recovery, (
        "two bad revolutions is not evidence that the ordinary window cannot "
        "find the pose, and widening that soon is the old behaviour under a "
        "new name")
    for _ in range(WIDEN_AFTER_LOST - 2):
        _rev(nav, False)
    assert not nav._wide_recovery, (
        f"widened after fewer than {WIDEN_AFTER_LOST} revolutions of tracking")
    _rev(nav, False)
    assert nav._wide_recovery, (
        f"tracking failed {WIDEN_AFTER_LOST} revolutions running and the search "
        f"never widened, so a genuinely lost rover would stay lost")
    assert any(e["what"] == "searching wide" for e in nav._events), (
        "the search widened without saying so")

    # Two agreeing healthy revolutions put it back, and put the sweep away with it.
    _rev(nav, True, (1.0, 0.0, 0.0))
    _rev(nav, True, (1.0, 0.0, 0.0))
    assert not nav._map_paused, "an agreeing pair did not resume the map"
    assert not nav._wide_recovery and nav._lost_run == 0, (
        "the wide search outlived the hold it was for")

    # A revolution the tracking window did find is evidence that it can, so the
    # count starts again. Without that, a rover matching every other revolution
    # accumulates its way to a sweep it never needed.
    patchy = _Tracking()
    _rev(patchy, False)
    for _ in range(WIDEN_AFTER_LOST - 2):
        _rev(patchy, False)
    _rev(patchy, True, (2.0, 0.0, 0.0))
    for _ in range(WIDEN_AFTER_LOST - 1):
        _rev(patchy, False)
    assert not patchy._wide_recovery, (
        "the failures either side of a revolution that matched were added "
        "together, so an intermittent match widens the search on its own")

    # The burst path is the one the sweep was built for and must still get it at
    # once: after a dead-reckoned turn the heading can be tens of degrees out, and
    # the tracking window spans nine.
    after_burst = _Tracking()
    after_burst._pause_mapping("a turn was dead reckoned")
    assert after_burst._wide_recovery, (
        "a dead-reckoned turn was left to find itself with the tracking window")

    # --- the gyro contradicting the matcher ---------------------------------
    # Everything above judges a match by the search that produced it, which cannot
    # see the failure that matters most: a scan snapped onto a wrong alignment
    # scores high, because scoring high is why it won. These revolutions are all
    # healthy by every measure the matcher has. The only thing wrong with them is
    # that the chassis did not move.
    def _witnessed(board):
        nav = _Tracking()
        nav._odom = Odometry(board, load=False)
        nav._odom.reset()
        for _ in range(80):                    # a few seconds of standing still
            board.advance(0.1, noise=1.5)
            nav._odom.learn_rest(nav._odom.span())
        assert nav._odom.rest_known, "the resting gyro never produced a threshold"
        return nav

    def _witness_rev(nav, board, pose, seconds=0.1, dps=0.0):
        nav._last_pose = nav.health.get("pose") if nav.health else nav._last_pose
        board.advance(seconds, dps=dps, noise=1.5)
        nav._span = nav._odom.span()
        _rev(nav, True, pose)

    board = _Board()
    caught = _witnessed(board)
    _witness_rev(caught, board, (0.0, 0.0, 0.0))
    assert not caught._map_paused, "a healthy revolution held the map"
    # The matcher swings the heading 20 degrees. The gyro sat still throughout.
    _witness_rev(caught, board, (0.0, 0.0, math.radians(20.0)))
    assert caught._map_paused, (
        "the matcher moved the heading 20 degrees over a chassis the gyro says "
        "never turned, and the map went on being written from it -- which is the "
        "whole mechanism behind a room stamped in twice at an angle")
    assert any("gyro" in e["why"] for e in caught._events), (
        "the map was held without saying the gyro was what disagreed")

    # And the other way round, which matters just as much: a rover that really is
    # turning must not have its map held every revolution for doing so.
    honest = _witnessed(_Board())
    board2 = honest._odom.source
    heading = 0.0
    for _ in range(10):
        heading += math.radians(4.0)
        _witness_rev(honest, board2, (0.0, 0.0, heading), dps=40.0)
    assert not honest._map_paused, (
        "a rover turning 40 degrees a second, with the gyro agreeing that it was, "
        "had its map held anyway")

    # A gyro with no threshold yet says "unknown", and unknown must not read as
    # "the chassis was still" -- that would manufacture the very disagreement this
    # exists to detect, on every revolution, from a cold start.
    cold = _Tracking()
    cold._odom = Odometry(_Board(), load=False)
    cold._odom.reset()
    cold._odom.source.advance(0.1)
    cold._span = cold._odom.span()
    cold._last_pose = (0.0, 0.0, 0.0)
    _rev(cold, True, (0.0, 0.0, math.radians(30.0)))
    assert not cold._map_paused, (
        "a gyro that has not yet learnt what rest looks like was allowed to "
        "contradict the matcher, so every cold start holds the map")

    # --- calibrating out of moves the rover made anyway ---------------------
    # Against a real Outcome, which is the point of this one: the first version
    # read `outcome.travelled` and `outcome.turned`, and the object calls them
    # `travelled_m` and `turned_deg`. Nothing offline noticed, because nothing
    # offline built one -- the rover found it, on the floor, at the end of a drive
    # that had already happened.
    calibrating = _Tracking()
    board = _Board()
    import tempfile
    store = os.path.join(tempfile.mkdtemp(), "odometry.json")
    calibrating._odom = Odometry(board, store=store, load=False)
    calibrating._odom.reset()
    for _ in range(80):
        board.advance(0.1, noise=1.5)
        calibrating._odom.learn_rest(calibrating._odom.span())

    for degrees in (90.0, -90.0, 180.0):
        mark = calibrating._odom.mark()
        board.advance(abs(degrees) / 60.0, dps=60.0 * (1 if degrees > 0 else -1))
        calibrating._calibrate_turn(degrees, mark)
    measured = calibrating._odom.gyro_lsb_per_dps
    assert measured is not None, "three confirmed turns measured no gyro scale"
    assert abs(measured - board.lsb_per_dps) < 0.5, (
        f"the gyro scale came out {measured} against a board built at "
        f"{board.lsb_per_dps}")

    def _drove(nav, metres, turned=2.0, reason="arrived", rejects=0, edges=0):
        """A drive of `metres` along the path, with the board rolling to match.

        `rejects` and `edges` happen *during* the drive, which is the only place
        they mean anything: bumping them before the mark is taken leaves the
        difference at zero and tests nothing, which is how the first version of
        these two assertions passed without exercising either gate.
        """
        nav._drive_mark = nav._odom.mark()
        nav._drive_marked_at = (nav._rejects, nav._edges, nav._path_m)
        nav._odom.source.advance(metres / 0.25, ms=0.25)
        nav._path_m += metres
        nav._rejects += rejects
        nav._edges += edges
        # The straight-line figure is deliberately shorter than the path, which is
        # the whole point: a wandering drive rolls more wheel than it displaces.
        nav._calibrate_drive(Outcome(reason, metres * 0.95, turned))

    for metres in (0.5, 1.0, 0.8):
        _drove(calibrating, metres)
    ticks = calibrating._odom.ticks_per_metre
    assert ticks is not None, "three confirmed drives measured no wheel scale"
    assert abs(ticks - board.ticks_per_metre) < 20.0, (
        f"the wheel scale came out {ticks} against a board built at "
        f"{board.ticks_per_metre}")

    # A drive that ended with the pose against the rim of the search window is
    # still a measurement. That bar had to be found on the floor: gating on the map
    # being written refused *every* drive, because stopping is exactly when the
    # rover outruns one revolution's search and the map is held for a moment.
    before = calibrating._odom.status()["drives_measured"]
    calibrating._map_paused = True
    _drove(calibrating, 1.0, edges=2)
    assert calibrating._odom.status()["drives_measured"] == before + 1, (
        "a drive that ended on the rim of the window was refused, which refuses "
        "every drive there is")
    calibrating._map_paused = False

    # A rejected revolution is different in kind: the scan fitted nothing anywhere,
    # so the distance the matcher reports for that drive is fiction.
    before = calibrating._odom.status()["drives_measured"]
    _drove(calibrating, 1.0, rejects=1)
    assert calibrating._odom.status()["drives_measured"] == before, (
        "a drive the matcher lost the pose during was fitted to the wheel scale")

    # A wander of twenty-odd degrees is now measured rather than refused, because
    # the path is what the wheels rolled. Only a drive that has swung right round,
    # or one that never arrived, has nothing to say.
    before = calibrating._odom.status()["drives_measured"]
    _drove(calibrating, 1.0, turned=25.0)
    assert calibrating._odom.status()["drives_measured"] == before + 1, (
        "a drive that wandered 25 degrees was refused, and this chassis wanders")
    before = calibrating._odom.status()["drives_measured"]
    _drove(calibrating, 1.0, turned=120.0)
    _drove(calibrating, 1.0, reason="blocked")
    assert calibrating._odom.status()["drives_measured"] == before, (
        "a drive that swung right round, or never arrived, was fitted anyway")

    # --- getting a silent lidar back ----------------------------------------
    #
    # The ladder is the part worth checking without hardware, because every rung of
    # it is a decision about how big an act to take and the biggest one takes the
    # camera down with it. What cannot be checked here is whether the reset works --
    # that is a property of the bus, and it was measured on the rover: a hub reset
    # brought a lidar that had been gone for sixteen minutes back in four seconds.
    issued = []

    class _Blind:
        """Just enough Navigator to ask what it would do about a quiet sensor."""

        def __init__(self, quiet_s, port=True, driving=False, suspended=False):
            self._driving, self._suspend_slam = driving, suspended
            self.lidar = object() if port else None
            self._last_packet_at = time.monotonic() - quiet_s
            self._lidar_watch_from = self._last_packet_at
            self._lidar_usb = "1-1.3.3.2"
            self._reset_at, self._reset_wait = 0.0, LIDAR_RESET_COOLDOWN_S
            self._resets, self._reset_note = 0, ""
            self._reset_rung = 0
            self._reopen_at = 999.0
            self.dropped = 0

        quiet_for = Navigator.quiet_for
        mind = Navigator._mind_the_lidar

        def _drop_lidar(self):
            self.lidar = None
            self.dropped += 1

        def _log_event(self, what, why, **_fields):
            pass

    def _pretend_reset(known="", rung=0, ids=None, rungs=3):
        """A bus with three things to reset, none of which ever helps -- which is
        the case the escalation exists for and the one a real bus cannot be asked
        to reproduce on demand."""
        issued.append((known, rung))
        return usbreset.Attempt(True, f"{known}@{rung}", f"reset {known} rung {rung}",
                                rung=min(rung, rungs - 1), rungs=rungs)

    was, usbreset.revive = usbreset.revive, _pretend_reset
    try:
        talking = _Blind(0.2)
        talking.mind(time.monotonic())
        assert talking.dropped == 0 and not issued, "a sensor that is talking was reset"

        # Rung one: reopen the port. Cheap, and it is the fix for the failure the
        # by-id name exists for -- a handle to an adapter that has re-enumerated.
        stuck = _Blind(LIDAR_SILENT_S + 1)
        stuck.mind(time.monotonic())
        assert stuck.dropped == 1, "a port that went quiet was not reopened"
        assert not issued, "the USB was reset before the port had even been reopened"

        # Rung two: there is no port to reopen, and there has not been for a while.
        gone = _Blind(LIDAR_RESET_AFTER_S + 1, port=False)
        now = time.monotonic()
        gone.mind(now)
        assert issued == [("1-1.3.3.2", 0)], f"no reset was issued: {issued}"
        assert gone._resets == 1 and gone._reset_note, "the reset went unrecorded"
        assert gone._reopen_at == 0.0, (
            "the port was not looked for again straight after the reset, so the "
            "rover waits out a reopen delay while its lidar is already back")

        # Once, not once per pass through the loop. The loop runs thousands of times
        # a second and the device needs seconds to re-enumerate.
        issued.clear()
        gone.mind(now + 0.1)
        assert not issued, "a second reset was issued inside the cooldown"
        assert gone._reset_rung == 1, (
            "the next attempt would reset the same device again, having just been "
            "shown that resetting it did not bring the sensor back")

        # A reset that succeeds and changes nothing is the trap this ladder is built
        # around: the ioctl returns fine against a device that is enumerated but
        # dead, so a recovery that only ever resets the same device would spend the
        # afternoon doing the one thing already shown not to work. Nothing came
        # back, so reach higher -- and only start backing off once there is nothing
        # higher left.
        issued.clear()
        climbing = _Blind(LIDAR_RESET_AFTER_S + 1, port=False)
        at = time.monotonic()
        for _ in range(2):
            climbing.mind(at)
            at += climbing._reset_wait + 0.1
        assert [rung for _where, rung in issued] == [0, 1], (
            f"the recovery did not escalate: {issued}")
        assert climbing._reset_wait == LIDAR_RESET_COOLDOWN_S, (
            "it backed off while it still had something bigger to try, which spends "
            "a quarter of an hour not doing the thing that would have worked")

        # The top rung is the last thing software can do, and that is where waiting
        # longer starts being the right answer rather than a delay.
        climbing.mind(at)
        at += climbing._reset_wait + 0.1
        assert [rung for _where, rung in issued] == [0, 1, 2], issued
        assert climbing._reset_wait > LIDAR_RESET_COOLDOWN_S, (
            "the ladder ran out and it went on trying at the same rate")

        # And then it does back off, rather than knocking the camera out every
        # minute for the rest of the afternoon over a lidar that is unplugged.
        for _ in range(12):
            climbing.mind(at)
            at += climbing._reset_wait + 0.1
        assert climbing._reset_wait == LIDAR_RESET_MAX_COOLDOWN_S, (
            f"the cooldown ran away to {climbing._reset_wait}")
        assert climbing._reset_rung == 2, (
            "the ladder climbed past its own top rung")


        # Never with the wheels turning: the reset takes the camera and the OAK with
        # it, and the watchdog is already stopping the move for the same silence.
        issued.clear()
        moving = _Blind(LIDAR_RESET_AFTER_S + 10, port=False, driving=True)
        moving.mind(time.monotonic())
        assert not issued and moving.dropped == 0, "a move was interrupted to reset USB"

        # Nor during a dead-reckoned turn, where silence is the design rather than a
        # fault -- the map is suspended and the sensor is not being read.
        turning = _Blind(LIDAR_RESET_AFTER_S + 10, port=False, suspended=True)
        turning.mind(time.monotonic())
        assert not issued, "a suspended map was mistaken for a dead sensor"

        # A rover that came up with the lidar already missing has no first packet to
        # measure from, and is exactly the case this is for.
        never = _Blind(LIDAR_RESET_AFTER_S + 1, port=False)
        never._last_packet_at = None
        assert never.quiet_for() > LIDAR_RESET_AFTER_S, (
            "a lidar that has never reported reads as one that never had to")
    finally:
        usbreset.revive = was

    # The name of the device, which is the one thing that cannot be looked up once
    # the device has gone, is remembered from when the port was open.
    assert list(usbreset.parents("1-1.3.3.2")) == ["1-1.3.3", "1-1.3", "1-1"], (
        "the ladder of hubs above the lidar came out wrong")

    print("navigator: ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(_selftest())
