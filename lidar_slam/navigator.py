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

import planner
from slam2d import Slam2D, default_config

LIDAR_BAUD = 230400
# A 10 Hz sensor that has said nothing for a second has missed ten revolutions, and
# whatever it last said is no longer a description of the room. Everything that
# could move the rover checks this first -- the alternative was observed and is
# worse than it sounds: the port vanished under a running daemon, the scan froze,
# and describe_surroundings went on confidently reporting the room as it had been.
LIDAR_STALE_S = 1.0
LIDAR_REOPEN_S = 2.0

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
# But a route allowed to touch that ring hugs it, and a hugged corner is passed
# at exactly the distance the follower brakes at -- an ordinary pose error turns
# a legal route into a stop. So travel inside this distance of anything blocked
# costs extra, fading to nothing at the edge: the route arcs wide of a corner
# whenever there is room, and still takes a narrow gap when there is not,
# because in a squeeze every route pays the toll and the shortest still wins.
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
RESEED_MIN_SCORE = 0.35    # does the scan fit here at all
RESEED_MAX_AMBIGUITY = 0.90  # was there an equally good answer somewhere else
REACQUIRE_GOOD_SCANS = 2   # consecutive healthy matches before the map is trusted again
SLAM_EVENTS = 20           # how much of the recent history of all this to keep

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
MAX_GOTO_S = 75.0
MAX_GOTO_M = 8.0
MAX_REPLANS = 8
GOTO_ARRIVE_M = 0.16       # close enough; the pose is not a millimetre thing
GOTO_CORRIDOR_M = 0.55     # off the polyline by more than this means replan
GOTO_LOOKAHEAD_M = 0.80    # carrot along the path, not the next 5 cm cell
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
UNKNOWN_AHEAD_SECTORS = 3  # +/-30 degrees at 36 sectors
# How many recent revolutions the "is anything touching us" test looks back over.
# One is not enough: a thin or dark object near the sensor's 0.12 m floor comes and
# goes between scans, and testing only the newest let a turn start beside something
# 0.13 m away and run for nearly four seconds before a scan happened to see it
# again. Taking the closest thing seen in the last half second instead means a
# return only has to appear once to be believed.
NEAR_HISTORY = 5


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
        return ("the best fit was against the edge of even the wide search, so the "
                "rover ended up further round than that search could reach")
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
        self._good_run = 0           # consecutive healthy matches while paused
        self._health = {}            # how the last match was won, for status
        self._events = []            # a short history of losing and regaining the pose
        self._trail = []
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
        self.report.begin("drive", {"distance_m": distance_m, "speed_ms": speed_ms},
                          "driving")
        return self._ended(self._drive(distance_m, speed_ms, seconds))

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
        self.report.begin("drive_to", {"ahead_m": round(float(ahead_m), 2),
                                       "left_m": round(float(left_m), 2)},
                          "planning")
        return self._ended(self._drive_to(ahead_m, left_m, speed_ms))

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
        """A polyline from here to `target_xy`, or (None, why)."""
        import numpy as np

        with self.slam.lock:
            grid = np.array(self.slam.grid(), copy=True)
            pose = self.slam.pose
            res = self.slam.config.resolution_m
            occupied_at = self.slam.config.occupied_at
        return planner.plan(grid, res, occupied_at, (pose[0], pose[1]), target_xy,
                            inflate_m=PLAN_INFLATE_M, comfort_m=PLAN_COMFORT_M)

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
        map until a wide search has agreed with it. Until that happens the pose is
        the only thing at risk; the map, which cannot be repaired because there is no
        loop closure, is not.

        Long turns go in bursts of TURN_BURST_MAX_DEG with a measurement between
        them, because a dead-reckoned error is a fraction of the burst it came from
        and a whole 180 guessed in one go can land outside what the search can undo.

        If the lidar is not reporting, the commanded figure is returned and said to
        be dead-reckoned rather than passed off as a measurement.
        """
        self.report.begin("turn_in_place", {"angle_deg": round(float(angle_deg), 1)},
                          "turning")
        return self._ended(self._turn_in_place(angle_deg, speed_dps))

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
                remaining = angle - self._heading_change(start_accum)
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
                return Outcome("lost", 0.0, done,
                               f"the turn was dead reckoned as {done:.0f} degrees but "
                               f"{_why_lost(lost)}, so the rover is not where it "
                               f"thinks it is and its heading is not to be trusted "
                               f"until it sees something it recognises. The map is "
                               f"not being written meanwhile, so nothing is being "
                               f"spoiled by it")

            turned = self._heading_change(start_accum)
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
            self._pause_mapping("a turn was dead reckoned and nothing has "
                                "confirmed where it ended yet")
        finally:
            self._suspend_slam = False
        return turned

    # --- knowing whether the pose is worth believing --------------------------

    def _match_health(self):
        """How the last match was won, and whether that is good enough to build on.

        The score on its own is not enough and never was. A scan that has snapped
        onto the wrong-but-self-consistent alignment scores *high* -- scoring high
        is exactly why that pose beat the others -- so a confident mistake and a
        good fix look identical in it. The other two numbers are what separate them:
        whether the winner sat against the rim of the window, meaning the answer was
        probably outside it, and whether some quite different heading fitted just as
        well. See slam2d.h.
        """
        with self.slam.lock:
            health = {"score": round(self.slam.score, 3),
                      "edge": self.slam.match_edge,
                      "ambiguity": round(self.slam.ambiguity, 3),
                      "rejected": self.slam.rejected}
        ok = (not health["rejected"]
              and health["score"] >= RESEED_MIN_SCORE
              and not health["edge"]
              and health["ambiguity"] < RESEED_MAX_AMBIGUITY)
        return ok, health

    def _refind(self, within_s=TURN_REFIND_S):
        """Wait for the matcher to agree with a re-seeded pose, or give up on it.

        Per revolution rather than after a fixed wait, which is the difference that
        matters: the old code slept seven revolutions and then read the score once,
        by which time seven scans had already been folded into the map at whatever
        heading the re-seed invented. Nothing is written here until this returns
        true, so the cost of being wrong is a few revolutions of pose and no map.

        Returns (ok, health) -- see _match_health.
        """
        # The burst coasts, and a scan taken mid-coast is smeared across the
        # rotation. Judging the re-seed on it would fail turns that were fine.
        time.sleep(TURN_SETTLE_S)
        deadline = time.monotonic() + within_s
        seen = self._scans
        good = 0
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
            ok, health = self._match_health()
            good = good + 1 if ok else 0
            if good >= REACQUIRE_GOOD_SCANS:
                return True, health
        return False, health

    def _pause_mapping(self, why):
        """Match, but write nothing, until something confirms where the rover is."""
        if not self._map_paused:
            self._log_event("map held", why)
        with self.slam.lock:
            self.slam.mapping = False
        self._map_paused = True
        self._good_run = 0

    def _resume_mapping(self, why):
        if self._map_paused:
            self._log_event("map resumed", why, **self._health)
        with self.slam.lock:
            self.slam.mapping = True
        self._map_paused = False
        self._good_run = 0

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

    def _note_match(self):
        """Record how the last match went, and heal the map when it comes good.

        While the map is held there is nothing to do but keep looking, so the way
        back is the matcher agreeing with itself for a couple of revolutions
        running. That is what "until it sees something it recognises" means in
        practice, and it means a rover left lost recovers by itself as soon as it is
        somewhere it knows, without anybody having to clear the map.
        """
        ok, self._health = self._match_health()
        if not self._map_paused:
            self._good_run = 0
            return
        self._good_run = self._good_run + 1 if ok else 0
        if self._good_run >= REACQUIRE_GOOD_SCANS:
            self._resume_mapping("the scan fits the map again")

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
        return True

    def _drop_lidar(self):
        try:
            if self.lidar is not None:
                self.lidar.close()
        except Exception:
            pass
        self.lidar = None
        self.lidar_path = None

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
        while self._run.is_set():
            if not self._open_lidar():
                self._watchdog()
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
                if revolutions and not self._suspend_slam:
                    self._dropped += revolutions - 1
                    if self._map_paused:
                        # Nothing is being written, so the pose is all there is to
                        # get back, and the ordinary window cannot reach it from
                        # tens of degrees out. Costs about three normal matches a
                        # revolution and stops the moment the pose is found again.
                        self.slam.request_recovery()
                    if self.slam.update():
                        self._scans += 1
                        self._last_scan_at = time.monotonic()
                        self._note_match()
                        self._on_scan()
            self._watchdog()
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

        # Kept every revolution, driving or not, so a move that is about to start
        # already has half a second of history to be cautious with.
        self._near_history.append(self._nearest())
        if len(self._near_history) > NEAR_HISTORY:
            del self._near_history[0]

        with self._lock:
            goal = self._goal
            estop = self._estop

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
                self._measured_speed += 0.5 * (along / dt - self._measured_speed)
                self._measured_turn += 0.5 * (math.degrees(dth) / dt
                                              - self._measured_turn)
        self._last_pose, self._last_at = pose, now

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
        turn = _clamp(chosen / 0.8, -MAX_TURN_DPS, MAX_TURN_DPS)
        self._drive_pwm(speed, turn)

    def _step_goto(self, goal, pose, now):
        """Follow a planned polyline by looking ahead along it.

        Progress is the closest point on the path, allowed to slide back a little
        so a weave beside the line is not a rewind. The carrot is nearly a metre
        further on, so the rover aims at a stretch rather than at the next cell.
        A corner is a turn on the spot; the rest is the same follow-the-gap drive
        as a straight move, aimed at the carrot. Replan rather than fight when
        the line is blocked, the rover has left the corridor, or the map has
        grown a wall on the remaining route.
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

        carrot = planner.point_at(path, s + GOTO_LOOKAHEAD_M)
        want = math.degrees(math.atan2(carrot[1] - y, carrot[0] - x) - th)
        want = (want + 180.0) % 360.0 - 180.0

        # A sharp corner is a turn, not a curve. Turning-over-the-move is how
        # the matcher used to lose the room; stopping and spinning is the move
        # this rover already has.
        if abs(want) > GOTO_TURN_DEG:
            turn = _clamp(want / 0.6, -MAX_TURN_DPS, MAX_TURN_DPS)
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
                self._drive_pwm(0.0, _clamp(best / 0.6, -MAX_TURN_DPS, MAX_TURN_DPS))
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
        turn = _clamp(chosen / 0.8, -MAX_TURN_DPS, MAX_TURN_DPS)
        self._drive_pwm(speed, turn)

    def _replan_could_differ(self, goal, now):
        """Whether asking the planner again could possibly come back with anything new.

        It plans from the pose, on the map. A rover that has not moved since the last
        route was drawn, over a map that has not been rebuilt under it, gets handed
        the same polyline and refuses it again one revolution later -- which is how
        eight replans and a "gave up" used to fit inside nine tenths of a second,
        with the rover standing still throughout and the caller told it had been
        tried eight times.
        """
        sx, sy, _th = goal["start_pose"]
        x, y, _ = self.slam.pose
        return (now - goal["started_at"] >= GOTO_REPLAN_MIN_S
                and math.hypot(x - sx, y - sy) >= GOTO_REPLAN_MIN_MOVE_M)

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
            # never the map -- but the map stops growing until it knows where it is.
            "mapping": not self._map_paused,
            "slam_events": [dict(e, age_s=round(time.monotonic() - e["at"], 1))
                            for e in self._events[-6:]],
            "scans": self._scans,
            "dropped_scans": self._dropped,
            "pwm": self._last_sent,
            "lidar_ok": self.lidar_ok(),
            # Both, because they disagree during a dead-reckoned turn and the pair is
            # what tells a healthy suspended map apart from a sensor that has died.
            "lidar_live": self.lidar_live(),
            "lidar_port": self.lidar_path,
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

    print("navigator: ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(_selftest())
