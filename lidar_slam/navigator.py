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
TURN_RESEED_S = 0.7        # after a burst, time for the matcher to find itself again
# How well the scan must fit the map after a re-seed before the new heading is
# believed. Re-seeding hands the matcher an answer it cannot argue with -- its window
# is 9 degrees -- so this score is the only evidence that the answer was right.
RESEED_MIN_SCORE = 0.35

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
UNKNOWN_AHEAD_SECTORS = 3  # +/-30 degrees at 36 sectors
# How many recent revolutions the "is anything touching us" test looks back over.
# One is not enough: a thin or dark object near the sensor's 0.12 m floor comes and
# goes between scans, and testing only the newest let a turn start beside something
# 0.13 m away and run for nearly four seconds before a scan happened to see it
# again. Taking the closest thing seen in the last half second instead means a
# return only has to appear once to be believed.
NEAR_HISTORY = 5


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
    @_one_move_at_a_time
    def drive(self, distance_m=None, speed_ms=None, seconds=None):
        """Go forward until the distance is covered or something is in the way.
        Blocks until it is done and says why it stopped. Avoidance may steer
        around obstacles, and will say so.
        """
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
        """
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
        path, last_why = self._plan_route(target)
        if not path:
            return Outcome("blocked", 0.0, 0.0, last_why)

        started = time.monotonic()
        travelled_before = 0.0
        turned_before = 0.0
        replans = 0

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

    def _plan_route(self, target_xy):
        """A polyline from here to `target_xy`, or (None, why)."""
        import numpy as np

        with self.slam.lock:
            grid = np.array(self.slam.grid(), copy=True)
            pose = self.slam.pose
            res = self.slam.config.resolution_m
            occupied_at = self.slam.config.occupied_at
        return planner.plan(grid, res, occupied_at, (pose[0], pose[1]), target_xy,
                            inflate_m=STANDOFF_M)

    @_one_move_at_a_time
    def turn_in_place(self, angle_deg, speed_dps=None):
        """Rotate by this many degrees, counter-clockwise positive.

        Dead reckoned: a burst of fixed PWM for a computed time, using the rates in
        TURN_RATES. That is roughly six times faster than servoing on the scan match
        was, and it keeps working when the lidar browns out during the turn -- which
        it does, because the motors and the lidar share one 5 V rail.

        The matcher cannot follow 170 deg/s (its window is 90), so map updates are
        suspended for the burst and the heading is re-seeded from the dead reckoning
        afterwards. Integrating scans at wrong poses would corrupt the map, and a map
        corrupted by a turn is worse than a turn that is a few degrees out.

        Then, if the lidar is alive, the result is checked and corrected once at the
        fine PWM. If it is not, the dead-reckoned figure is reported and said to be
        dead-reckoned rather than passed off as a measurement.
        """
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
            start_th = self.slam.pose[2]
            had_lidar = self.lidar_live()

            # The bulk of it, fast -- unless something is close, in which case the
            # whole turn goes at the fine rate. Turning is still never refused.
            pwm = (TURN_FINE_PWM if gentle or abs(angle) < TURN_FAST_MIN_DEG
                   else TURN_FAST_PWM)
            done = self._burst_turn(pwm, angle)   # moves the pose to match, itself
            time.sleep(TURN_RESEED_S)             # and this is the matcher re-finding it

            # Three outcomes, and they are worth telling apart. The sensor may have
            # stopped reporting, in which case the commanded turn is all there is to
            # report and it must not be dressed up as a measurement. Or it is
            # reporting but what it sees no longer fits the map at the re-seeded pose,
            # which means the rover did not go where dead reckoning thinks -- jammed
            # against something, most likely -- and the heading is not to be trusted.
            # Or it fits, and the small remaining error can be corrected.
            if not self.lidar_live():
                return Outcome("arrived", 0.0, done,
                               "dead reckoned; the lidar stopped reporting, so this "
                               "is the commanded turn and not a measured one")
            reseed_score = self.slam.score
            if reseed_score < RESEED_MIN_SCORE:
                return Outcome("lost", 0.0, done,
                               f"the turn was dead reckoned as {done:.0f} degrees but "
                               f"the scan no longer fits the map there (match "
                               f"{reseed_score:.2f}), so the rover was probably "
                               f"obstructed part way round and its heading is not to "
                               f"be trusted until it sees something it recognises")

            error = math.degrees((math.radians(angle) - (self.slam.pose[2] - start_th)
                                  + math.pi) % (2 * math.pi) - math.pi)
            if abs(error) > TURN_TOLERANCE_DEG:
                done += self._burst_turn(TURN_FINE_PWM, error)
                time.sleep(TURN_RESEED_S)
                if self.lidar_live():
                    error = math.degrees(
                        (math.radians(angle) - (self.slam.pose[2] - start_th)
                         + math.pi) % (2 * math.pi) - math.pi)

            turned = math.degrees((self.slam.pose[2] - start_th + math.pi)
                                  % (2 * math.pi) - math.pi) if had_lidar else done
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

        Re-seeding is a loaded gun and worth understanding before trusting the
        heading that comes back. It *tells* the matcher where it is, and the coarse
        search window is only about 9 degrees, so if the dead reckoning was well out
        the matcher cannot climb back and will simply agree with the wrong answer.
        Observed: a turn that physically managed 42 degrees of a requested 90 was
        reported as 90, because that is what it had been told. The match score is the
        only thing that gives it away, which is why the caller checks it.
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
            x, y, th = self.slam.pose
            self.slam.pose = (x, y, th + math.radians(turned))
        finally:
            self._suspend_slam = False
        return turned

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
                    if self.slam.update():
                        self._scans += 1
                        self._last_scan_at = time.monotonic()
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

        if len(self._trail) < 4000 and (
                not self._trail
                or math.hypot(pose[0] - self._trail[-1][0],
                              pose[1] - self._trail[-1][1]) > 0.05):
            self._trail.append((pose[0], pose[1]))

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
        if cross > GOTO_CORRIDOR_M:
            goal["done"] = ("replan",
                            f"drifted {cross:.2f} m off the route, so planning "
                            f"again from here")
            self._drive_pwm(0.0, 0.0)
            return

        if now - goal.get("last_check", now) >= GOTO_RECHECK_S:
            goal["last_check"] = now
            if self._route_blocked_on_map(path, s):
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
            goal["done"] = ("replan",
                            f"the way towards the route is {clear:.2f} m and the "
                            f"rover keeps {STANDOFF_M:.2f} m from anything it can "
                            f"see")
            self._drive_pwm(0.0, 0.0)
            return

        if self._unknown_ahead():
            limit = min(limit, CRAWL_SPEED_MS)
        if abs(want) > GOTO_ALIGN_DEG:
            limit = min(limit, CRAWL_SPEED_MS)
        speed = min(goal["speed"], limit)
        turn = _clamp(chosen / 0.8, -MAX_TURN_DPS, MAX_TURN_DPS)
        self._drive_pwm(speed, turn)

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
    def status(self):
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
            "match_score": round(self.slam.score, 3),
            "position_trusted": not self.slam.rejected,
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

    def map_png(self, half_extent_m=3.0, scale=3, rover_up=False):
        import mapimg
        return mapimg.render(self.slam, half_extent_m, scale, tuple(self._trail),
                             rover_up=rover_up)
