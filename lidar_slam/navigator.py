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
import math
import threading
import time

import planner
from nav_drive import NavDrive
from nav_sense import NavSense
from nav_types import (
    CMD_HEARTBEAT, CMD_PWM, HEARTBEAT_MS, LIDAR_RESET_COOLDOWN_S,
    LOOKAHEAD_M, MAX_GOTO_M, MIN_PWM, MoveReport, Outcome, find_lidar,
)
from odometry import Odometry
from slam2d import Slam2D, default_config

# Re-exports for dryrun.py / calibrate_turn.py / `from navigator import ...`.
__all__ = [
    "CMD_HEARTBEAT", "CMD_PWM", "HEARTBEAT_MS", "LOOKAHEAD_M", "MAX_GOTO_M",
    "MIN_PWM", "MoveReport", "Navigator", "Outcome", "find_lidar",
]


class Navigator(NavDrive, NavSense):
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
    from navigator_selftest import selftest
    return selftest()


if __name__ == "__main__":
    raise SystemExit(_selftest())
