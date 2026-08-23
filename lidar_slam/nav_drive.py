"""Drive commands, path following and PWM. Mixed into Navigator."""
import math
import time

import journey
import planner
from nav_types import *  # noqa: F403
from nav_types import Outcome, _clamp, _one_move_at_a_time, _why_lost


class NavDrive:
    """Move requests and the steering that carries them out."""

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
        the motors and the lidar share one 5 V rail. `speed_dps` is accepted so
        older callers still type-check, and ignored; the rates are TURN_RATES.

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
            outcome = self._ended(self._turn_in_place(angle_deg))
        finally:
            recording, self._journey = self._journey, None
        if recording:
            recording.end(outcome.reason, outcome.detail)
        return outcome

    def _turn_in_place(self, angle_deg):
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
