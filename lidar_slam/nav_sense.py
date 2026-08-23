"""Pose trust, lidar I/O and the 10 Hz loop. Mixed into Navigator."""
import math
import time

import serial

import usbreset
from nav_types import *  # noqa: F403
from nav_types import _clamp, _pose_close, _why_lost, find_lidar


class NavSense:
    """Whether the pose is worth believing, and keeping the lidar alive."""

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

