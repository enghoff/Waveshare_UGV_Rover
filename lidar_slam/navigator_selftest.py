"""Offline checks for navigator: commentary, pose trust, lidar recovery."""
import math
import threading

import usbreset
from nav_types import *  # noqa: F403
from nav_types import MoveReport, _pose_close
from navigator import Navigator
from odometry import Odometry


def _xy_close(one, two, tol=1e-6):
    return abs(one[0] - two[0]) < tol and abs(one[1] - two[1]) < tol


def selftest():
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

    # Where a route is planned to, for the two ways of asking. An offset is read
    # from wherever the rover is standing when the call arrives; a point is the
    # point, whenever it arrives. The difference is what lets a tap on the console's
    # map interrupt a move: the tap has to stop what is running first, and the rover
    # keeps driving until the stop lands, so the same click means one fixed place
    # absolutely and a place that has drifted by a metre relatively.
    class _Planning:
        """Enough of a Navigator to see what target the planner was handed."""

        _estop = False
        report = MoveReport()

        class slam:
            pose = (2.0, -1.0, math.pi / 2)     # at (2, -1), facing +y

        def __init__(self):
            self.targets = []

        def _preflight(self, _kind):
            return ""

        def _plan_route(self, target_xy):
            self.targets.append(target_xy)
            return None, "no route, which is all this stub is for"

        _drive_to = Navigator._drive_to

    nav = _Planning()
    outcome = nav._drive_to(1.0, 0.0, None)
    assert outcome.reason == "blocked", outcome
    assert _xy_close(nav.targets[-1], (2.0, 0.0)), (
        "a metre ahead of a rover facing +y is not a metre along +x")

    nav = _Planning()
    nav._drive_to(None, None, None, x_m=3.0, y_m=-1.0)
    assert _xy_close(nav.targets[-1], (3.0, -1.0)), (
        "a point on the map was moved by the pose it was planned from")

    # The distance cap and "already there" are about the target, so they have to be
    # measured from the rover in both forms. A point at the rover's own position is
    # already there however far from the origin the rover has driven.
    nav = _Planning()
    assert nav._drive_to(None, None, None, x_m=2.0, y_m=-1.0).reason == "arrived"
    assert nav.targets == [], "a route was planned to where the rover already is"
    far = nav._drive_to(None, None, None, x_m=2.0, y_m=-19.0)
    assert far.reason == "blocked" and "18.0 m away" in far.detail, far

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
    raise SystemExit(selftest())
