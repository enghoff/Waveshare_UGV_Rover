"""The two bridges, the calibration store, and what the board discovers.

The protocol between the daemon and the ROS side is the interface between two
processes in two different Pythons, so a mismatch in it cannot be caught by
reading either side alone. Discovery is checked for staying on this board.
"""
import json
import math
import os
import socket
import sys
import time

from test_harness import HERE, check, section


# --- the board bridge ---------------------------------------------------------
def test_bridge_protocol():
    """The bridge's own command parsing, against a board that records what it got."""
    section("board bridge")
    # Two layouts. In the repository this file is in ros_nav/ and board_bridge.py
    # is in the sibling rover_daemon/; on the rover ~/ugv is flat and the daemon's
    # modules sit directly in the parent. Checking both means the bridge is tested
    # on the machine that actually runs it, which is where it matters -- this
    # section is thirteen of the checks, and skipping them there was silent.
    for candidate in (os.path.join(HERE, "..", "rover_daemon"),
                      os.path.join(HERE, "..")):
        sys.path.insert(0, os.path.abspath(candidate))
    try:
        import board_bridge
    except ImportError as exc:
        print("  .... skipped, no board_bridge.py beside this checkout (%s)" % exc)
        return

    class FakeLink:
        def __init__(self):
            self.sent = []
            self.pumps = 0

        def pump(self):
            self.pumps += 1

        def motion(self):
            return {"at": 1.0, "gz_lsb_s": 12.5, "ticks": 7.5, "samples": 3,
                    "breaks": 0}

        def telemetry(self):
            return {"T": 1001, "v": 1150, "gz": 3, "ax": 10}

        def send(self, command):
            self.sent.append(command)
            return True

    link = FakeLink()
    # Port 0 lets the OS pick a free one, so a self-test never collides with a
    # bridge that is actually running on this machine.
    bridge = board_bridge.BoardBridge(link, host="127.0.0.1", port=0)

    check("a command reaches the board",
          bridge.command(b'{"send": {"T": 11, "L": 5, "R": 5}}')["ok"], True)
    check("...and it is the command that was sent",
          link.sent[-1], {"T": 11, "L": 5, "R": 5})
    check("a bare command object works too, without the wrapper",
          bridge.command(b'{"T": 132, "IO4": 8}')["ok"], True)
    check("nonsense is refused rather than forwarded",
          bridge.command(b'not json at all')["ok"], False)
    check("...and an object with no command in it is refused",
          bridge.command(b'{"hello": 1}')["ok"], False)
    check("a refusal does not reach the board", len(link.sent), 2)

    # Before the pump has run there is nothing to report, and saying so is the
    # right answer -- a snapshot that invented zeroes would be odometry claiming
    # the rover is stationary when in truth nothing has been asked yet.
    check("a snapshot before the first pump reports nothing, not zeroes",
          bridge.snapshot()["motion"], None)

    bridge.start()
    try:
        for _ in range(50):
            if bridge.snapshot()["motion"] is not None:
                break
            time.sleep(0.02)
        snapshot = bridge.snapshot(full=False)
        check("a snapshot carries the motion counters",
              snapshot["motion"]["ticks"], 7.5)
        check("...and leaves out the telemetry when it was not asked for",
              "telemetry" in snapshot, False)
        check("...but includes it when it is", "telemetry" in bridge.snapshot(full=True),
              True)
        # End to end over a real socket, which is what catches a framing mistake
        # that every in-process test would miss.
        host, port = bridge.address
        sock = socket.create_connection((host, port), timeout=3)
        sock.settimeout(3)
        sock.sendall(b'{"send": {"T": 11, "L": 1, "R": -1}}\n')
        pending, records, deadline = b"", [], time.monotonic() + 3
        while time.monotonic() < deadline and len(records) < 4:
            pending += sock.recv(4096)
            while b"\n" in pending:
                line, pending = pending.split(b"\n", 1)
                if line.strip():
                    records.append(json.loads(line))
        sock.close()
        kinds = {r["kind"] for r in records}
        check("the stream carries motion records", "motion" in kinds, True)
        check("...and the command was acknowledged", "ack" in kinds, True)
        check("...and the board actually got it",
              {"T": 11, "L": 1, "R": -1} in link.sent, True)
        check("the pump loop is running", link.pumps > 0, True)
    finally:
        bridge.close()


# --- where the rover has been -------------------------------------------------
def test_a_cleared_map_does_not_start_the_track_in_the_old_frame():
    """**The fault, as the rover had it on 2026-09-04.**

    The map was cleared and the console drew a straight 5.37 m line at the head
    of the track, out of the mapped room and across open grey, from
    (2.267, -18.303) to (4.186, -23.318). The rover had not moved: every other
    step in that track was 0.34 m or less, half a second of driving, and there
    was nothing in between. Clearing the map re-anchors the frame on raw
    odometry, so the correction the session had built up is discarded in one
    step -- and slam_toolbox goes on publishing the old one until it folds a scan
    into the new graph, which a parked rover never gives it.

    So the numbers below are the rover's own, and the check is that the point
    from the discarded frame is not written down at all.
    """
    section("the track after a cleared map")
    import trail

    old_frame = (2.267, -18.303)
    new_frame = (4.186, -23.318)
    # The correction as it stood before the clear, and as slam_toolbox published
    # it once the new graph existed: a fresh graph is anchored on odometry, so
    # its correction is nothing at all.
    was = (-10.706, -3.895, math.radians(34.97))
    now = (0.0, 0.0, 0.0)
    odom = (4.186, -23.318, 0.0)

    track = trail.Trail()
    track.offer(old_frame, was, odom)
    check("before the clear the track is being recorded", len(track), 1)

    track.cleared(was, odom)
    check("clearing empties it", len(track), 0)
    check("a pose read in the frame that was just thrown away is not kept",
          track.offer(old_frame, was, odom), False)
    check("...however many times it is offered",
          any(track.offer(old_frame, was, odom) for _ in range(20)), False)

    check("the first pose in the new frame is kept",
          track.offer(new_frame, now, odom), True)
    check("...and it is the first point there is, so there is no 5.37 m step",
          track.points, [(4.186, -23.318)])

    # Everything after that is ordinary: thinned at 5 cm, and nothing waiting.
    check("a pose 2 cm on is too near to be worth keeping",
          track.offer((4.206, -23.318), now, odom), False)
    check("...and one 20 cm on is kept", track.offer((4.386, -23.318), now, odom),
          True)

    # A correction that was already nothing has no jump in it to wait out, and a
    # rover restarted a minute ago is exactly that case. Waiting there would be
    # waiting for a change that is never coming.
    fresh = trail.Trail()
    fresh.cleared((0.0, 0.0, 0.0), odom)
    check("clearing a map whose correction was already nothing holds nothing back",
          fresh.offer((0.5, 0.5), (0.0, 0.0, 0.0), odom), True)

    # And the backstop: a mapper that never re-anchors must not cost the whole
    # track. slam_toolbox takes its first scan after 0.2 m, so a metre of dead
    # reckoning with the correction unmoved is a mapper that is not going to move
    # it, and a track with one false step beats no track at all.
    stuck = trail.Trail()
    stuck.cleared(was, (0.0, 0.0, 0.0))
    check("half a metre driven with the correction unmoved still waits",
          stuck.offer((0.5, 0.0), was, (0.5, 0.0, 0.0)), False)
    check("...and a metre and a half gives up waiting and records",
          stuck.offer((1.5, 0.0), was, (1.5, 0.0, 0.0)), True)

    # A transform tree that has not said anything yet is not evidence of
    # anything, and must not be read as the correction having changed.
    quiet = trail.Trail()
    quiet.cleared(was, odom)
    check("no correction published at all is not a re-anchoring",
          quiet.offer(old_frame, None, None), False)


# --- calibration --------------------------------------------------------------
def test_calibration_store():
    section("the calibration store")
    store = os.path.expanduser("~/ugv/odometry.json")
    if not os.path.exists(store):
        print("  .... skipped, no %s on this machine" % store)
        return
    with open(store) as fh:
        loaded = json.load(fh)
    check("the gyro scale is present and positive",
          isinstance(loaded.get("gyro_lsb_per_dps"), (int, float))
          and loaded["gyro_lsb_per_dps"] > 0, True)
    # The measured envelope has to contain what Nav2 is allowed to ask for, or
    # the controller spends its life commanding a speed the base cannot deliver.
    points = loaded.get("drive_pwm_points")
    if isinstance(points, list) and len(points) >= 2:
        speeds = sorted(v for _, v in points)
        check("the measured speed curve rises with PWM, so it can be inverted",
              [v for _, v in points] == speeds, True)
        check("Nav2's 0.40 m/s limit is inside the measured range (%.2f-%.2f)"
              % (speeds[0], speeds[-1]), speeds[0] <= 0.40 <= speeds[-1], True)
        # The one that was missing, and its absence is what let the controller
        # spend a third of every drive commanding speeds the wheels cannot
        # produce. There is no creep on this chassis: below the slowest measured
        # PWM the motors do not turn, so anything Nav2 asks for between zero and
        # that speed arrives at the wheels as that speed.
        floor = speeds[0]
        cfg = os.path.join(HERE, "config", "nav2.yaml")
        if os.path.exists(cfg):
            with open(cfg) as fh:
                nav_text = fh.read()
            check("Nav2's top speed clears the chassis's %.2f m/s floor" % floor,
                  "max_vel_x: 0.40" in nav_text and floor < 0.40, True)
            check("...and it samples only the two speeds the chassis has",
                  "vx_samples: 2" in nav_text, True)
            check("...and its acceleration window spans the whole range, or the "
                  "samples collapse to a creep",
                  "acc_lim_x: 4.0" in nav_text, True)
    else:
        print("  ....  no drive_pwm_points yet -- run calibrate_chassis.py")

    ticks = loaded.get("ticks_per_metre")
    if ticks is None:
        print("  ....  ticks_per_metre is still null -- run calibrate_chassis.py on "
              "the rover; odometry distance is the commanded speed until then")
    else:
        check("the wheel scale is positive", ticks > 0, True)


def test_the_two_halves_agree_on_the_port():
    """The bridge's port is written in four files and nothing links them.

    A mismatch is the quietest failure in this whole stack: the daemon offers
    every driving tool, each one connects to a port nothing is listening on, and
    every tool call comes back "the ROS navigation stack is not answering" on a
    rover where it plainly is.
    """
    section("both halves agree where the bridge is")
    wanted = "8773"
    places = {
        "ros_nav/nav_bridge.py": ("PORT = " + wanted),
        "rover_daemon/ros_navigator.py": ("PORT = " + wanted),
        "rover_daemon/rover_daemon.py": ("ROS_NAV_PORT = " + wanted),
        "ros_nav/slam.launch.py": ('"nav_port", default_value="%s"' % wanted),
    }
    root = os.path.dirname(HERE)
    for relative, needle in places.items():
        path = os.path.join(root, relative)
        if not os.path.exists(path):
            # On the rover everything is deployed flat, so the repository layout
            # is not there to check. Saying so beats a failure that means nothing.
            print("  .... skipped, no %s" % relative)
            continue
        with open(path, encoding="utf-8", errors="replace") as fh:
            check("%s says %s" % (relative, wanted), needle in fh.read(), True)


def test_discovery_stays_on_this_board():
    """A dead radio must not be able to take the ROS graph with it.

    The console saying "only the mapping half is up" with every process still
    listed is CycloneDDS writing to a leftover address (wlan0's .139 after its
    DHCP lease moved). RoboStack's activate hook sets discovery to the
    subnet; dds.sh has to override that after env.sh, in every launcher, or the
    next interface change looks like Nav2 crashing.
    """
    section("discovery stays on this board")
    dds_path = os.path.join(HERE, "dds.sh")
    if not os.path.isfile(dds_path):
        print("  .... skipped, no dds.sh")
        return
    with open(dds_path, encoding="utf-8", errors="replace") as fh:
        dds = fh.read()
    check("dds.sh pins discovery to localhost",
          "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" in dds, True)
    check("...and ROS_LOCALHOST_ONLY, so CycloneDDS will not keep LAN peers",
          "ROS_LOCALHOST_ONLY=1" in dds, True)
    for name in ("run_ros_nav.sh", "restart.sh", "run_record.sh"):
        path = os.path.join(HERE, name)
        with open(path, encoding="utf-8", errors="replace") as fh:
            text = fh.read()
        check("%s sources dds.sh after env.sh" % name,
              'DIR/dds.sh' in text and text.find('DIR/env.sh') < text.find('DIR/dds.sh'),
              True)
    with open(os.path.join(HERE, "sweep.sh"), encoding="utf-8", errors="replace") as fh:
        sweep = fh.read()
    check("sweep SIGKILLs what ignored SIGTERM, or the leftover keeps the port",
          "pkill -9 -f" in sweep, True)

    # --- replaying a recorded drive -------------------------------------------
    # The replay harness runs a second mapper on the same board while the rover
    # is driving, so the one thing it must never do is tidy up by pattern: every
    # pattern that matches a replay's mapper matches the one the rover is
    # steering on. It kills what it started, by PID.
    replay_path = os.path.join(HERE, "replay_bag.sh")
    if os.path.isfile(replay_path):
        with open(replay_path, encoding="utf-8", errors="replace") as fh:
            replay = fh.read()
        # Read the code, not the comments. The header of that file says the word
        # "pkill" in order to warn about it, and a check that cannot tell a
        # warning from a use is worse than no check.
        replay_code = "\n".join(
            line for line in replay.splitlines()
            if line.strip() and not line.lstrip().startswith("#"))
        check("the bag replay kills only what it started, never by pattern",
              "pkill" not in replay_code and "sweep.sh" not in replay_code, True)
        # And it must not share a DDS domain with the rover, or the replayed
        # scans arrive on the live /scan and the replayed map on the live /map.
        check("...and it replays on its own DDS domain, off the rover's",
              "ROS_DOMAIN_ID=43" in replay, True)
        # The recorder is stopped by a signal, and a backgrounded child of a
        # non-interactive shell inherits SIGINT set to ignore -- which is a
        # recording nobody can stop, splitting a new file every N seconds.
        with open(os.path.join(HERE, "record_drive.sh"), encoding="utf-8",
                  errors="replace") as fh:
            recorder = fh.read()
        # Comments stripped first, for the third time in this file: that header
        # names --max-bag-duration in order to warn that it splits rather than
        # stops, and a check that cannot tell a warning from a use is worse than
        # no check.
        recorder_code = "\n".join(
            line for line in recorder.splitlines()
            if line.strip() and not line.lstrip().startswith("#"))
        check("the drive recorder can be stopped, so its bag gets its metadata",
              "timeout --signal=INT" in recorder_code
              and "--max-bag-duration" not in recorder_code, True)

    # RTAB-Map ran here for a day and was removed on 2026-08-31 -- the README
    # says why. This is the check that it went cleanly: a launch, a sweep or a
    # boot entry still reaching for a mapper that is not installed fails at a
    # distance from anything that names it.
    for name in ("slam.launch.py", "sweep.sh", "restart.sh", "install-boot.sh",
                 "nav_bridge.py", "replay_bag.sh"):
        path = os.path.join(HERE, name)
        if not os.path.isfile(path):
            continue
        with open(path, encoding="utf-8", errors="replace") as fh:
            body = fh.read().lower()
        check("%s does not reach for RTAB-Map, which this rover no longer has" % name,
              "rtabmap" not in body, True)

    # --- getting off something the rover is touching --------------------------
    # Nav2's Spin, DriveOnHeading and BackUp start their look-ahead projection at
    # the pose the rover is standing in, so a rover in contact is refused every
    # motion in every direction -- it will not drive off the obstacle and it will
    # not turn. `behaviors/` replaces all three with subclasses that differ only
    # in that state. These checks are the fix and its two guard rails: a motion
    # into an obstacle must still be refused, and the reasoning behind the spin
    # only holds while the footprint is a circle.
    import corridor_sim
    import goal_fit as _goal_fit

    def _wall(behind=None, ahead=None, span=4.0):
        """Open ground with a wall that far in front of or behind the rover.

        The rover stands at the origin facing +x. 0.12 m behind is inside the
        chassis, which reaches 0.16 m back from `base_link` -- so the rover is
        genuinely touching, which is the case that matters. The narrow band
        where the costmap says collision and the body is actually clear is only
        about a centimetre wide and is not what this is about.
        """
        res = corridor_sim.RESOLUTION
        cells = int(round(span / res))
        origin = -span / 2.0
        lethal = []
        for col in range(cells):
            x = origin + (col + 0.5) * res
            if (behind is not None and x <= -behind) or \
               (ahead is not None and x >= ahead):
                lethal.extend((col, row) for row in range(cells))
        return _goal_fit.CostGrid(cells, cells, res, origin, origin,
                                  corridor_sim.inflate(cells, cells, lethal))

    wall = _wall(behind=0.12)
    stock_spin = corridor_sim.spin_recovery(wall, 0.0, 0.0, 0.0,
                                            target=math.radians(90))
    escape_spin = corridor_sim.escape_spin(wall, 0.0, 0.0, 0.0,
                                           target=math.radians(90))
    check("a rover touching something behind cannot turn under stock Nav2",
          round(math.degrees(stock_spin[0]), 1), 0.0)
    check("...and can under the escape behaviours, which is the whole point",
          round(math.degrees(escape_spin[0])), 90)
    # The case that was watched on the rover and that the first version of this
    # got wrong: the rover is standing legally, and Nav2 still refuses the turn
    # because the rasterised outline of its sixteen-sided stand-in for a circle
    # clips a cell at one projected heading. A circular body must turn anyway --
    # every heading covers the same ground -- and a non-circular one must not,
    # because it really can sweep a corner into something.
    def _one_cell(distance, bearing_deg, span=4.0):
        """One lethal cell that far away on that bearing, and open floor otherwise.

        A straight wall will not show this: a half-plane is symmetric enough that
        a near-circular outline clips it at every heading or none. It takes a
        single marginal cell right at the edge of the footprint -- which is what
        a chair leg or a door frame is -- for the rasterised outline to catch it
        at one angle and miss it at another.
        """
        res = corridor_sim.RESOLUTION
        cells = int(round(span / res))
        origin = -span / 2.0
        col = int((distance * math.cos(math.radians(bearing_deg)) - origin) / res)
        row = int((distance * math.sin(math.radians(bearing_deg)) - origin) / res)
        return _goal_fit.CostGrid(cells, cells, res, origin, origin,
                                  corridor_sim.inflate(cells, cells, [(col, row)]))

    # Behind and to the left, at the distance where the footprint outline is
    # marginal. 582 such placements exist in this model; the bearing barely
    # matters, the distance is everything.
    edge = _one_cell(0.16, 135.0)
    check("the rover is standing somewhere perfectly legal",
          corridor_sim.collision_free(edge, 0.0, 0.0, 0.0), True)
    check("...and stock Nav2 still refuses to turn from it, which is the fault "
          "watched on the rover",
          corridor_sim.spin_recovery(edge, 0.0, 0.0, 0.0,
                                     target=math.radians(180))[1],
          "collision ahead")
    check("...and a circular footprint turns anyway, which is the 180 the rover "
          "was refused",
          round(math.degrees(corridor_sim.escape_spin(
              edge, 0.0, 0.0, 0.0, target=math.radians(180), circular=True)[0])),
          180)
    check("...while a non-circular one standing legally still may not",
          round(math.degrees(corridor_sim.escape_spin(
              edge, 0.0, 0.0, 0.0, target=math.radians(180), circular=False)[0]), 1),
          0.0)
    stock_fwd = corridor_sim.drive_on_heading(wall, 0.0, 0.0, 0.0,
                                              target=0.5, sign=1.0)
    escape_fwd = corridor_sim.escape_drive_on_heading(wall, 0.0, 0.0, 0.0,
                                                      target=0.5, sign=1.0)
    check("...nor drive away from it under stock Nav2", round(stock_fwd[0], 2), 0.0)
    check("...and can drive away from it under the escape behaviours",
          round(escape_fwd[0], 2), 0.5)
    escape_back = corridor_sim.escape_drive_on_heading(wall, 0.0, 0.0, 0.0,
                                                       target=0.3, sign=-1.0)
    check("but reversing *into* the thing it is touching is still refused",
          round(escape_back[0], 2), 0.0)
    ahead = _wall(ahead=0.12)
    escape_into = corridor_sim.escape_drive_on_heading(ahead, 0.0, 0.0, 0.0,
                                                       target=0.5, sign=1.0)
    check("...and so is driving forward into a wall, which is the safety that "
          "must survive all of this", round(escape_into[0], 2), 0.0)
    nav2_path = os.path.join(HERE, "config", "nav2.yaml")
    with open(nav2_path, encoding="utf-8", errors="replace") as fh:
        nav2_cfg = fh.read()
    check("the behaviour server actually loads the escape behaviours",
          "ugv_behaviors::EscapeSpin" in nav2_cfg
          and "ugv_behaviors::EscapeDriveOnHeadingAction" in nav2_cfg
          and "ugv_behaviors::EscapeBackUpAction" in nav2_cfg, True)
    # EscapeSpin is only sound because rotating a circle about its own centre
    # sweeps no new ground. A footprint polygon would break that silently.
    nav2_settings = "\n".join(
        line for line in nav2_cfg.splitlines()
        if line.strip() and not line.lstrip().startswith("#"))
    check("...and the circular footprint EscapeSpin's soundness rests on is "
          "still a circle", "robot_radius:" in nav2_settings
          and "footprint:" not in nav2_settings, True)
    # Guarded, like the other read of it below. The rover's ~/ugv is flat and has
    # no deploy/ in it, so this raised there rather than skipping -- and it took
    # the whole run down with it, which is why nothing in this file had been
    # checked on the rover for as long as the check has existed.
    deploy_path = os.path.join(os.path.dirname(HERE), "deploy", "manifest.json")
    if os.path.isfile(deploy_path):
        with open(deploy_path, encoding="utf-8") as fh:
            deploy_cfg = json.load(fh)
        ros_nav_cmds = next((c.get("commands") or []
                             for c in deploy_cfg["components"]
                             if c.get("name") == "ros_nav"), [])
        check("a deploy rebuilds the plugin, or the rover runs last week's .so",
              any("behaviors/build.sh" in cmd for cmd in ros_nav_cmds), True)
    else:
        print("  .... skipped, no deploy/manifest.json")
    # deploy.py packs every file with mtime = 0 so rsync can skip an unchanged
    # one, which leaves every source on the rover older than every object file.
    # An incremental build then finds nothing to do, for ever -- watched here,
    # three deploys running, with the behaviour server holding the first
    # build's library. The build has to key on content.
    with open(os.path.join(HERE, "behaviors", "build.sh"),
              encoding="utf-8", errors="replace") as fh:
        plugin_build = fh.read()
    check("...and keys the rebuild on the sources' content, not their timestamps",
          "sha256sum" in plugin_build and "rm -rf \"$BUILD\"" in plugin_build, True)
    with open(os.path.join(os.path.dirname(HERE), "deploy", "deploy.py"),
              encoding="utf-8", errors="replace") as fh:
        check("...which is needed because the deployer still dates files 1970",
              "info.mtime = 0" in fh.read(), True)
    check("...and it builds before it restarts, not after",
          ([i for i, c in enumerate(ros_nav_cmds) if "behaviors/build.sh" in c] or [99])[0]
          < ([i for i, c in enumerate(ros_nav_cmds) if "restart.sh" in c] or [-1])[0],
          True)
    with open(os.path.join(HERE, "restart.sh"), encoding="utf-8", errors="replace") as fh:
        restart = fh.read()
    check("restart.sh will not hang SSH on a wedged ros2 node list",
          "timeout 15 ros2 node list" in restart, True)
    with open(os.path.join(HERE, "nav_record.py"), encoding="utf-8", errors="replace") as fh:
        recorder = fh.read()
    check("a hung nav_record cannot sit in spin_once past the recording window",
          "threading.Timer" in recorder and "os._exit" in recorder, True)
    with open(os.path.join(HERE, "run_record.sh"), encoding="utf-8", errors="replace") as fh:
        wrapper = fh.read()
    check("...and the shell wrapper still fires if Python itself is stuck",
          "timeout --kill-after=15" in wrapper, True)
    manifest_path = os.path.join(os.path.dirname(HERE), "deploy", "manifest.json")
    if os.path.isfile(manifest_path):
        with open(manifest_path, encoding="utf-8") as fh:
            manifest = json.load(fh)
        ros_nav = next((c for c in manifest["components"]
                        if c.get("name") == "ros_nav"), None)
        when = []
        for rule in (ros_nav or {}).get("special_commands") or []:
            when.extend(rule.get("when") or [])
        check("deploying dds.sh replaces the supervisor, or boot still has SUBNET",
              "ros_nav/dds.sh" in when, True)
    else:
        print("  .... skipped, no deploy/manifest.json")


TESTS = (
    test_bridge_protocol,
    test_a_cleared_map_does_not_start_the_track_in_the_old_frame,
    test_calibration_store,
    test_the_two_halves_agree_on_the_port,
    test_discovery_stays_on_this_board,
)
