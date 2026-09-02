"""Following a route: steering, progress, and what DWB may sample.

The controller has to be held to what the wheels can actually do, so the samples
it is allowed to consider are checked against the same envelope the chassis
measurements set, and progress is checked for being more than translation.
"""
import json
import math
import os
import re
import sys

from test_harness import HERE, check, section
from test_config import settings_of


def wrap(radians):
    """A copy of nav_bridge.wrap."""
    return math.atan2(math.sin(radians), math.cos(radians))


def yaw_of_zw(z, w):
    """A copy of nav_bridge.yaw_of, taking the two components that matter."""
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def steering(where, poses, lookahead=1.0):
    """A copy of nav_bridge.NavBridge.steering, over plain (x, y) tuples."""
    if where is None or not poses:
        return None
    x, y, yaw = where
    for px, py in poses:
        if math.hypot(px - x, py - y) >= lookahead:
            return round(math.degrees(wrap(math.atan2(py - y, px - x) - yaw)), 1)
    px, py = poses[-1]
    if math.hypot(px - x, py - y) < 0.05:
        return None
    return round(math.degrees(wrap(math.atan2(py - y, px - x) - yaw)), 1)


def test_the_transform_budget_survives_a_scan():
    """The controller has to tolerate a scan going missing, and only just does.

    `map -> odom` is stamped with the last scan slam_toolbox picked up plus
    `transform_timeout`, and DWB refuses a pose that is older than
    `transform_tolerance` -- so between the two configs there is a fixed number of
    seconds that `laserCallback` may be busy before the next control tick throws
    ControllerTFError and the goal comes back as code 102, which is "lost".
    tf_stall_sim.py explains where that subtraction comes from.

    Two things are checked. The budget must be longer than the gap between scans,
    or every ordinary revolution would abort a goal; and the simulation must
    report a healthy stack as healthy, which is the guard the steering simulation
    had to learn the hard way.
    """
    section("the transform budget between slam_toolbox and the controller")
    try:
        import tf_stall_sim
    except ImportError:
        print("  .... skipped, no tf_stall_sim.py")
        return
    cfg = tf_stall_sim.config()
    budget = (cfg["transform_timeout"] + cfg["transform_tolerance"]
              - cfg["scan_stamp_offset"])
    scan_gap = 1.0 / tf_stall_sim.SCAN_HZ
    check("the budget is longer than one revolution", budget > scan_gap, True)
    print("       %.2f s of budget, %.0f ms between scans" % (budget, scan_gap * 1000))
    quiet = tf_stall_sim.run(cfg, seconds=6.0)
    check("nothing aborts when nothing stalls", len(quiet["aborts"]), 0)
    stalled = tf_stall_sim.run(cfg, seconds=6.0, stall=budget + 0.3, stall_at=2.0)
    check("...and a stall past the budget does", len(stalled["aborts"]) > 0, True)
    # And the way out of it, which is the only reason the budget above is
    # survivable on a route long enough to close a loop: with this on,
    # `map -> odom` is stamped when it is published rather than when the mapper
    # last picked up a scan, so a busy mapper no longer expires the correction
    # the controller steers on.
    slam = os.path.join(HERE, "config", "slam_toolbox.yaml")
    if os.path.exists(slam):
        check("slam_toolbox restamps map -> odom, so a busy mapper cannot expire it",
              "restamp_tf: true" in open(slam).read(), True)


def test_heading_arithmetic():
    """Wrapping, and reading a yaw back out of a quaternion.

    Both are one line and both have a failure that looks like the rover being
    possessed: an unwrapped difference turns "ten degrees to the left" into "three
    hundred and fifty to the right", and a quaternion read with the wrong sign
    turns every heading in the map upside down.
    """
    section("headings wrap and quaternions come back out")
    check("just under half a turn stays positive",
          math.degrees(wrap(math.radians(179))), 179.0, tolerance=0.001)
    check("just over half a turn comes back as the short way round",
          math.degrees(wrap(math.radians(181))), -179.0, tolerance=0.001)
    check("a full turn is no turn",
          math.degrees(wrap(math.radians(360))), 0.0, tolerance=0.001)
    check("two and a bit turns anticlockwise is still ten degrees",
          math.degrees(wrap(math.radians(730))), 10.0, tolerance=0.001)
    for degrees in (-179.0, -90.0, -1.0, 0.0, 45.0, 90.0, 179.0):
        half = math.radians(degrees) / 2.0
        check("a yaw of %+.0f survives the round trip" % degrees,
              math.degrees(yaw_of_zw(math.sin(half), math.cos(half))),
              degrees, tolerance=0.001)


def test_steering_bearing():
    """Which way the rover is trying to go, off its own nose.

    This replaced a number the old follower could name directly, because it chose
    between candidate arcs. A velocity controller chooses no such thing, so the
    honest substitute is the bearing to the route a lookahead ahead -- and the one
    thing it must get right is the sign, because a panel that says "left" while
    the rover goes right is worse than a panel that says nothing.
    """
    section("steering points at the route, on the correct side")
    # Facing along +x at the origin, with a route that turns left.
    route = [(0.2, 0.0), (0.6, 0.1), (1.2, 0.6)]
    check("a route bending left reads as a left bearing",
          steering((0.0, 0.0, 0.0), route) > 0, True)
    check("...and the same route mirrored reads as a right one",
          steering((0.0, 0.0, 0.0), [(x, -y) for x, y in route]) < 0, True)
    check("a route straight ahead is zero degrees",
          steering((0.0, 0.0, 0.0), [(2.0, 0.0)]), 0.0)
    # Facing +y now: a point at (0, 2) is dead ahead rather than 90 degrees off.
    check("the bearing is relative to the rover, not to the map",
          steering((0.0, 0.0, math.pi / 2), [(0.0, 2.0)]), 0.0)
    check("a route entirely underneath the rover says nothing at all",
          steering((0.0, 0.0, 0.0), [(0.01, 0.0)]), None)
    check("no route at all says nothing", steering((0.0, 0.0, 0.0), []), None)
    check("no pose says nothing", steering(None, route), None)


def test_a_route_is_budgeted_on_the_route():
    """A goal 3 m away round a wall must not be cancelled as though it were 3 m.

    The numbers here are the route the planner really returned on 2026-08-24 from
    (0.19, -11.34) to (2.74, -9.86), read off the rover: 346 poses, 8.81 m, and a
    straight line of 2.95 m with a wall across it. The old allowance came to
    50.6 s against about 44 s of flawless driving -- 15% of headroom on a stack
    that replans every second and spends 15 s on each rung of its recovery ladder.
    Three progress-checker windows is 45 s by itself, and there were three,
    because the checker kept calling a legitimate pivot a jam.
    """
    section("a route is budgeted on the route, not on the straight line")
    straight = 2.95
    # The real route, as the eighteen waypoints it was sampled at on the rover.
    route = [(0.19, -11.34), (-0.3, -11.3), (-0.8, -11.3), (-1.0, -11.0),
             (-0.9, -10.7), (-0.6, -10.3), (-0.1, -10.2), (0.3, -10.1),
             (0.8, -10.1), (1.0, -9.7), (0.9, -9.2), (0.8, -8.8), (1.1, -8.6),
             (1.5, -8.4), (2.0, -8.3), (2.4, -8.3), (2.6, -8.8), (2.8, -9.2),
             (2.8, -9.7), (2.74, -9.86)]
    sys.path.insert(0, HERE)
    try:
        import route_cost
    except Exception as exc:
        print("  .... skipped, cannot import route_cost: %s" % exc)
        return
    metres, turning = route_cost.length_and_turning(route)
    check("the route is longer than the straight line", metres > 2 * straight)
    check("and it is measured as such", round(metres, 1), 8.3, tolerance=0.6)
    check("its turning is counted, not ignored", turning > 200.0)

    was = max(30.0, 6.0 * straight / 0.35)
    now = route_cost.seconds_for(metres, turning, 0.35, 27.0, slack=3.0,
                                 floor=45.0)
    need = metres / 0.40 + turning / 27.0
    # **Not "the old allowance was too short to drive the route" -- it was not.**
    # 50.6 s against about 44 s of flawless execution is 15% of margin, and 15%
    # is nothing on a stack that replans once a second and whose recovery ladder
    # spends 15 s a rung. Three progress-checker windows is 45 s on its own. So
    # what the old number lacked was not length, it was headroom, and that is what
    # this asserts.
    check("the old allowance left no room for a single recovery",
          was < 1.5 * need)
    check("the new allowance leaves room for the recovery ladder",
          now > 2.5 * need)

    # A grid staircase is not a route full of corners. This is the guard on the
    # sampling: a dead straight line stored at 5 cm must cost no turning at all.
    staircase = [(0.05 * k, 0.0) for k in range(60)]
    check("a straight line asks for no turning",
          route_cost.length_and_turning(staircase)[1], 0.0, tolerance=1.0)
    # And the reverse guard: a real right angle must survive the sampling.
    corner = [(0.05 * k, 0.0) for k in range(40)] +              [(1.95, 0.05 * k) for k in range(1, 40)]
    check("a real right angle is still a right angle",
          route_cost.length_and_turning(corner)[1], 90.0, tolerance=10.0)
    # Nothing to budget on is not a licence to budget on nothing.
    check("an empty plan costs nothing", route_cost.length_and_turning([]),
          (0.0, 0.0))
    check("and falls back to the floor",
          route_cost.seconds_for(0.0, 0.0, 0.35, 27.0, floor=45.0), 45.0)


def test_progress_is_not_only_translation():
    """The progress checker has to accept a pivot, on a chassis that pivots.

    `SimpleProgressChecker` measures displacement alone, and every direction
    change this rover makes is a turn on the spot. That is what aborted a
    perfectly good route three times in fifteen-second slices, cleared two
    costmaps that had nothing wrong with them, and let the replan come back the
    other way round the wall so the rover turned back -- the wiggle somebody
    watched.
    """
    section("progress means moving or turning, not only moving")
    path = os.path.join(HERE, "config", "nav2.yaml")
    if not os.path.exists(path):
        print("  .... skipped, no config/nav2.yaml")
        return
    with open(path) as fh:
        text = fh.read()
    # Comments stripped first, and that matters more here than anywhere else in
    # this file: the block explaining *why* SimpleProgressChecker was replaced
    # names it four times, so a search over the raw text finds the explanation
    # and calls it the setting.
    settings = settings_of(text)
    check("the progress checker is PoseProgressChecker",
          "nav2_controller::PoseProgressChecker" in settings)
    check("SimpleProgressChecker is not the one configured",
          "nav2_controller::SimpleProgressChecker" in settings, False)

    def number(name):
        found = re.search(r"^\s*%s:\s*([-\d.]+)" % name, settings,
                          re.MULTILINE)
        return float(found.group(1)) if found else None

    angle = number("required_movement_angle")
    allowance = number("movement_time_allowance") or 15.0
    check("a turn counts as progress", angle is not None)
    if angle is not None:
        # Above the gyro's standing bias over the whole window, or a rover
        # standing perfectly still would report progress for ever. base_node
        # measures that bias at about +0.43 deg/s.
        drift = math.radians(0.43 * allowance)
        check("and the threshold clears the gyro's own drift over the window",
              angle > 2 * drift)
        # And well below any real turn, or a legitimate pivot would fail to
        # register. The slowest pivot this chassis holds is about 9 deg/s.
    check("while staying far inside the slowest real pivot",
          angle < math.radians(9.0 * allowance) / 4.0)


def test_dwb_will_not_sample_a_turn_the_wheels_cannot_hold():
    """The 2026-08-25 doorway recording, as a sample-set test.

    Eight in ten commands were standing turns, most of them 3 deg/s. The mixer
    lifts those to 12 deg/s, the rover overshoots, and it swaps sides. Nav2's
    isValidSpeed only drops a sample when *both* floors fail, so this checks
    the generated set, not just the yaml strings.
    """
    section("DWB does not sample a standing turn slower than the mixer")
    sys.path.insert(0, HERE)
    try:
        import corridor_sim as dwb
    except ImportError as exc:                          # pragma: no cover
        print("  .... skipped, cannot import corridor_sim: %s" % exc)
        return

    samples = dwb.twists()
    slow = [(vx, wz) for vx, wz in samples
            if abs(vx) < 0.05 and abs(wz) < dwb.MIN_SPEED_THETA]
    check("the sample count matches the 29 the live controller will log",
          len(samples), dwb.CANDIDATES)
    check("...and that is 12 standing turns, not the old 16",
          dwb.PIVOTS, 12)
    check("no standing turn is slower than 0.21 rad/s", slow, [])
    rolling = [(vx, wz) for vx, wz in samples if abs(vx) >= 0.05]
    check("...while the forward sample still has every steer, including slow ones",
          any(abs(wz) < dwb.MIN_SPEED_THETA for vx, wz in rolling), True)

    episode_path = os.path.join(HERE, "recordings", "doorway-2026-08-25.json")
    if os.path.isfile(episode_path):
        with open(episode_path) as handle:
            episode = json.load(handle)
        below = sum(1 for c in episode["commands"]
                    if abs(c["vx"]) < 0.05 and 0.05 <= abs(c["wz"]) < 0.21)
        check("the doorway recording still shows the fault this floor deletes",
              below > 100, True)
    else:
        print("  .... no recordings/doorway-2026-08-25.json, sample set only")


def test_lattice_respects_the_dwb_envelope():
    """The doorway corner, on a map that does not need a recording.

    NavFn's grid search has no turning radius, so the path it traces through
    a metre-wide 55 deg bend kinks more in 0.32 m than DWB can follow at
    speed. The differential lattice is given a 0.5 m control set (this
    chassis's max_vel_x / max_vel_theta, to a centimetre) and has to stay
    inside that envelope on the driving stretches. This is the reproduction
    docs/doorway-pivot.md asked for before SmacPlanner replaced NavFn: the
    same costmap, both searches, the path geometry -- not a closed loop
    started from a NavFn deadlock.
    """
    section("the state lattice will not draw a corner DWB cannot follow")
    sys.path.insert(0, HERE)
    try:
        import hybrid_astar as geo
        import lattice
        import corridor_sim as dwb
    except ImportError as exc:                          # pragma: no cover
        print("  .... skipped, cannot import lattice: %s" % exc)
        return

    radius = dwb.MAX_VEL_X / dwb.MAX_VEL_THETA
    check("DWB's forward envelope is max_vel_x over max_vel_theta",
          geo.MIN_TURNING_RADIUS, radius, tolerance=1e-9)
    check("...and is 0.51 m with the numbers in nav2.yaml",
          radius, 0.51, tolerance=0.005)
    meta, _ = lattice.load_lattice()
    check("the control set's radius is that envelope to a centimetre",
          meta["turning_radius"], 0.5, tolerance=1e-9)
    check("...and the motion model is differential, not ackermann",
          meta["motion_model"], "diff")

    result = lattice.doorway_reproduction()
    ninfo = result["navfn"]
    linfo = result["lattice"]
    check("the grid search still finds a route through the doorway",
          ninfo is not None)
    check("the lattice finds one too", linfo is not None)
    if ninfo is None or linfo is None:
        return
    check("NavFn's doorway corner is tighter than one DWB rollout (%.1f deg > %.1f)"
          % (ninfo["tightest_deg"], result["rollout_deg"]),
          ninfo["followable"], False)
    check("the lattice stays inside that envelope (%.1f deg)"
          % linfo["tightest_deg"],
          linfo["followable"], True)
    check("...without needing a pivot on a doorway that takes an arc",
          linfo["pivots"], 0)
    check("...and does not wander off: the route is within 2x the grid one",
          linfo["length_m"] < 2.0 * ninfo["length_m"], True)


def test_dwb_drives_the_body_into_a_door_frame():
    """The after-floor doorway recording, as a pose test, not a closed loop.

    Mixer floor stopped the pivoting. The rover then drove 3.5 m and sat
    next to a door frame for fifty seconds. Last driving command was
    0.40 m/s with the rectangle already in the inscribed ring, the nose
    4 cm from lethal, and 1.1 m still to the goal. Then one (0, 0).

    The synthetic 0.80 m door is that last driving tick without the
    recording: 14 cm off centre, 20 degrees toward the jamb. Frozen-map
    closed loops are how the 0.8 m look-ahead shipped on worthless
    evidence; this scores the pose once. The recording, when present,
    is the sit: PoseProgressChecker would have fired, and the model
    still had a forward candidate on the stop tick, so FollowPath had
    already ended.
    """
    section("DWB will drive the body into a door-frame halo")
    sys.path.insert(0, HERE)
    try:
        import jam_repro
    except ImportError as exc:                          # pragma: no cover
        print("  .... skipped, cannot import jam_repro: %s" % exc)
        return

    result = jam_repro.jam_reproduction()
    a = result["approach"]
    check("the body already covers the inscribed ring of the jamb",
          a["ring"] > 0, True)
    check("...without covering a lethal cell", a["lethal"], 0)
    check("...while the centre cell is still a legal planner step",
          a["centre"] < 253, True)
    check("DWB still has a legal candidate", a["legal"] > 0, True)
    check("...and the one it picks is 0.40 m/s forward",
          a["best_vx"], 0.40, tolerance=0.01)
    check("there is no reverse sample to back out with",
          a["reverse_samples"], 0)
    check("...even though 30 cm behind is clear",
          a["room_behind_m"], 0.30, tolerance=0.01)
    check("the nose is about 4 cm from lethal, as on the rover",
          a["nose_lethal_m"], 0.22, tolerance=0.03)

    live = result["recording"]
    if live is None:
        print("  .... no recordings/doorway-2026-08-25-after-floor.json")
        return
    check("the recording's last drive is still 0.40 m/s",
          live["last_drive_vx"], 0.40, tolerance=0.01)
    check("...with the body already in the ring and not in lethal",
          live["ring"] > 0 and live["lethal"] == 0, True)
    check("...and more than a goal-tolerance from the plan's end",
          live["goal_m"] > 0.22, True)
    check("then it sat less than 20 cm in fifty seconds",
          live["sat_m"] < 0.20 and live["sat_s"] > 40.0, True)
    check("PoseProgressChecker would have called that stuck",
          live["stuck_at"] is not None, True)
    if live["stuck_at"] is not None:
        check("...inside one 15 s window of the last drive",
              live["stuck_at"] - live["last_drive_t"] < 16.0, True)
    check("on the (0, 0) tick the model still had a legal forward",
          (live["model_legal_at_stop"] or 0) > 0, True)


TESTS = (
    test_the_transform_budget_survives_a_scan,
    test_heading_arithmetic,
    test_steering_bearing,
    test_a_route_is_budgeted_on_the_route,
    test_progress_is_not_only_translation,
    test_dwb_will_not_sample_a_turn_the_wheels_cannot_hold,
    test_lattice_respects_the_dwb_envelope,
    test_dwb_drives_the_body_into_a_door_frame,
)
