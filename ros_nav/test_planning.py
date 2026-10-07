"""Goals, frontiers and exploring: where the rover decides to go.

Run against real recorded maps rather than invented grids, because the faults
worth catching -- a goal inside a wall, a frontier that is not reachable, an
exploration that never finishes -- only appear on a map with the mess in it.
"""
import json
import math
import os
import sys

from test_harness import HERE, check, section


def _bridge_source():
    """The navigation bridge's source, all five files of it, or "" off the rover.

    These checks read the bridge as text because they cannot import it: it needs
    rclpy, and this file runs on a workstation that has none. Since the bridge was
    split -- the node, its moves, its exploring, the map it keeps on disk and the
    numbers it is held to -- that means reading all five and looking at them as
    one, which is also what makes a count like "this appears exactly once" mean
    what it used to.
    """
    out = []
    for name in ("nav_bridge.py", "nav_moves.py", "nav_explore.py",
                 "nav_map.py", "nav_limits.py"):
        path = os.path.join(HERE, name)
        if os.path.exists(path):
            with open(path, encoding="utf-8") as fh:
                out.append(fh.read())
    return "\n".join(out)


def test_goal_fits_before_it_is_sent():
    """The goal check, on the real geometry rather than a stand-in.

    goal_fit.py has no ROS in it for exactly this reason, so what runs here is
    the code the rover runs. The numbers in the last two checks are the recorded
    failure: a goal at (4.34, -0.98) on a costmap where the body covered a lethal
    cell at the heading the bridge would have sent, which Nav2 accepted, planned
    a clean straight path to, and then spent thirty seconds failing to reach.
    """
    section("a goal is checked against the body before it is sent")
    sys.path.insert(0, HERE)
    try:
        import goal_fit
    except ImportError as exc:                          # pragma: no cover
        print("  .... skipped, cannot import goal_fit: %s" % exc)
        return

    body = goal_fit.polygon_from(
        '[[0.20, 0.14], [0.20, -0.14], [-0.16, -0.14], [-0.16, 0.14]]', 0.0)
    check("the footprint parses out of the string nav2.yaml holds",
          body == [(0.20, 0.14), (0.20, -0.14), (-0.16, -0.14), (-0.16, 0.14)],
          True)
    check("...and a bare radius still gives a polygon rather than nothing",
          len(goal_fit.polygon_from("", 0.25) or []) >= 8, True)

    # Two metres square of clear floor, with a wall down the right-hand side:
    # lethal from 1.70 m, and the inscribed ring reaching back to 1.50.
    width = height = 40
    data = [0] * (width * height)
    for row in range(height):
        for col in range(30, width):
            data[row * width + col] = 254 if col >= 34 else 253
    floor = goal_fit.CostGrid(width, height, 0.05, 0.0, 0.0, data)

    check("a goal in open floor fits", goal_fit.fits(floor, body, 0.5, 1.0, 0.0),
          True)
    check("...and one in the wall does not",
          goal_fit.fits(floor, body, 1.6, 1.0, 0.0), False)
    check("...and is left exactly where it was asked for",
          goal_fit.fit(floor, body, 0.5, 1.0, 0.0)["moved_m"] == 0.0, True)

    moved = goal_fit.fit(floor, body, 1.6, 1.0, 0.0)
    check("a goal in the wall is moved to somewhere the body fits",
          moved is not None and goal_fit.fits(floor, body, moved["x"],
                                              moved["y"], moved["yaw"]), True)
    check("...and not moved further than it has to be",
          moved is not None and moved["moved_m"] <= goal_fit.REACH_M, True)
    check("...and a goal with no way out at all is refused rather than tried",
          goal_fit.fit(floor, body, 1.9, 1.0, 0.0, reach_m=0.10), None)

    # Unknown is not an obstacle. The planner is configured with allow_unknown
    # because this rover maps as it drives, so a goal in a room it has not seen
    # yet has to be allowed through -- refusing it would stop exploration dead.
    unseen = goal_fit.CostGrid(width, height, 0.05, 0.0, 0.0,
                               [255] * (width * height))
    check("unknown floor does not block a goal, or the rover stops exploring",
          goal_fit.fits(unseen, body, 1.0, 1.0, 0.0), True)

    # The outline matters as well as the interior: a body can straddle a wall
    # one cell thick without any cell centre landing inside the polygon.
    thin = [0] * (width * height)
    for row in range(height):
        thin[row * width + 20] = 254
    check("a wall one cell thick is not stepped over by the interior test",
          goal_fit.fits(goal_fit.CostGrid(width, height, 0.05, 0.0, 0.0, thin),
                        body, 1.0, 1.0, 0.0), False)

    # And that the bridge actually asks. The geometry being right is no use if
    # `goto` never calls it, and this file cannot import nav_bridge to find out.
    source = _bridge_source()
    if source:
        check("the bridge checks a goal before sending it",
              "self.fit_goal(gx, gy, yaw)" in source, True)
        check("...and turns round rather than reversing the length of a room",
              "REVERSE_LIMIT_M" in source and "reverse_by_turning" in source,
              True)


def test_frontiers_are_found_on_a_real_map():
    """Which gap in the map is worth driving to, argued against a real map.

    `frontier.py` has no ROS in it for `goal_fit.py`'s reason, so what runs here
    is the code the rover runs. What it runs against is not a room invented for
    the test: it is the occupancy grid slam_toolbox produced from the recorded
    `kitchen-loop` drive, kept beside the recordings it came from. A chooser that
    works on a rectangle with a doorway drawn in it has been proved against a
    rectangle with a doorway drawn in it -- see "The simulation that could not
    fail" in the README for what that is worth.
    """
    section("frontiers, on the map a real drive produced")
    sys.path.insert(0, HERE)
    try:
        import frontier
    except ImportError as exc:                          # pragma: no cover
        print("  .... skipped, cannot import frontier: %s" % exc)
        return

    # --- the arithmetic first, on geometry small enough to reason about.
    # Two metres of floor with the right-hand third never seen, inside a wall.
    # The boundary between floor and unknown is a frontier about 2 m tall, and
    # nothing else in here is one.
    #
    # **The wall round the outside is not scenery.** Off the edge of a grid reads
    # as unknown, because that is what it is -- the map is only as big as what
    # has been seen -- so free floor running to the last column is a frontier,
    # and correctly so. A test room without walls is a room with four extra
    # frontiers round the outside, which is not what a mapped room looks like.
    width = height = 40
    data = [0] * (width * height)
    for i in range(width):
        data[i] = data[(height - 1) * width + i] = 100
    for row in range(height):
        data[row * width] = data[row * width + width - 1] = 100
    for row in range(1, height - 1):
        for col in range(30, width - 1):
            data[row * width + col] = -1
    room = frontier.Grid(width, height, 0.05, 0.0, 0.0, data)

    found, summary = frontier.survey(room, (0.5, 1.0))
    check("the edge of the known floor is found", len(found) >= 1, True)
    check("...and it is where the unknown starts, not somewhere in the middle",
          found and abs(found[0]["x"] - 1.475) < 0.06, True)
    check("...and the rover is sent facing the unknown, not away from it",
          found and abs(math.degrees(found[0]["yaw"])) < 45.0, True)
    check("...and the whole 2 m of boundary counts as one frontier, not forty",
          summary["frontiers"], 1)

    # A wall right across the room with unknown behind it is not a frontier: the
    # rover cannot walk to the far side, and offering it is how an explore spends
    # its budget failing to reach the same place.
    walled = list(data)
    for row in range(1, height - 1):
        walled[row * width + 20] = 100
        for col in range(21, width - 1):
            walled[row * width + col] = -1
    check("unknown ground behind a wall is not offered, because the walk to it "
          "does not exist",
          frontier.survey(frontier.Grid(width, height, 0.05, 0.0, 0.0, walled),
                          (0.5, 1.0))[0], [])

    # The walk is four-connected, and that is not a detail. Two rooms touching
    # at a single corner are not connected for a rover 36 cm wide.
    pinched = [100] * (width * height)
    for row in range(5, 15):
        for col in range(5, 15):
            pinched[row * width + col] = 0
    for row in range(15, 25):
        for col in range(15, 25):
            pinched[row * width + col] = -1
    check("a diagonal touch between two cells is not a way through",
          frontier.survey(frontier.Grid(width, height, 0.05, 0.0, 0.0, pinched),
                          (0.5, 0.5))[0], [])

    # A blacklisted frontier is not offered again, which is what stops an
    # explore driving to the same doorway until its budget runs out.
    keep = frontier.survey(room, (0.5, 1.0))[0]
    again, summary = frontier.survey(room, (0.5, 1.0),
                                     blacklist=[(keep[0]["x"], keep[0]["y"])])
    check("a frontier already tried is not offered again", again, [])
    check("...and the reason is reported rather than silent",
          summary["rejected_blacklisted"] >= 1, True)

    # --- and then the real map.
    saved = os.path.join(HERE, "fixtures", "kitchen-loop.pgm.gz")
    if not os.path.exists(saved):                       # pragma: no cover
        print("  .... skipped, %s is not here" % saved)
        return
    house = frontier.read_pgm(saved)
    free, unknown = frontier.classify(house)
    check("the saved map loads as the 12.4 x 16.4 m the drive covered",
          (round(house.width * house.resolution, 1),
           round(house.height * house.resolution, 1)), (12.4, 16.4))
    check("...with the floor and the unmapped part map_score.py counted",
          (sum(free), sum(unknown)), (22062, 56533))

    seen = [i for i, f in enumerate(free) if f]
    middle = house.point_of(int(sum(i % house.width for i in seen) / len(seen)),
                            int(sum(i // house.width for i in seen) / len(seen)))
    found, summary = frontier.survey(house, middle)
    check("there is somewhere worth driving to in a half-explored house",
          len(found) >= 3, True)
    check("...and some of the floor is behind something, and known to be",
          summary["reachable_cells"] < summary["free_cells"], True)
    check("...and every goal offered is on floor the mapper calls free",
          all(free[house.cell_of(c["x"], c["y"])[1] * house.width
                   + house.cell_of(c["x"], c["y"])[0]] for c in found), True)
    check("...and every one of them has unknown ground next to it",
          all(any(unknown[(house.cell_of(c["x"], c["y"])[1] + dr) * house.width
                          + house.cell_of(c["x"], c["y"])[0] + dc]
                  for dc, dr in ((-1, 0), (1, 0), (0, -1), (0, 1)))
              for c in found), True)
    check("...and the best of them is a real opening rather than a ragged cell",
          found[0]["size_m"] >= frontier.MIN_FRONTIER_M, True)

    # Ranking. The nearest frontier is not automatically the best one, and the
    # far one being preferred is the behaviour that gets a rover out of the room
    # it is in -- but only when it is enough bigger to be worth the drive.
    near = {"x": 0.0, "y": 0.0}
    cheap = min(found, key=lambda c: c["cost"])
    check("what wins is the trade between distance and size, not distance",
          cheap is found[0] and any(c["distance_m"] < found[0]["distance_m"]
                                    for c in found), True)
    del near


def test_a_rim_of_unknown_round_the_rover_is_still_open():
    """The rim that has exploring announce a finished house it has not driven in.

    **This is a recording, and it is an open fault rather than a fixed one.**
    `fixtures/ringed-2026-09-07.json.gz` is the occupancy grid off the running
    bridge on the Orin on 2026-09-07, taken while the console was showing "there
    is nothing left on the map worth driving to -- 1 frontier tried, 1 reached,
    0.0 m driven -- 97% of the map is still unknown". The rover was standing in
    about two square metres of mapped floor with unknown ground on every side of
    it and one cell of unknown right at its elbow, so the whole boundary -- the
    rim at arm's length and the hole beside the wheel alike -- is a single
    eight-connected clump of 209 cells.

    The clumping is not the fault: 10.45 m of boundary really is one boundary.
    The fault is what `survey` then does with it. A clump gets one goal, at the
    member cell nearest the clump's centre of mass; the centre of mass of a ring
    is the middle of the ring, which is where the rover is; so the goal comes out
    3.5 cm away. The rover drives nothing, arrives, and `Explorer.committed`
    writes off all 10.45 m on the strength of that arrival -- which is the rule
    that stops a rover re-driving the same 30 cm for ever, and is right for a
    doorway. The next survey has nothing left to offer.

    **Cutting the rim into pieces with a goal each was tried, shipped and taken
    back out**, because on the rover it turned this into a pirouette: six arcs
    all within 1.15 m of the rover pointing six different ways, five goals and
    3.9 m of path to finish 0.44 m from the start, still on the same two square
    metres. `docs/decisions/rim-frontiers-are-not-cut-up.md` has the numbers and
    the map they were taken on. What is wanted is a rule that stops one arrival
    retiring a whole rim *without* turning the rim into a handful of sub-metre
    goals, and neither behaviour is that -- so what this checks is the fault as
    it stands, which is the honest thing to have in front of whoever fixes it.

    `explore_sim.py` cannot stand in for any of this: its mapper marks a cell
    free or wall for ever and never leaves unknown beside the rover, so it cannot
    build the geometry at all. Reduced-sight runs of it finish the house at every
    range down to 1.5 m.
    """
    section("a rim of unknown all the way round the rover")
    sys.path.insert(0, HERE)
    saved = os.path.join(HERE, "fixtures", "ringed-2026-09-07.json.gz")
    if not os.path.exists(saved):                       # pragma: no cover
        print("  .... skipped, %s is not here" % saved)
        return
    try:
        import base64
        import gzip
        import frontier
    except ImportError as exc:                          # pragma: no cover
        print("  .... skipped, cannot import: %s" % exc)
        return

    with gzip.open(saved, "rt", encoding="utf-8") as fh:
        snap = json.load(fh)
    raw = base64.b64decode(snap["data"])
    grid = frontier.Grid(snap["width"], snap["height"], snap["resolution"],
                         snap["origin"][0], snap["origin"][1],
                         [v - 256 if v > 127 else v for v in raw])
    where = (snap["pose"][0], snap["pose"][1])

    # The map really is the one described above, checked rather than asserted:
    # a fixture that quietly stopped being the ringed map would leave everything
    # below passing against something else.
    free, unknown = frontier.classify(grid)
    check("the recorded map is the 2 m2 of floor in a 97% unknown grid",
          (sum(free), sum(unknown), len(grid.data)), (850, 26789, 27693))
    start = frontier.standing_on(grid, free, where)
    edge = frontier.frontier_cells(
        grid, free, unknown, frontier.reachable_from(grid, free, start))
    check("...and its whole boundary is one clump, which is not the fault",
          len(frontier.clump(grid, edge)), 1)
    check("...of 10.45 m, far more than one arrival can have dealt with",
          round(len(edge) * grid.resolution, 2), 10.45)

    found, summary = frontier.survey(grid, where)
    check("the whole rim is offered as a single goal", len(found), 1)
    check("...and it is where the rover is already standing, so driving to it "
          "reveals nothing",
          round(math.hypot(found[0]["x"] - where[0],
                           found[0]["y"] - where[1]), 2) <= 0.05, True)
    check("...and the summary counts the one", summary["frontiers"], 1)

    # Driving the policy, which is what makes it a fault rather than an oddity:
    # one arrival ends the run. The map cannot grow here, which is the
    # pessimistic case, since on the rover every arrival redraws it -- and on
    # the rover it did not save the run either.
    explorer = frontier.Explorer()
    at = where
    path_m = 0.0
    goals = 0
    for _ in range(40):
        offered = explorer.choose(grid, at)
        if not offered:
            break
        best = offered[0]
        path_m += math.hypot(best["x"] - at[0], best["y"] - at[1])
        explorer.committed(best["x"], best["y"])
        at = (best["x"], best["y"])
        goals += 1
    check("the run ends after one goal", goals, 1)
    check("...having driven 3.5 cm, with 10.45 m of boundary written off",
          round(path_m, 2), 0.03)
    check("...and the rover still standing on its island",
          round(math.hypot(at[0] - where[0], at[1] - where[1]), 2) <= 0.05,
          True)


def test_a_goal_that_goes_nowhere_is_given_up():
    """The stall watcher, against the drives where this rover really was stuck.

    Not a synthetic spin. `recordings/trap-2026-08-25-spin.json` and
    `corridor-2026-08-25-spin.json` are a minute each of the rover pivoting on
    the spot going nowhere -- the controller aiming it at a point behind a wall,
    which is the README's open fault -- and the two doorway recordings are the
    same rover on the same day driving properly. A watcher that cannot tell those
    apart is worse than none, because it would cancel good drives.

    The rover met this again on 2026-09-01 while exploring: fifty seconds, six
    centimetres, forty-three replans, and not one recovery attempted, because
    `PoseProgressChecker` counts a pivot as progress and so never fired.
    """
    section("a goal that is going nowhere is given up, and a slow one is not")
    sys.path.insert(0, HERE)
    try:
        import frontier
    except ImportError as exc:                          # pragma: no cover
        print("  .... skipped, cannot import frontier: %s" % exc)
        return

    def replay(name, recoveries=0):
        """Drive the watcher down a recorded drive, and say when it gave up.

        **Only over the part of the recording where a goal was actually being
        driven**, which is between the first and last `/cmd_vel_nav` command.
        `nav_record.py` records a fixed sixty seconds whether or not anything is
        happening, so every one of these files ends with the rover sitting idle
        -- and replaying across that tail asks the watcher a question it is never
        asked on the rover, where it only runs while a goal is in flight. Getting
        this wrong makes every recording look like a stall, which is how this
        test first read.

        The map frame rather than odom, because that is the frame `pose()`
        answers in, and because odom drifts while the rover stands still --
        exactly the situation here, and it would read as movement that never
        happened.
        """
        path = os.path.join(HERE, "recordings", name)
        if not os.path.exists(path):
            return "missing"
        with open(path) as fh:
            episode = json.load(fh)
        commands = episode.get("commands") or []
        if not commands:
            return "missing"
        first, last = commands[0]["t"], commands[-1]["t"]
        track = [(p["t"], (p["map"][0], p["map"][1])) for p in episode["poses"]
                 if "map" in p and first <= p["t"] <= last]
        if len(track) < 20:
            return "missing"
        watch = frontier.Stall()
        for when, where in track:
            if watch.update(when, where, recoveries):
                return when - first
        return None

    # The three recordings of the rover failing to get anywhere. Between 1% and
    # 17% of the commands in each have any forward speed at all -- the rest are
    # pivots -- and none of the three ever reached its goal.
    trap = replay("trap-2026-08-25-spin.json")
    check("the 3038-degree spin is given up on", isinstance(trap, float), True)
    check("...and within about half a minute, not after the full allowance",
          isinstance(trap, float) and trap <= 40.0, True)
    check("the other recorded spin is given up on too",
          isinstance(replay("corridor-2026-08-25-spin.json"), float), True)
    check("...and so is the doorway drive that pivoted for a minute and ended "
          "2.4 m short", isinstance(replay("doorway-2026-08-25.json"), float),
          True)

    # And the one recording of the rover driving properly: 86% of its commands
    # have forward speed and it covers 3.5 m in the 9.6 s it is commanded. This
    # is the false positive that would matter, because a watcher that cancels
    # this is a rover that cannot cross a room.
    driving = replay("doorway-2026-08-25-after-floor.json")
    check("the drive that was actually getting somewhere is left alone",
          driving in (None, "missing"), True)

    # Nav2 working its recovery ladder is Nav2 knowing it is stuck, and it is
    # left to get on with it -- the ladder beats this at the thing the ladder is
    # for. What this catches is the case where nothing is being attempted.
    watch = frontier.Stall()
    fired = None
    for tick in range(200):
        # Standing perfectly still, but with a recovery starting every 10 s.
        fired = watch.update(tick * 0.5, (1.0, 1.0), recoveries=tick // 20)
        if fired:
            break
    check("a rover Nav2 is actively recovering is not cancelled underneath it",
          fired, None)

    watch = frontier.Stall()
    fired = None
    for tick in range(200):
        fired = watch.update(tick * 0.5, (1.0, 1.0), recoveries=0)
        if fired:
            break
    check("...but standing still with nothing being attempted is given up on",
          bool(fired), True)
    check("...after the patience, not before",
          frontier.STALL_PATIENCE_S >= 25.0, True)

    # A pose the transform tree could not supply must not read as a rover that
    # has not moved: that is a stall invented out of a dropped lookup.
    watch = frontier.Stall()
    watch.update(0.0, (0.0, 0.0), 0)
    check("a pose nobody could vouch for is not read as standing still",
          watch.update(100.0, None, 0), None)

    source = _bridge_source()
    if source:
        check("exploring passes the watcher to the goal it sends",
              "give_up=going_nowhere" in source, True)
        check("...and only exploring does, so drive_to keeps every recovery",
              source.count("give_up=going_nowhere"), 1)
        check("...and a goal given up on is not reported as having timed out",
              'outcome["reason"] = "blocked"' in source, True)


def test_a_rover_it_cannot_plan_from_is_not_a_finished_house():
    """The refusal that had exploring announce the house was mapped.

    **This is a recording, not a story.** `fixtures/start-occupied.json.gz` is
    the rover's own global costmap, taken off the running planner on
    2026-09-01, and the pose below is the one every refusal in that run named.
    The rover stood 0.156 m from a mapped wall with a 0.200 m footprint, so the
    planner declined to plan from there -- correctly -- and did so for every
    destination it was offered. Exploring read four of those as four
    unreachable frontiers and ended with "everything still unmapped is behind
    something the rover cannot get through", in the same sentence as "73% of the
    map is still unknown".

    Two things have to hold for the fix to mean anything, and both are checked
    against that costmap rather than against a description of it: the pose
    really is one the body does not fit in, and there really is somewhere close
    by that it does. Whether the planner then plans is `plan_bench.py`'s
    question and was answered on the rover -- 0 of 1 start headings from the
    recorded pose, 1 of 1 from the spot found below, same map, same goal.
    """
    section("the rover standing where nothing can plan from")
    sys.path.insert(0, HERE)
    saved = os.path.join(HERE, "fixtures", "start-occupied.json.gz")
    if not os.path.exists(saved):                       # pragma: no cover
        print("  .... skipped, %s is not here" % saved)
        return
    try:
        import base64
        import gzip
        import goal_fit
        import nav_codes
    except ImportError as exc:                          # pragma: no cover
        print("  .... skipped, cannot import: %s" % exc)
        return

    with gzip.open(saved, "rt") as fh:
        snap = json.load(fh)["global_costmap"]
    grid = goal_fit.CostGrid(snap["width"], snap["height"], snap["resolution"],
                             snap["origin"][0], snap["origin"][1],
                             base64.b64decode(snap["data"]))
    # The footprint the costmap node is configured with: `footprint: []` and
    # `robot_radius: 0.200`, read off the running node when this was taken.
    body = goal_fit.polygon_from("[]", 0.200)
    stuck = (-2.30, 1.46)

    check("the recorded pose is one the planner is right to refuse",
          goal_fit.fits(grid, body, stuck[0], stuck[1], 0.0), False)
    check("...because the costmap calls that cell inscribed, not merely near",
          grid.cost(*grid.cell_of(*stuck)) >= goal_fit.INSCRIBED, True)

    out = goal_fit.fit(grid, body, stuck[0], stuck[1], 0.0)
    check("there is somewhere close by the body does fit", out is not None, True)
    check("...and it is a shuffle rather than a journey",
          bool(out) and out["moved_m"] <= 0.5, True)
    check("...which is where the run that gave up would have carried on from",
          bool(out) and goal_fit.fits(grid, body, out["x"], out["y"],
                                      out["yaw"]), True)

    # The distinction the loop now turns on. 208 is NO_VALID_PATH, which really
    # is a verdict on the destination, and putting it in here would put the rover
    # back to shuffling itself over a frontier that is genuinely walled off.
    check("a refusal about the start is told apart from one about the goal",
          205 in nav_codes.ABOUT_THE_ROVER and 208 not in
          nav_codes.ABOUT_THE_ROVER, True)
    check("...and only the occupied one is something moving 30 cm can cure",
          nav_codes.START_OCCUPIED, 205)

    source = _bridge_source()
    if source:
        # Comments stripped for the absence check, so that the account of the
        # fault written above the fix does not read as the fault still being
        # there. What matters is which sentences the code can still produce.
        can_say = " ".join(line.split("#")[0] for line in source.splitlines())
        check("exploring asks the planner why, not just whether",
              "if code in nav_codes.ABOUT_THE_ROVER:" in source, True)
        check("...and it can no longer call four refusals a mapped house",
              "everything still unmapped is behind something" in can_say, False)
        check("...it says what actually happened to the frontiers instead",
              "the rover could not get a route to any of the %d " in can_say,
              True)
        check("...and the back-off goes to where goal_fit says the body fits",
              "goal_fit.fit(grid, body, where[0], where[1], where[2])" in source,
              True)


def test_exploring_finishes_and_covers_the_house():
    """The explore loop, run round the room the recorded drive mapped.

    The question this answers is the one that cannot be answered by reading the
    code: a loop that hands itself new work stops. `explore_sim.py` drives the
    *shipped* policy -- `frontier.Explorer`, the same object the bridge uses --
    round the kitchen-loop floor plan, and the run has to end because it ran out
    of frontiers rather than because it hit the backstop.

    The coverage figure is checked loosely and on purpose. It is an optimistic
    bound: the simulated lidar never misses a chair leg and the simulated
    driving never fails. What would make it meaningless is not being a few
    percent out, it is the run not finishing.
    """
    section("exploring the kitchen-loop house, start to finish")
    sys.path.insert(0, HERE)
    saved = os.path.join(HERE, "fixtures", "kitchen-loop.pgm.gz")
    if not os.path.exists(saved):                       # pragma: no cover
        print("  .... skipped, %s is not here" % saved)
        return
    try:
        import frontier
        import explore_sim
    except ImportError as exc:                          # pragma: no cover
        print("  .... skipped, cannot import explore_sim: %s" % exc)
        return

    room = explore_sim.Room(frontier.read_pgm(saved))
    floor = explore_sim.reachable_floor(room)
    start = room.origin_x + (floor[len(floor) // 2] % room.width + 0.5) \
        * room.resolution, \
        room.origin_y + (floor[len(floor) // 2] // room.width + 0.5) \
        * room.resolution
    result = explore_sim.run(room, start, verbose=False)
    known, total = explore_sim.coverage(room, result["seen"])

    check("the run ends because there is nothing left, not because it gave up",
          result["reason"], "finished")
    check("...and it took a sensible number of goals to do it, not two hundred",
          2 <= result["goals"] <= 40, True)
    check("...and it never offered a frontier its own walk could not reach",
          result["blocked"], 0)
    check("...and it found nearly all the floor there was to find",
          known >= 0.95 * total, True)
    check("...and it did not drive the length of a marathon to do it",
          result["metres"] < 200.0, True)

    # The rule that makes it terminate, checked directly rather than inferred
    # from the run above: a frontier that has been driven to is not offered
    # again, whatever happened when the rover got there.
    explorer = frontier.Explorer()
    explorer.committed(1.0, 1.0)
    check("a frontier that has been driven to is written off, not just a failed "
          "one", explorer.blacklist, [(1.0, 1.0)])
    check("...and the one being driven to is what the next round prefers",
          explorer.previous, (1.0, 1.0))

    # A run that ends before it has looked at the map once still has to be able
    # to say so. This raised KeyError on the rover -- an explore given less
    # clock than one goal needs returned nothing at all, and the caller was left
    # holding an open socket, which reads as a bridge that has hung.
    check("a run that never got to look at the map can still report itself",
          frontier.unknown_share({}), None)
    check("...and one that looked at an empty map does not divide by zero",
          frontier.unknown_share({"free_cells": 0, "unknown_cells": 0}), None)

    # And that the bridge is actually running this policy rather than a second
    # copy of it, which this file cannot check by importing nav_bridge.
    source = _bridge_source()
    if source:
        check("the bridge explores with the shared policy, not its own copy",
              "frontier.Explorer(" in source and "explorer.committed(" in source,
              True)
        check("...and asks the planner for a route before it commits the rover",
              "self.route_to(" in source, True)
        check("...and stops when the stop is latched or a stop was asked for",
              'return totals("stopped"' in source
              and 'return totals("blocked"' in source, True)

        # **The stop that used to be swallowed.** `run_goal` clears `cancelled`
        # as every goal starts, which is right for a move and wrong for a run of
        # them: a stop pressed while `explore` was between goals -- looking at
        # the map, checking the body, asking the planner, which is seconds --
        # was wiped by the next goal and the rover set off again. A counter that
        # nothing clears is what the loop watches instead, and it is checked
        # once more immediately before the goal is sent.
        check("a stop is counted, so no move can clear it by starting",
              "self.stop_seq += 1" in source, True)
        check("...and exploring watches that counter, not just the flag",
              source.count("self.stop_seq != stops") >= 2, True)
        check("...including in the seconds between choosing and setting off",
              "before " in source and "the rover set off" in source, True)


def _ros_messages():
    """Just enough of the ROS message packages for the bridge's moves to import.

    Installed only where the real ones are missing, which is every machine but
    the rover; there the real classes are used and the checks below read the
    same fields off them.
    """
    import types
    try:
        import nav2_msgs.action                         # noqa: F401
        return
    except ImportError:
        pass

    def message(name, **defaults):
        def init(self, **fields):
            for key, value in {**defaults, **fields}.items():
                setattr(self, key, value() if callable(value) else value)
        return type(name, (), {"__init__": init})

    def action(name):
        return type(name, (), {"Goal": message(name + "Goal")})

    header = message("Header", frame_id="", stamp=None)
    pose = message("Pose", position=types.SimpleNamespace,
                   orientation=types.SimpleNamespace)
    modules = {
        "action_msgs": {}, "action_msgs.msg": {"GoalStatus": types.SimpleNamespace(
            STATUS_SUCCEEDED=4, STATUS_CANCELED=5)},
        "builtin_interfaces": {}, "builtin_interfaces.msg": {
            "Duration": message("Duration", sec=0, nanosec=0)},
        "geometry_msgs": {}, "geometry_msgs.msg": {
            "Point": message("Point", x=0.0, y=0.0, z=0.0),
            "PoseStamped": message("PoseStamped", header=header, pose=pose)},
        "nav2_msgs": {}, "nav2_msgs.action": {
            name: action(name) for name in ("BackUp", "ComputePathToPose",
                                            "DriveOnHeading", "NavigateToPose",
                                            "Spin")},
        "nav2_msgs.srv": {"GetCostmap": action("GetCostmap")},
        "rcl_interfaces": {}, "rcl_interfaces.srv": {
            "GetParameters": action("GetParameters")},
    }
    for name, contents in modules.items():
        module = types.ModuleType(name)
        for key, value in contents.items():
            setattr(module, key, value)
        sys.modules[name] = module


def test_the_map_says_what_clearance_the_planner_keeps():
    """The map goes out with the body a walk over it has to respect.

    The autonomy executive decides what it can reach by walking the occupancy
    grid, and walked as a point that went through a 30-40 cm gap the planner
    will not: every frontier a run chose before 2026-10-05 was behind one, and
    every one failed. The fix is two numbers the bridge already holds -- the
    clearance the planner keeps from walls and how far `fit_goal` moves a goal --
    sent with the map, so the executive carries no copy of either.
    """
    section("the map says what clearance the planner keeps")
    sys.path.insert(0, HERE)
    _ros_messages()
    try:
        import types
        import goal_fit
        import nav_moves
    except ImportError as exc:                          # pragma: no cover
        print("  .... skipped, cannot import: %s" % exc)
        return

    check("a bare radius keeps the radius, the inscribed ring the costmap "
          "measured", goal_fit.inscribed_radius("[]", 0.200), 0.200)
    check("...not the twelve-sided stand-in's, which is 3% short",
          round(goal_fit.inscribed_radius("[]", 0.200), 3) > 0.195, True)
    check("a rectangle keeps its half-width",
          round(goal_fit.inscribed_radius(
              "[[0.20, 0.14], [0.20, -0.14], [-0.16, -0.14], [-0.16, 0.14]]",
              0.0), 3), 0.14)
    check("...and no body at all is no answer",
          goal_fit.inscribed_radius("[]", 0.0), None)

    class Bridge(nav_moves.NavMoves):
        def __init__(self, ready):
            self.body = None
            self.inscribed_m = None
            self.asked = 0
            self.footprint_client = types.SimpleNamespace(
                service_is_ready=lambda: ready)

        def footprint(self):
            self.asked += 1
            self.inscribed_m = goal_fit.inscribed_radius("[]", 0.200)
            self.body = goal_fit.polygon_from("[]", 0.200)
            return self.body

    up = Bridge(ready=True)
    check("with the costmap up, the map carries the clearance and the "
          "allowance", up.walking_body(),
          {"inscribed_radius_m": 0.2, "goal_fit_reach_m": goal_fit.REACH_M})
    down = Bridge(ready=False)
    check("with it not up, the map says nothing rather than guess",
          down.walking_body(), {})
    check("...and does not wait on it", down.asked, 0)
    source = _bridge_source()
    if source:
        check("the map reply carries it",
              "**self.walking_body()" in source, True)


def test_a_drive_asked_from_beside_a_wall_backs_off_and_goes():
    """A `drive_to` from a spot the planner will not plan from frees itself once.

    **Reproduced from the record before it was fixed.** On 2026-10-05, in M3
    session 1, a look left the rover 0.2 m from a wall and navigation refused
    the run's next three drives with START_OCCUPIED -- "turn on the spot or back
    up, then try again" -- which a run cannot do: its only moves are `drive_to`,
    a look and a stop. Three refusals in a row ended the run, and one turn by
    hand freed it. Exploring has answered the same refusal with `back_off` since
    2026-09-01; `goto` handed it back to the caller instead.

    Driven here on the costmap taken off the rover in the same state on
    2026-09-01 -- a 0.200 m body 0.156 m from a mapped wall -- through the real
    `goto` and `back_off`, with only Nav2 itself replaced: a goal is refused
    with START_OCCUPIED exactly when the body does not fit where it stands.
    """
    section("a drive asked for from beside a wall backs off and then goes")
    sys.path.insert(0, HERE)
    saved = os.path.join(HERE, "fixtures", "start-occupied.json.gz")
    if not os.path.exists(saved):                       # pragma: no cover
        print("  .... skipped, %s is not here" % saved)
        return
    _ros_messages()
    try:
        import base64
        import gzip
        import threading
        import types
        import goal_fit
        import nav_codes
        import nav_explore
        import nav_moves
    except ImportError as exc:                          # pragma: no cover
        print("  .... skipped, cannot import: %s" % exc)
        return

    with gzip.open(saved, "rt") as fh:
        snap = json.load(fh)["global_costmap"]
    grid = goal_fit.CostGrid(snap["width"], snap["height"], snap["resolution"],
                             snap["origin"][0], snap["origin"][1],
                             base64.b64decode(snap["data"]))
    body = goal_fit.polygon_from("[]", 0.200)
    stuck = (-2.30, 1.46, 0.0)

    class Wedged(nav_moves.NavMoves, nav_explore.NavExplore):
        """The bridge's own moves on a rover whose Nav2 is replaced by the
        costmap: a route is refused from where the body does not fit."""

        def __init__(self, refuse_always=False):
            self.at = list(stuck)
            self.sent = []
            self.stop_seq = 3
            self.refuse_always = refuse_always
            self._lock = threading.Lock()
            self.plan = None
            self.args = types.SimpleNamespace(map_frame="map")

        def pose(self):
            return tuple(self.at)

        def footprint(self):
            return body

        def costmap(self):
            return grid

        def get_clock(self):
            stamp = types.SimpleNamespace(to_msg=lambda: None)
            return types.SimpleNamespace(now=lambda: stamp)

        def run_goal(self, kind, goal, limit_s, say, measure, motion="driving",
                     budget=None, give_up=None, guard=None):
            self.sent.append((kind, guard))
            blocked = nav_moves.autonomy_guard.refusal(
                guard, self.stop_seq, pose=self.pose())
            if blocked:
                return {"reason": "stopped", "travelled_m": 0.0,
                        "turned_deg": 0.0, "detail": blocked}
            x, y, yaw = self.at
            if kind == "spin":
                self.at[2] = yaw + goal.target_yaw
                return {"reason": "arrived", "travelled_m": 0.0,
                        "turned_deg": math.degrees(goal.target_yaw)}
            if kind == "forward":
                step = goal.target.x
                self.at[:2] = [x + step * math.cos(yaw), y + step * math.sin(yaw)]
                return {"reason": "arrived", "travelled_m": step,
                        "turned_deg": 0.0}
            if self.refuse_always or not goal_fit.fits(grid, body, x, y, yaw):
                return {"reason": "blocked", "code": nav_codes.START_OCCUPIED,
                        "travelled_m": 0.0, "turned_deg": 0.0,
                        "detail": nav_codes.phrase_for(nav_codes.START_OCCUPIED)}
            to = goal.pose.pose.position
            self.at[:2] = [to.x, to.y]
            return {"reason": "arrived", "travelled_m": math.hypot(to.x - x, to.y - y),
                    "turned_deg": 0.0}

    quiet = lambda *args, **kwargs: None                # noqa: E731
    # A goal two metres away in open floor, the kind a run asks for.
    goal = (-0.30, 1.46)
    rover = Wedged()
    out = rover.goto(goal, None, quiet)
    kinds = [kind for kind, _ in rover.sent]
    check("the drive arrives rather than handing the refusal back",
          out.get("reason"), "arrived")
    check("...by asking once, backing off, and asking again",
          kinds, ["goto", "spin", "forward", "goto"])
    check("...and it says it backed off, so the move is accounted for",
          "backed off" in (out.get("detail") or ""), True)
    shuffle = rover.sent and goal_fit.fit(grid, body, stuck[0], stuck[1], stuck[2])
    check("...the back-off is the short shuffle goal_fit names, not a journey",
          bool(shuffle) and shuffle["moved_m"] <= 0.5, True)

    rover = Wedged(refuse_always=True)
    out = rover.goto(goal, None, quiet)
    check("a second refusal after backing off is handed back, not looped on",
          [kind for kind, _ in rover.sent], ["goto", "spin", "forward", "goto"])
    check("...as the refusal it is", out.get("reason"), "blocked")

    # An autonomous drive carries its run's guard into the back-off: a stop
    # since the goal was issued, or a back-off spot outside the safe area, ends
    # it before the wheels turn.
    rover = Wedged()
    away = goal_fit.fit(grid, body, stuck[0], stuck[1], stuck[2])
    dx, dy = stuck[0] - away["x"], stuck[1] - away["y"]
    norm = math.hypot(dx, dy)
    centre = (stuck[0] + dx / norm * 0.5, stuck[1] + dy / norm * 0.5)
    fence = {"x_m": centre[0], "y_m": centre[1],
             "radius_m": 0.5 + nav_moves.autonomy_guard.FENCE_MARGIN_M + 0.1}
    guard = {"stop_seq": 3, "geofence": fence}
    check("the fence used here holds the rover and excludes the back-off spot",
          (nav_moves.autonomy_guard.refusal(guard, 3, pose=stuck[:2]) == "",
           bool(nav_moves.autonomy_guard.refusal(
               guard, 3, pose=stuck[:2], goal=(away["x"], away["y"])))),
          (True, True))
    out = rover.back_off(quiet, guard=guard)
    check("...and a back-off that would leave the safe area does not move",
          ([kind for kind, _ in rover.sent], tuple(rover.at)), ([], stuck))
    check("...and says so, as a refusal rather than a stop",
          (out.get("reason"), "safe-area" in (out.get("detail") or "")),
          ("blocked", True))

    class StoppedMeanwhile(Wedged):
        """A stop arrives at the moment the first route is refused."""

        def run_goal(self, kind, *args, **kwargs):
            out = Wedged.run_goal(self, kind, *args, **kwargs)
            if kind == "goto" and out.get("code") == nav_codes.START_OCCUPIED:
                self.stop_seq += 1
            return out

    rover = StoppedMeanwhile()
    rover.goto(goal, None, quiet, guard={"stop_seq": 3, "geofence": None})
    check("a stop that lands with the refusal keeps the back-off from moving",
          [kind for kind, _ in rover.sent], ["goto"])
    check("...so the rover is where it was", tuple(rover.at), stuck)


def test_a_near_goal_is_a_turn_a_line_and_a_turn():
    """A goal close by is faced, driven to straight, and then turned to.

    **Reproduced on the rover before it was changed.** In M3 session 7 on
    2026-10-07 a drive to a viewpoint 0.4 m away, facing -70 degrees, swung
    between 143 and 167 degrees for 72 s until it was stopped. Asked again from
    the same spot, the live planner drew a 3.3-4.0 m loop from every start
    heading between 130 and 173; the real controller, given it, preferred
    turning on the spot by 45 points to 67; turned to face the goal, the same
    goal planned as 0.48 m straight -- but only facing within about 30 degrees
    of it, and only with a final heading within about 30 degrees of the way it
    travelled. Nav2 is replaced here by exactly that rule.
    """
    section("a near goal is a turn, a straight line and a turn")
    sys.path.insert(0, HERE)
    _ros_messages()
    try:
        import threading
        import types
        import nav_moves
    except ImportError as exc:                          # pragma: no cover
        print("  .... skipped, cannot import: %s" % exc)
        return
    wrap = nav_moves.wrap

    class Floor(nav_moves.NavMoves):
        """Open floor, and the planner/controller rule measured that day."""

        def __init__(self, at, pivots=True):
            self.at = [at[0], at[1], math.radians(at[2])]
            self.pivots = pivots
            self.sent = []
            self.stop_seq = 3
            self._lock = threading.Lock()
            self.plan = None
            self.args = types.SimpleNamespace(map_frame="map")

        def pose(self):
            return tuple(self.at)

        def footprint(self):
            return None

        def get_clock(self):
            stamp = types.SimpleNamespace(to_msg=lambda: None)
            return types.SimpleNamespace(now=lambda: stamp)

        blocked_asks = 0

        def route_to(self, gx, gy, yaw):
            # Someone standing in the way for the first `blocked_asks` asks.
            straight = math.hypot(gx - self.at[0], gy - self.at[1])
            if self.blocked_asks:
                self.blocked_asks -= 1
                return (straight + 3.0, 180.0), 0
            return (straight, 0.0), 0

        def run_goal(self, kind, goal, limit_s, say, measure, motion="driving",
                     budget=None, give_up=None, guard=None):
            x, y, h = self.at
            if kind == "spin":
                self.sent.append(("spin", math.degrees(goal.target_yaw)))
                if not self.pivots:
                    return {"reason": "blocked", "travelled_m": 0.0,
                            "turned_deg": 0.0, "detail": "no room to turn"}
                self.at[2] = wrap(h + goal.target_yaw)
                return {"reason": "arrived", "travelled_m": 0.0,
                        "turned_deg": math.degrees(goal.target_yaw)}
            to, turn = goal.pose.pose.position, goal.pose.pose.orientation
            yaw = 2.0 * math.atan2(turn.z, turn.w)
            self.sent.append(("goto", math.degrees(yaw), give_up is not None))
            bearing = math.atan2(to.y - y, to.x - x)
            if (abs(math.degrees(wrap(bearing - h))) > 30.0
                    or abs(math.degrees(wrap(yaw - bearing))) > 30.0):
                # The loop the controller will not drive: it swings on the spot,
                # and only the give-up can end it.
                why = ""
                for second in range(0, 80, 5):
                    why = give_up(float(second), {"recoveries": 0}) if give_up else ""
                    if why:
                        break
                return {"reason": "blocked" if why else "timeout",
                        "travelled_m": 0.1, "turned_deg": 0.0,
                        "detail": why or "it swung on the spot until it ran out of time"}
            self.at = [to.x, to.y, yaw]
            return {"reason": "arrived",
                    "travelled_m": math.hypot(to.x - x, to.y - y),
                    "turned_deg": math.degrees(wrap(yaw - h))}

    def steps(rover):
        # To the nearest degree, which is all any of these claims is about.
        return [(kind, int(math.floor(angle + 0.5)))
                for kind, angle, *_ in rover.sent]

    quiet = lambda *args, **kwargs: None                # noqa: E731
    stuck = (-17.45, -14.65, 150.0)
    goal = (-17.2, -15.055)

    rover = Floor(stuck)
    out = rover.goto(goal, -69.5, quiet, near=False)
    check("session 7's goal sent as one goal swings on the spot, as it did",
          out.get("reason") != "arrived", True)
    check("...and is ended by the stall watch, not left to swing",
          "turning on the spot" in (out.get("detail") or ""), True)

    rover = Floor(stuck)
    out = rover.goto(goal, -69.5, quiet)
    check("as a near goal it arrives", out.get("reason"), "arrived")
    check("...by facing it, then driving straight, the heading close enough",
          steps(rover), [("spin", 152), ("goto", -58)])
    check("...at the place asked for",
          [round(v, 2) for v in rover.pose()[:2]], [-17.2, -15.05])
    check("...and counts the turn as well as the drive",
          (round(out["travelled_m"], 2), round(out["turned_deg"])), (0.48, 152))

    rover = Floor(stuck)
    out = rover.goto(goal, 30.0, quiet)
    check("a heading well off the way it travelled is turned to afterwards",
          steps(rover), [("spin", 152), ("goto", -58), ("spin", 88)])
    check("...and it ends facing it",
          round(math.degrees(rover.pose()[2])), 30)

    rover = Floor((-17.45, -14.65, -50.0))
    rover.goto(goal, -60.0, quiet)
    check("a near goal it already faces is one straight drive",
          steps(rover), [("goto", -58)])

    rover = Floor(stuck)
    rover.goto(goal, None, quiet)
    check("a click with no heading is one goal, left to Nav2 as before",
          steps(rover), [("goto", -58)])

    rover = Floor((-17.25, -15.0, 150.3))
    out = rover.goto(goal, -69.5, quiet)
    check("one it is already standing on is only its heading",
          (steps(rover), out.get("reason")), ([("spin", 140)], "arrived"))

    rover = Floor(stuck, pivots=False)
    out = rover.goto(goal, -69.5, quiet)
    check("where it cannot turn, it falls back to the one goal it always sent",
          steps(rover), [("spin", 152), ("goto", -69)])

    # Somebody stands in the way after it has turned to face the goal.
    saved_sleep = nav_moves.time.sleep
    nav_moves.time.sleep = lambda seconds: None
    try:
        rover = Floor(stuck)
        rover.blocked_asks = 2
        out = rover.goto(goal, -69.5, quiet)
        check("a near goal whose way is blocked waits, then drives once it clears",
              (steps(rover), out.get("reason")),
              ([("spin", 152), ("goto", -58)], "arrived"))
        rover = Floor(stuck)
        rover.blocked_asks = 99
        out = rover.goto(goal, -69.5, quiet)
        check("...and one that stays blocked is handed back, not driven round",
              (steps(rover), out.get("reason")), ([("spin", 152)], "blocked"))
        check("...saying something is in the way",
              "something is in the way" in (out.get("detail") or ""), True)
        rover = Floor(stuck)
        rover.blocked_asks = 99
        rover.route_to = lambda *a: (None, None)
        rover.goto(goal, -69.5, quiet)
        check("...and a planner that does not answer leaves it to Nav2 as before",
              steps(rover), [("spin", 152), ("goto", -58)])
    finally:
        nav_moves.time.sleep = saved_sleep

    # Going round a person who stays put: a 0.2 m blob half way to a goal
    # 0.7 m ahead, on an otherwise empty costmap, and a planner whose route
    # goes round it. The rover turns and drives straight legs, never a curve.
    import goal_fit
    width = 60
    cells = bytearray(width * width)
    grid = goal_fit.CostGrid(width, width, 0.05, -1.5, -1.5, bytes(cells))
    for col in range(width):
        for row in range(width):
            x, y = -1.5 + (col + 0.5) * 0.05, -1.5 + (row + 0.5) * 0.05
            if 0.25 <= x <= 0.45 and -0.1 <= y <= 0.1:
                cells[row * width + col] = goal_fit.LETHAL
    grid = goal_fit.CostGrid(width, width, 0.05, -1.5, -1.5, bytes(cells))
    body = goal_fit.polygon_from("[]", 0.200)
    around = [(0.0, 0.0)] + [(0.0, 0.1 * i) for i in range(1, 6)] + [
        (0.1 * i, 0.5) for i in range(1, 8)] + [(0.7, 0.5 - 0.1 * i) for i in range(1, 6)]

    class Blocked(Floor):
        def footprint(self):
            return body

        def costmap(self):
            return grid

        def route_to(self, gx, gy, yaw):
            self.last_route = around
            return (2.4, 270.0), 0

        def run_goal(self, kind, goal, limit_s, say, measure, motion="driving",
                     budget=None, give_up=None, guard=None):
            if kind == "forward":
                x, y, h = self.at
                step = goal.target.x
                self.sent.append(("forward", step * 100))
                self.at[:2] = [x + step * math.cos(h), y + step * math.sin(h)]
                return {"reason": "arrived", "travelled_m": step, "turned_deg": 0.0}
            return Floor.run_goal(self, kind, goal, limit_s, say, measure,
                                  motion=motion, budget=budget, give_up=give_up,
                                  guard=guard)

    legs = goal_fit.straight_legs(grid, body, (0.0, 0.0), around)
    ends = [(0.0, 0.0)] + list(legs or [])
    check("the legs round the blob are a few straight lines the body fits down, "
          "ending at the goal",
          (bool(legs) and len(legs) <= 4,
           [round(v, 2) for v in (legs or [(None, None)])[-1]],
           all(goal_fit.line_fits(grid, body, a, b) for a, b in zip(ends, ends[1:]))),
          (True, [0.7, 0.0], True))
    check("...cutting the corners the route went round",
          len(legs or []) < 3 or sum(math.hypot(b[0] - a[0], b[1] - a[1])
                                     for a, b in zip(ends, ends[1:])) < 2.4, True)
    check("...and a route straight through it is no legs at all",
          goal_fit.straight_legs(grid, body, (0.0, 0.0), [(0.7, 0.0)]), None)

    saved_sleep = nav_moves.time.sleep
    nav_moves.time.sleep = lambda seconds: None
    try:
        rover = Blocked((0.0, 0.0, 0.0))
        out = rover.goto((0.7, 0.0), 0.0, quiet)
    finally:
        nav_moves.time.sleep = saved_sleep
    kinds = [kind for kind, _ in steps(rover)]
    check("a near goal that stays blocked is gone round, in straight legs",
          (out.get("reason"), "goto" in kinds, kinds.count("forward"),
           all(a != b for a, b in zip(kinds, kinds[1:]))),
          ("arrived", False, len(legs), True))
    check("...ending where it was asked to, facing the way asked",
          ([round(v, 2) for v in rover.pose()[:2]], round(math.degrees(rover.pose()[2]))),
          ([0.7, 0.0], 0))
    check("...and saying so",
          "went round something in the way" in (out.get("detail") or ""), True)

    rover = Floor((-19.45, -14.65, 0.0))
    rover.goto((-17.2, -14.65), 0.0, quiet)
    check("a far goal is still one goal, with the heading asked for",
          steps(rover), [("goto", 0)])
    check("...and every goal now carries a give-up",
          all(step[2] for step in rover.sent if step[0] == "goto"), True)


TESTS = (
    test_goal_fits_before_it_is_sent,
    test_frontiers_are_found_on_a_real_map,
    test_a_rim_of_unknown_round_the_rover_is_still_open,
    test_a_goal_that_goes_nowhere_is_given_up,
    test_a_rover_it_cannot_plan_from_is_not_a_finished_house,
    test_the_map_says_what_clearance_the_planner_keeps,
    test_a_drive_asked_from_beside_a_wall_backs_off_and_goes,
    test_a_near_goal_is_a_turn_a_line_and_a_turn,
    test_exploring_finishes_and_covers_the_house,
)
