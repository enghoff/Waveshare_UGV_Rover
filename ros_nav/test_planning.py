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


TESTS = (
    test_goal_fits_before_it_is_sent,
    test_frontiers_are_found_on_a_real_map,
    test_a_goal_that_goes_nowhere_is_given_up,
    test_a_rover_it_cannot_plan_from_is_not_a_finished_house,
    test_exploring_finishes_and_covers_the_house,
)
