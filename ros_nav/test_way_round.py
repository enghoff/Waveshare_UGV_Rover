"""Something in the way of a near goal: seen on the live scan, waited for, gone round.

Until 2026-10-08 the planner's costmap was the map and nothing else, so a
person standing in front of the rover was only on the controller's: the live
scan's local costmap. Most checks here put a person there and nowhere else,
which is what the rover had in M3 session 13 on 2026-10-07, and is still what
a near goal works from; the planner's live layer (config/nav2.yaml) puts them
on its costmap too, and the checks that say so are the ones about that.
"""
import math
import sys

from test_harness import HERE, check, section

#: The rover's footprint (config/nav2.yaml): 0.20 m ahead, 0.16 m behind,
#: 0.14 m to each side.
BODY = [(0.20, 0.14), (0.20, -0.14), (-0.16, -0.14), (-0.16, 0.14)]


def _grid(goal_fit, centre, people, half_m=1.5, res=0.05):
    """A live costmap: a square window round `centre`, lethal where `people`
    stand -- 0.2 m across each -- and empty everywhere else."""
    width = int(round(2 * half_m / res))
    ox, oy = centre[0] - half_m, centre[1] - half_m
    cells = bytearray(width * width)
    for col in range(width):
        for row in range(width):
            x, y = ox + (col + 0.5) * res, oy + (row + 0.5) * res
            if any(math.hypot(x - px, y - py) <= 0.1 for px, py in people):
                cells[row * width + col] = goal_fit.LETHAL
    return goal_fit.CostGrid(width, width, res, ox, oy, bytes(cells))


def _to_odom(correction, point):
    """The map -> odom correction undone, worked out again here so the room
    below does not lean on the code it is checking."""
    dx, dy = point[0] - correction[0], point[1] - correction[1]
    c, s = math.cos(correction[2]), math.sin(correction[2])
    return (c * dx + s * dy, -s * dx + c * dy)


def _clearance(a, b, person):
    """How near the rover's centre passes a point on the way from a to b."""
    length = math.hypot(b[0] - a[0], b[1] - a[1]) or 1e-9
    share = max(0.0, min(1.0, ((person[0] - a[0]) * (b[0] - a[0])
                               + (person[1] - a[1]) * (b[1] - a[1])) / length ** 2))
    return math.hypot(a[0] + (b[0] - a[0]) * share - person[0],
                      a[1] + (b[1] - a[1]) * share - person[1])


def test_a_way_round_a_person_is_found_on_the_live_scan():
    section("a way round a person, found on the live scan alone")
    sys.path.insert(0, HERE)
    import goal_fit

    person = (0.5, 0.0)
    grid = _grid(goal_fit, (0.0, 0.0), [person])
    check("the straight line through them does not fit, lethal cells only",
          goal_fit.line_fits(grid, BODY, (0.0, 0.0), (1.0, 0.0),
                             worst=goal_fit.LETHAL, skip_m=0.1), False)
    check("...and one beside them does",
          goal_fit.line_fits(grid, BODY, (0.0, 0.5), (1.0, 0.5),
                             worst=goal_fit.LETHAL, skip_m=0.1), True)

    legs = goal_fit.legs_round(grid, (0.0, 0.0), (1.0, 0.0), 0.30, 0.14, 0.30)
    ends = [(0.0, 0.0)] + list(legs or [])
    check("there is a way round, in at most four straight legs, ending at the goal",
          (bool(legs) and len(legs) <= 4,
           [round(v, 2) for v in (legs or [(None, None)])[-1]]),
          (True, [1.0, 0.0]))
    check("...that keeps the rover's centre 0.25 m or more from them",
          min(_clearance(a, b, person) for a, b in zip(ends, ends[1:])) >= 0.25,
          True)
    check("...and is not much further than the straight line",
          sum(math.hypot(b[0] - a[0], b[1] - a[1])
              for a, b in zip(ends, ends[1:])) < 2.0, True)

    check("someone standing on the goal has no way round them",
          goal_fit.legs_round(_grid(goal_fit, (0.0, 0.0), [(1.0, 0.0)]),
                              (0.0, 0.0), (1.0, 0.0), 0.30, 0.14, 0.30), None)
    check("nobody there is one leg, straight to the goal",
          goal_fit.legs_round(_grid(goal_fit, (0.0, 0.0), []),
                              (0.0, 0.0), (1.0, 0.0), 0.30, 0.14, 0.30),
          [(1.0, 0.0)])
    wall = [(x * 0.05, y) for x in range(-30, 31) for y in (0.35, -0.35)]
    check("a person in a corridor too narrow to pass them has no way round",
          goal_fit.legs_round(_grid(goal_fit, (0.0, 0.0), [person] + wall),
                              (0.0, 0.0), (1.0, 0.0), 0.30, 0.14, 0.30), None)


def test_a_person_stepping_in_front_of_a_near_goal_is_waited_for():
    """Session 13, as the rover had it.

    A look 1.0 m away, from (-17.19, -15.94) to (-17.23, -14.93). The rover
    faced it and set off; the owner stepped in front. The planner, whose map
    has no person on it, kept the straight line; the controller, whose live
    costmap does, would not drive into them and turned back and forth on the
    spot until the stall watch ended the goal 25 s later.
    """
    section("a person stepping in front of a near goal")
    sys.path.insert(0, HERE)
    from test_planning import _ros_messages
    _ros_messages()
    try:
        import threading
        import types
        import goal_fit
        import nav_moves
    except ImportError as exc:                          # pragma: no cover
        print("  .... skipped, cannot import: %s" % exc)
        return
    wrap = nav_moves.wrap
    start = (-17.19, -15.94)
    goal = (-17.23, -14.93)
    bearing = math.degrees(math.atan2(goal[1] - start[1], goal[0] - start[0]))
    # Where the owner stood: 0.6 m on, 0.4 m in front of where the rover had
    # got to when they stepped in.
    owner = (start[0] + (goal[0] - start[0]) * 0.6, start[1] + (goal[1] - start[1]) * 0.6)
    # odom has drifted from the map, as it always has on the rover.
    correction = (0.4, -0.2, math.radians(17.0))

    class Room(nav_moves.NavMoves):
        """Open floor, a planner that cannot see people and a controller that
        can, and an owner who steps in front after `steps_in_s` of driving and
        steps away after `stays_s` of the rover waiting."""

        def __init__(self, steps_in_s=1, stays_s=99.0):
            self.at = [start[0], start[1], math.radians(bearing)]
            self.sent, self.said = [], []
            self.stop_seq = 3
            self._lock = threading.Lock()
            self.plan = None
            self.args = types.SimpleNamespace(map_frame="map")
            self.owner_there = steps_in_s == 0
            self.stepped_in = self.owner_there
            self.steps_in_s, self.stays_s = steps_in_s, stays_s
            self.waited_s = 0.0
            self.stopped_after_s = None
            self.nearest = 9.0

        def pose(self):
            return tuple(self.at)

        def footprint(self):
            return BODY

        def costmap(self):
            return None

        def correction(self):
            return correction

        def get_clock(self):
            stamp = types.SimpleNamespace(to_msg=lambda: None)
            return types.SimpleNamespace(now=lambda: stamp)

        def live_costmap(self):
            here = _to_odom(correction, self.at)
            people = [_to_odom(correction, owner)] if self.owner_there else []
            return _grid(goal_fit, here, people), correction

        def route_to(self, gx, gy, yaw):
            # The planner's map has no owner on it: always the straight line.
            self.last_route = [tuple(self.at[:2]), (gx, gy)]
            return (math.hypot(gx - self.at[0], gy - self.at[1]), 0.0), 0

        def sleep(self, seconds):
            self.waited_s += seconds
            if self.waited_s >= self.stays_s:
                self.owner_there = False

        def on_the_line(self):
            # What the controller refuses: a move that would put the body on
            # a cell the live scan hit.
            if not self.owner_there:
                return False
            grid, _ = self.live_costmap()
            return not goal_fit.line_fits(
                grid, BODY, _to_odom(correction, self.at), _to_odom(correction, goal),
                worst=goal_fit.LETHAL, skip_m=0.1)

        def run_goal(self, kind, goal_msg, limit_s, say, measure, motion="driving",
                     budget=None, give_up=None, guard=None):
            x, y, h = self.at
            if kind == "spin":
                self.sent.append(("spin", round(math.degrees(goal_msg.target_yaw))))
                self.at[2] = wrap(h + goal_msg.target_yaw)
                return {"reason": "arrived", "travelled_m": 0.0,
                        "turned_deg": math.degrees(goal_msg.target_yaw)}
            if kind == "forward":
                step = goal_msg.target.x
                self.sent.append(("forward", round(step, 2)))
                end = (x + step * math.cos(h), y + step * math.sin(h))
                if self.owner_there:
                    self.nearest = min(self.nearest, _clearance((x, y), end, owner))
                self.at[:2] = list(end)
                return {"reason": "arrived", "travelled_m": step, "turned_deg": 0.0}
            to = goal_msg.pose.pose.position
            self.sent.append(("goto",))
            travelled = 0.0
            for second in range(0, 80):
                why = give_up(float(second), {"recoveries": 0}) if give_up else ""
                if why:
                    self.stopped_after_s = second - self.steps_in_s
                    return {"reason": "blocked", "travelled_m": travelled,
                            "turned_deg": 0.0, "detail": why}
                if second == self.steps_in_s and not self.stepped_in:
                    self.owner_there = self.stepped_in = True
                if self.on_the_line():
                    continue                # back and forth on the spot
                left = math.hypot(to.x - self.at[0], to.y - self.at[1])
                hop = min(0.2, left)
                heading = math.atan2(to.y - self.at[1], to.x - self.at[0])
                self.at = [self.at[0] + hop * math.cos(heading),
                           self.at[1] + hop * math.sin(heading), heading]
                travelled += hop
                if left - hop < 0.05:
                    return {"reason": "arrived", "travelled_m": travelled,
                            "turned_deg": 0.0}
            return {"reason": "timeout", "travelled_m": travelled, "turned_deg": 0.0,
                    "detail": "it swung on the spot until it ran out of time"}

    def say(phase, why="", **fields):
        rover.said.append((phase, why))

    saved_sleep = nav_moves.time.sleep
    try:
        rover = Room()
        nav_moves.time.sleep = rover.sleep
        out = rover.goto(goal, bearing, say)
        check("stepped in front of once it has set off, it stops within two "
              "seconds, not the stall watch's 25",
              rover.stopped_after_s is not None and rover.stopped_after_s <= 2, True)
        check("...waits for them, still",
              any(phase == "waiting" for phase, _ in rover.said), True)
        check("...then, as they stay, goes round them in straight legs and arrives",
              (out.get("reason"), "went round" in (out.get("detail") or ""),
               [kind for kind, *_ in rover.sent].count("goto"),
               any(kind == "forward" for kind, *_ in rover.sent)),
              ("arrived", True, 1, True))
        check("...passing them with its centre 0.25 m or more away",
              rover.nearest >= 0.25, True)
        check("...ending at the goal",
              math.hypot(rover.at[0] - goal[0], rover.at[1] - goal[1]) < 0.05, True)

        rover = Room(stays_s=2.0)
        nav_moves.time.sleep = rover.sleep
        out = rover.goto(goal, bearing, say)
        check("one who steps away again is waited for, then driven on to",
              (out.get("reason"), [kind for kind, *_ in rover.sent]),
              ("arrived", ["goto", "goto"]))

        rover = Room(steps_in_s=0)
        nav_moves.time.sleep = rover.sleep
        out = rover.goto(goal, bearing, say)
        check("one already standing there before it sets off is never driven at",
              (out.get("reason"), "goto" in [kind for kind, *_ in rover.sent]),
              ("arrived", False))
        check("...and is gone round, 0.25 m or more away",
              rover.nearest >= 0.25, True)

        rover = Room(steps_in_s=99)
        nav_moves.time.sleep = rover.sleep
        out = rover.goto(goal, bearing, say)
        check("with nobody there it is one straight drive, as before",
              (out.get("reason"), rover.sent, rover.said.count(("waiting", ""))),
              ("arrived", [("goto",)], 0))
    finally:
        nav_moves.time.sleep = saved_sleep


def test_a_person_in_the_way_of_a_longer_goal_is_waited_for_and_gone_round():
    """M4 session 4, 2026-10-08, as the rover had it.

    A geometry goal 1.8 m away, from (-18.32, -13.79) to (-18.57, -15.59) --
    further than `NEAR_GOAL_M`, so one Nav2 goal with nothing watching the live
    scan. The owner stood in the way. The planner, whose map has no person on
    it, sent the same path every second for 27 s, and the controller turned on
    the spot until they moved.
    """
    section("a person in the way of a goal further than a near one")
    sys.path.insert(0, HERE)
    from test_planning import _ros_messages
    _ros_messages()
    try:
        import threading
        import types
        import goal_fit
        import nav_moves
    except ImportError as exc:                          # pragma: no cover
        print("  .... skipped, cannot import: %s" % exc)
        return
    wrap = nav_moves.wrap
    start = (-18.32, -13.79)
    goal = (-18.57, -15.59)
    bearing = math.degrees(math.atan2(goal[1] - start[1], goal[0] - start[0]))
    owner = (start[0] + (goal[0] - start[0]) * 0.5, start[1] + (goal[1] - start[1]) * 0.5)
    correction = (0.4, -0.2, math.radians(17.0))
    #: Where Nav2's straight drive stops for the owner: the body's furthest
    #: corner, 0.244 m from the centre, and a few centimetres of a person's
    #: scan returning wider than their middle.
    DRIVE_STOPS_M = 0.32
    #: How far the rover runs on at 0.41 m/s once a goal is stopped.
    STOP_SLIDE_M = 0.2

    class Room(nav_moves.NavMoves):
        """The near-goal room above, for a longer goal: the planner's route is
        the straight line to whatever it was asked for, the controller swings
        on the spot while the owner is on its next stretch, and a straight
        drive stops short of brushing them."""

        def __init__(self, steps_in_s=1, stays_s=99.0, owner_at=owner,
                     begin=start, speed_ms=0.2):
            self.at = [begin[0], begin[1], math.radians(bearing)]
            self.speed_ms = speed_ms
            self.sent, self.said = [], []
            self.stop_seq = 3
            self._lock = threading.Lock()
            self.plan = None
            self.args = types.SimpleNamespace(map_frame="map")
            self.owner = owner_at
            self.owner_there = steps_in_s == 0
            self.stepped_in = self.owner_there
            self.steps_in_s, self.stays_s = steps_in_s, stays_s
            self.waited_s = 0.0
            self.swung_s = 0
            self.stopped_after_s = None
            self.nearest = 9.0
            self.heading_to = goal

        def pose(self):
            return tuple(self.at)

        def footprint(self):
            return BODY

        def costmap(self):
            return None

        def mapped_walls(self):
            return None

        def correction(self):
            return correction

        def get_clock(self):
            stamp = types.SimpleNamespace(to_msg=lambda: None)
            return types.SimpleNamespace(now=lambda: stamp)

        def live_costmap(self):
            here = _to_odom(correction, self.at)
            people = [_to_odom(correction, self.owner)] if self.owner_there else []
            return _grid(goal_fit, here, people), correction

        def route_to(self, gx, gy, yaw):
            self.last_route = [tuple(self.at[:2]), (gx, gy)]
            return (math.hypot(gx - self.at[0], gy - self.at[1]), 0.0), 0

        def route_points(self):
            # What Nav2 would be following: the straight line, its map has
            # nobody on it.
            return [tuple(self.at[:2]), self.heading_to]

        def sleep(self, seconds):
            self.waited_s += seconds
            if self.waited_s >= self.stays_s:
                self.owner_there = False

        def on_the_line(self, to):
            # What the controller refuses: the next 0.6 m of its way, which is
            # about as far as its rollouts reach.
            if not self.owner_there:
                return False
            grid, _ = self.live_costmap()
            left = math.hypot(to[0] - self.at[0], to[1] - self.at[1]) or 1e-9
            share = min(1.0, 0.6 / left)
            near = (self.at[0] + (to[0] - self.at[0]) * share,
                    self.at[1] + (to[1] - self.at[1]) * share)
            return not goal_fit.line_fits(
                grid, BODY, _to_odom(correction, self.at), _to_odom(correction, near),
                worst=goal_fit.LETHAL, skip_m=0.1)

        def run_goal(self, kind, goal_msg, limit_s, say, measure, motion="driving",
                     budget=None, give_up=None, guard=None):
            x, y, h = self.at
            if kind == "spin":
                self.sent.append(("spin", round(math.degrees(goal_msg.target_yaw))))
                self.at[2] = wrap(h + goal_msg.target_yaw)
                return {"reason": "arrived", "travelled_m": 0.0,
                        "turned_deg": math.degrees(goal_msg.target_yaw)}
            if kind == "forward":
                step = goal_msg.target.x
                self.sent.append(("forward", round(step, 2)))
                end = (x + step * math.cos(h), y + step * math.sin(h))
                if self.owner_there and _clearance((x, y), end, self.owner) < DRIVE_STOPS_M:
                    # Nav2's drive checks its own way and stops short of
                    # anything the body would touch, which on the rover on
                    # 2026-10-08 left it stopped beside the owner: move to the
                    # last point that keeps the clearance and stop there.
                    went = 0.0
                    while went + 0.02 <= step and _clearance(
                            (x, y), (x + (went + 0.02) * math.cos(h),
                                     y + (went + 0.02) * math.sin(h)),
                            self.owner) >= DRIVE_STOPS_M:
                        went += 0.02
                    self.at[:2] = [x + went * math.cos(h), y + went * math.sin(h)]
                    self.nearest = min(self.nearest, DRIVE_STOPS_M)
                    return {"reason": "blocked", "travelled_m": went, "turned_deg": 0.0,
                            "detail": "there is something in the way"}
                if self.owner_there:
                    self.nearest = min(self.nearest, _clearance((x, y), end, self.owner))
                self.at[:2] = list(end)
                return {"reason": "arrived", "travelled_m": step, "turned_deg": 0.0}
            to = goal_msg.pose.pose.position
            self.heading_to = (to.x, to.y)
            self.sent.append(("goto",))
            travelled = 0.0
            for second in range(0, 80):
                why = give_up(float(second), {"recoveries": 0}) if give_up else ""
                if why:
                    if self.stopped_after_s is None and self.stepped_in:
                        self.stopped_after_s = second - self.steps_in_s
                    # Stopping takes a moment at speed: on the rover that
                    # evening it went on 0.2 m after the stop was asked for.
                    slide = min(STOP_SLIDE_M * self.speed_ms / 0.41,
                                math.hypot(to.x - self.at[0], to.y - self.at[1]))
                    if self.owner_there and not self.on_the_line((to.x, to.y)):
                        heading = math.atan2(to.y - self.at[1], to.x - self.at[0])
                        self.at = [self.at[0] + slide * math.cos(heading),
                                   self.at[1] + slide * math.sin(heading), heading]
                        travelled += slide
                    return {"reason": "blocked", "travelled_m": travelled,
                            "turned_deg": 0.0, "detail": why}
                if second == self.steps_in_s and not self.stepped_in:
                    self.owner_there = self.stepped_in = True
                if self.on_the_line((to.x, to.y)):
                    self.swung_s += 1
                    continue                # back and forth on the spot
                left = math.hypot(to.x - self.at[0], to.y - self.at[1])
                hop = min(self.speed_ms, left)
                heading = math.atan2(to.y - self.at[1], to.x - self.at[0])
                self.at = [self.at[0] + hop * math.cos(heading),
                           self.at[1] + hop * math.sin(heading), heading]
                travelled += hop
                if left - hop < 0.05:
                    return {"reason": "arrived", "travelled_m": travelled,
                            "turned_deg": 0.0}
            return {"reason": "timeout", "travelled_m": travelled, "turned_deg": 0.0,
                    "detail": "it swung on the spot until it ran out of time"}

    def say(phase, why="", **fields):
        rover.said.append((phase, why))

    saved_sleep = nav_moves.time.sleep
    try:
        rover = Room()
        nav_moves.time.sleep = rover.sleep
        out = rover.goto(goal, bearing, say)
        check("stepped in front of a 1.8 m goal, it stops within two seconds "
              "rather than turning on the spot",
              (rover.stopped_after_s is not None and rover.stopped_after_s <= 2,
               rover.swung_s <= 2), (True, True))
        check("...waits for them",
              any(phase == "waiting" for phase, _ in rover.said), True)
        check("...goes round them, 0.25 m or more away, and arrives -- within the "
              "near goals' own 0.22 m of the goal",
              (out.get("reason"), rover.nearest >= 0.25,
               math.hypot(rover.at[0] - goal[0], rover.at[1] - goal[1])
               <= nav_moves.THERE_M + 0.01),
              ("arrived", True, True))

        rover = Room(steps_in_s=0)
        nav_moves.time.sleep = rover.sleep
        out = rover.goto(goal, bearing, say)
        check("standing there before it sets off: it goes round without "
              "ending up stopped beside them",
              (out.get("reason"), rover.swung_s <= 2), ("arrived", True))

        # The blocking trial of that evening, as the owner marked it: the
        # trials' 4.4 m leg up the charger room, the owner standing still
        # 0.25 m to the right of its line about 1.6 m up. **This room passes
        # where the rover did not**: on the rover the last leg of the way round
        # drifted 9 degrees towards the owner and Nav2's drive stopped it
        # (docs/progress/2026-10-08-blocking-trials.md). Straight legs here go
        # exactly where they are pointed, so this checks the plan, not its
        # driving.
        leg_start, leg_end = (-17.11, -15.86), (-17.05, -11.60)
        rover = Room(steps_in_s=0, owner_at=(-16.85, -14.24), begin=leg_start,
                     speed_ms=0.41)
        rover.at[2] = math.atan2(leg_end[1] - leg_start[1], leg_end[0] - leg_start[0])
        nav_moves.time.sleep = rover.sleep
        out = rover.goto(leg_end, None, say)
        check("the evening's trial: it goes round the owner and on up the leg",
              (out.get("reason"), rover.swung_s <= 2,
               math.hypot(rover.at[0] - leg_end[0], rover.at[1] - leg_end[1])
               <= nav_moves.THERE_M + 0.01), ("arrived", True, True))

        rover = Room(stays_s=2.0)
        nav_moves.time.sleep = rover.sleep
        out = rover.goto(goal, bearing, say)
        check("one who steps away again is waited for, then driven on to",
              (out.get("reason"),
               math.hypot(rover.at[0] - goal[0], rover.at[1] - goal[1])
               <= nav_moves.THERE_M + 0.01,
               rover.swung_s <= 2), ("arrived", True, True))

        rover = Room(owner_at=goal)
        nav_moves.time.sleep = rover.sleep
        out = rover.goto(goal, bearing, say)
        check("someone standing on the goal itself: given up as blocked, not "
              "turned at", (out.get("reason"), rover.swung_s <= 2), ("blocked", True))

        rover = Room(steps_in_s=99)
        nav_moves.time.sleep = rover.sleep
        out = rover.goto(goal, bearing, say)
        check("with nobody there it is one Nav2 goal, as before",
              (out.get("reason"), rover.sent), ("arrived", [("goto",)]))

        # A doorway's frame the route passes close by: on the live scan and on
        # the map alike. Nav2 meant to pass it; it is not stopped for.
        rover = Room(steps_in_s=0)
        frame = rover.owner
        rover.costmap = rover.mapped_walls = lambda: _grid(goal_fit, frame, [frame])
        check("something the map has as well is a wall, not something "
              "in the way", rover.seen_on_the_route(), "")
        rover.costmap = rover.mapped_walls = lambda: _grid(goal_fit, frame, [])
        check("...while something only the live scan has is",
              rover.seen_on_the_route().startswith(nav_moves.IN_THE_WAY), True)

        # **A person the planner's costmap has too, since its live layer**
        # (config/nav2.yaml, 2026-10-08). Where there is a way round, the
        # planner takes it and the route never reaches them. Where there is
        # none -- a doorway they stand in -- the planner finds no route, Nav2
        # keeps following the last one, through them, and runs its recoveries:
        # a quarter turn on the spot. So it is the map's walls that say what is
        # a wall, not the planner's costmap.
        rover = Room(steps_in_s=0)
        person = rover.owner
        rover.costmap = lambda: _grid(goal_fit, person, [person])
        rover.mapped_walls = lambda: _grid(goal_fit, person, [])
        check("a person on the planner's costmap as well as the live scan is "
              "still in the way, not a wall",
              rover.seen_on_the_route().startswith(nav_moves.IN_THE_WAY), True)
        out = rover.goto(goal, bearing, say)
        check("...so where the planner can find no way round them it stops "
              "within two seconds and waits, rather than turning on the spot",
              (rover.stopped_after_s is not None and rover.stopped_after_s <= 2,
               rover.swung_s <= 2, any(phase == "waiting" for phase, _ in rover.said)),
              (True, True, True))
    finally:
        nav_moves.time.sleep = saved_sleep


def test_the_live_layer_is_switched_off_and_on_at_run_time():
    """`{"op": "live_layer", "enabled": false}` is how the planner's live
    obstacle layer comes back out without a redeploy (config/nav2.yaml)."""
    section("the live layer's switch")
    sys.path.insert(0, HERE)
    from test_planning import _ros_messages
    _ros_messages()
    try:
        import types
        import nav_moves
    except ImportError as exc:                          # pragma: no cover
        print("  .... skipped, cannot import: %s" % exc)
        return

    class Done:
        def __init__(self, answer):
            self.answer = answer

        def result(self):
            return self.answer

    class Costmap:
        """The planner's costmap node: its parameters and nothing else."""

        def __init__(self):
            self.values = {"live_layer.enabled": True}
            self.set_calls = []

        def wait_for_service(self, timeout_sec=None):
            return True

    costmap = Costmap()

    class Setter:
        def wait_for_service(self, timeout_sec=None):
            return True

        def call_async(self, request):
            one = request.parameters[0]
            costmap.set_calls.append((one.name, one.value.bool_value))
            costmap.values[one.name] = one.value.bool_value
            return Done(types.SimpleNamespace(
                results=[types.SimpleNamespace(successful=True, reason="")]))

    class Getter:
        def wait_for_service(self, timeout_sec=None):
            return True

        def call_async(self, request):
            name = request.names[0]
            if name not in costmap.values:
                value = types.SimpleNamespace(type=0, bool_value=False)
            else:
                value = types.SimpleNamespace(type=1, bool_value=costmap.values[name])
            return Done(types.SimpleNamespace(values=[value]))

    class Node(nav_moves.NavMoves):
        def __init__(self):
            self.layer_set_client = Setter()
            self.footprint_client = Getter()

        def wait(self, future, timeout):
            return True

    node = Node()
    check("read without a value, it says the layer is on",
          node.live_layer(), {"ok": True, "enabled": True})
    check("...switched off, it says off and asked for exactly that",
          (node.live_layer(False), costmap.set_calls),
          ({"ok": True, "enabled": False}, [("live_layer.enabled", False)]))
    check("...and back on", node.live_layer(True), {"ok": True, "enabled": True})
    del costmap.values["live_layer.enabled"]
    check("a costmap with no live layer says so rather than guessing",
          node.live_layer().get("enabled"), None)


def test_a_person_with_no_way_round_is_waited_for_not_spun_at():
    """The doorway trial of 2026-10-08, 23:48, as Nav2 logged it.

    The owner stood in the middle of a corridor's 1.0 m mouth, the only way to
    a goal 1.9 m beyond it, 2 m from the rover. The planner's live layer had
    them on its costmap, so there was no route at all: every attempt failed
    after about 2.5 s and was followed by a costmap clear, and after the fourth
    Nav2's recoveries began -- a spin at 11 s (225 degrees on the chassis), a
    5 s wait, a reverse at 23 s, another spin at 30 s -- until the owner
    stepped aside. The route watch, which looks 1 m ahead, never came near them.
    """
    section("a person where there is no way round them")
    sys.path.insert(0, HERE)
    from test_planning import _ros_messages
    _ros_messages()
    try:
        import threading
        import types
        import nav_moves
    except ImportError as exc:                          # pragma: no cover
        print("  .... skipped, cannot import: %s" % exc)
        return
    start = (-17.38, -14.41)
    goal = (-13.5, -13.65)
    #: Nav2's ladder against a planner that cannot plan, from its logs: a
    #: recovery counted at each costmap clear and each behaviour, by second.
    LADDER = {2: 1, 5: 2, 8: 3, 11: 4, 14: 5, 23: 6, 27: 7, 30: 8}
    SPINS = (11, 30)
    BACKUPS = (23,)

    class Doorway(nav_moves.NavMoves):
        """The planner has no route while the owner stands in the mouth, and
        Nav2 climbs its ladder; once they have gone, the goal is driven."""

        def __init__(self, stays_s, planner_fails=True):
            self.at = [start[0], start[1], math.radians(100.0)]
            self.clock = 0.0
            self.stays_s = stays_s
            self.planner_fails = planner_fails
            self.spun, self.backed, self.said, self.asked = 0, 0, [], 0
            self.stop_seq = 3
            self._lock = threading.Lock()
            self.plan = None
            self.plan_at = None
            self.args = types.SimpleNamespace(map_frame="map")

        def owner_there(self):
            return self.clock < self.stays_s

        def pose(self):
            return tuple(self.at)

        def footprint(self):
            return BODY

        def costmap(self):
            return None

        def mapped_walls(self):
            return None

        def live_costmap(self):
            return None

        def route_points(self):
            return []

        def get_clock(self):
            stamp = types.SimpleNamespace(to_msg=lambda: None)
            return types.SimpleNamespace(now=lambda: stamp)

        def route_to(self, gx, gy, yaw):
            # The planner asked directly takes as long to fail as Nav2's own
            # attempts did.
            self.asked += 1
            if self.owner_there() and self.planner_fails:
                self.clock += 2.5
                return None, nav_moves.NO_VALID_PATH
            return (math.hypot(gx - self.at[0], gy - self.at[1]), 0.0), 0

        def sleep(self, seconds):
            self.clock += seconds

        def run_goal(self, kind, goal_msg, limit_s, say, measure, motion="driving",
                     budget=None, give_up=None, guard=None):
            to = goal_msg.pose.pose.position
            began = self.clock
            count = 0
            for second in range(0, 120):
                now = began + second
                self.clock = now
                if self.owner_there() and self.planner_fails:
                    count = LADDER.get(second, count)
                    self.spun += second in SPINS
                    self.backed += second in BACKUPS
                else:
                    # The controller stuck on something else for a moment:
                    # recoveries with a fresh route from the planner each time.
                    self.plan_at = now
                    if not self.planner_fails and second in (3, 6):
                        count += 1
                why = give_up(now, {"recoveries": count}) if give_up else ""
                if why:
                    return {"reason": "blocked", "travelled_m": 0.0,
                            "turned_deg": 0.0, "detail": why}
                if not self.owner_there() or not self.planner_fails:
                    if second >= 8:
                        self.at = [to.x, to.y, self.at[2]]
                        return {"reason": "arrived", "travelled_m": 4.0,
                                "turned_deg": 0.0}
            return {"reason": "timeout", "travelled_m": 0.0, "turned_deg": 0.0,
                    "detail": "ran out of time"}

    def said(rover):
        def say(phase, why="", **fields):
            rover.said.append((phase, why))
        return say

    saved = nav_moves.time.sleep, nav_moves.time.monotonic
    try:
        rover = Doorway(stays_s=30.0)
        nav_moves.time.sleep = rover.sleep
        nav_moves.time.monotonic = lambda: rover.clock
        out = rover.goto(goal, None, said(rover))
        check("blocked in a doorway, it is not turned on the spot or reversed "
              "while the owner stands there",
              (rover.spun, rover.backed), (0, 0))
        check("...it says it is waiting, and hands the goal back as blocked when "
              "they stay, a few seconds later",
              (any(phase == "waiting" for phase, _ in rover.said), out.get("reason"),
               str(out.get("detail") or "").startswith(nav_moves.NO_WAY),
               rover.clock < 15.0),
              (True, "blocked", True, True))

        rover = Doorway(stays_s=4.0)
        nav_moves.time.sleep = rover.sleep
        nav_moves.time.monotonic = lambda: rover.clock
        out = rover.goto(goal, None, said(rover))
        check("one who steps aside while it waits: it drives on and arrives, "
              "never having turned on the spot",
              (out.get("reason"), rover.spun, rover.backed), ("arrived", 0, 0))

        rover = Doorway(stays_s=0.0, planner_fails=False)
        nav_moves.time.sleep = rover.sleep
        nav_moves.time.monotonic = lambda: rover.clock
        out = rover.goto(goal, None, said(rover))
        check("Nav2 recovering from something else, with the planner answering, "
              "is left to it: the planner is not asked",
              (out.get("reason"), rover.asked), ("arrived", 0))
    finally:
        nav_moves.time.sleep, nav_moves.time.monotonic = saved


def test_a_spot_refused_for_something_the_scan_sees_says_so():
    """2026-10-09: the owner, driving by map clicks after the doorway trials, was
    told a spot was "inside a wall or under something". The spot is fitted to
    the body on the planner's costmap, which since its live layer has on it
    whatever the scan sees -- the owner walking beside the rover, the charger
    dock -- and the sentence blamed the map for it."""
    section("a spot refused for something the scan sees")
    sys.path.insert(0, HERE)
    from test_planning import _ros_messages
    _ros_messages()
    try:
        import threading
        import goal_fit
        import nav_moves
    except ImportError as exc:                          # pragma: no cover
        print("  .... skipped, cannot import: %s" % exc)
        return
    spot = (2.0, 1.0)

    def ring(points):
        # Something round the spot at 0.3 m every 20 degrees: no body fits
        # within half a metre of the middle.
        return [(spot[0] + 0.3 * math.cos(math.radians(a)),
                 spot[1] + 0.3 * math.sin(math.radians(a))) for a in range(0, 360, 20)] + points

    class Node(nav_moves.NavMoves):
        def __init__(self, costmap, walls):
            self._lock = threading.Lock()
            self.grid, self.walls = costmap, walls

        def footprint(self):
            return BODY

        def costmap(self):
            return self.grid

        def mapped_walls(self):
            return self.walls

    around = _grid(goal_fit, spot, ring([]))
    empty = _grid(goal_fit, spot, [])
    placed, why = Node(around, empty).fit_goal(spot[0], spot[1], 0.0)
    check("something only the scan has round the spot: refused, and said to be "
          "something the scan sees, not a wall",
          (placed, "scan" in why, "wall" in why), (None, True, False))
    placed, why = Node(around, around).fit_goal(spot[0], spot[1], 0.0)
    check("...the same on the map as well: a wall, as before",
          (placed, "inside a wall" in why), (None, True))
    placed, why = Node(around, None).fit_goal(spot[0], spot[1], 0.0)
    check("...and with no map to ask, as before",
          (placed, "inside a wall" in why), (None, True))


def test_the_map_walls_are_the_occupied_cells():
    """What the route watch asks "is that a wall" of: slam_toolbox's map, as
    the planner's static layer reads it, and nothing the live layer adds."""
    section("the map's walls")
    sys.path.insert(0, HERE)
    from test_planning import _ros_messages
    _ros_messages()
    try:
        import threading
        import types
        import goal_fit
        import nav_moves
    except ImportError as exc:                          # pragma: no cover
        print("  .... skipped, cannot import: %s" % exc)
        return

    class Node(nav_moves.NavMoves):
        def __init__(self, msg):
            self._lock = threading.Lock()
            self.map_msg = msg

    origin = types.SimpleNamespace(position=types.SimpleNamespace(x=-2.0, y=-1.0))
    info = types.SimpleNamespace(width=4, height=1, resolution=0.05, origin=origin)
    msg = types.SimpleNamespace(info=info, data=[100, 0, -1, 65])
    node = Node(msg)
    walls = node.mapped_walls()
    check("occupied is a wall; free, unknown and the merely likely are not",
          (list(walls.data), walls.width, walls.origin_x, walls.origin_y),
          ([goal_fit.LETHAL, 0, 0, 0], 4, -2.0, -1.0))
    check("...asked again of the same map, the same grid, not a new one",
          node.mapped_walls() is walls, True)
    node.map_msg = types.SimpleNamespace(info=info, data=[0, 0, 0, 100])
    check("...and a new map is read afresh",
          list(node.mapped_walls().data), [0, 0, 0, goal_fit.LETHAL])
    check("no map yet: nothing to say", Node(None).mapped_walls(), None)


TESTS = (
    test_a_way_round_a_person_is_found_on_the_live_scan,
    test_a_person_stepping_in_front_of_a_near_goal_is_waited_for,
    test_a_person_in_the_way_of_a_longer_goal_is_waited_for_and_gone_round,
    test_the_live_layer_is_switched_off_and_on_at_run_time,
    test_a_person_with_no_way_round_is_waited_for_not_spun_at,
    test_a_spot_refused_for_something_the_scan_sees_says_so,
    test_the_map_walls_are_the_occupied_cells,
)
