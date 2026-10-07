"""Something in the way of a near goal: seen on the live scan, waited for, gone round.

The planner's costmap is the map and nothing else, so a person standing in
front of the rover is only on the controller's: the live scan's local
costmap. These checks put a person there and nowhere else, which is what the
rover had in M3 session 13 on 2026-10-07.
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


TESTS = (
    test_a_way_round_a_person_is_found_on_the_live_scan,
    test_a_person_stepping_in_front_of_a_near_goal_is_waited_for,
)
