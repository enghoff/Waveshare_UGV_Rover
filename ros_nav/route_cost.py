#!/usr/bin/env python3
"""What a route costs to drive: how far it runs, and how much of it is turning.

Its own module, with no ROS in it, because three things need this arithmetic and
only one of them can import `rclpy`: the bridge, which budgets a move's time
allowance on it; `dwb_bench.py`, which reports whether a goal would have timed
out before anybody drives it; and `selftest.py`, which runs on a workstation with
no rover attached. The same reason `goal_fit.py` and `drive_mixer.py` are
modules, and the same risk if it were copied -- a copy of a *table* drifts
visibly, a copy of a measurement drifts invisibly.

**The fault this exists to stop.** `drive_to` used to work its time allowance out
from the distance to the goal in a straight line, and NavFn does not fly. Sent to
a spot 2.95 m away with a wall in between, it returned a correct 8.81 m route --
out west, round the wall and back -- which needs about 44 seconds of driving and
pivoting if nothing at all goes wrong. The bridge allowed 53. That is not too
short to drive the route, and saying so would be the easy half-truth: it is 15%
of headroom on a stack that replans once a second and spends fifteen seconds on
each rung of its recovery ladder, so the first thing to go wrong spends all of it.
Three things went wrong, the bridge cancelled, and the console said "timed out" --
which reads as a rover that could not find its way. It had found its way and was
driving it.

**Turning is half the bill on this chassis, so it is counted.** A skid-steer
rover changes direction by stopping and pivoting, and a grid route round a wall
has a corner at every turn of the building. The route above snakes through six of
them and asks for 598 degrees of heading change all told, which at the rate the
controller pivots is 22 seconds against the 22 seconds its 8.81 m of driving
costs. Budget on the length alone and you have halved the answer.

**Sampled, not summed pose by pose.** NavFn plans on a 5 cm grid, so a path's
heading is quantised to eight compass points and a gentle curve is stored as a
staircase. Summing the heading change between consecutive poses charges 45 degrees
for every step of it. Measured on the 346-pose, 8.81 m route above:

    pose by pose            3259 deg      the grid, not the route
    sampled every 0.10 m     998 deg
    sampled every 0.25 m     598 deg      what this module uses
    sampled every 0.50 m     470 deg
    sampled every 1.00 m     422 deg

**The figure is sensitive to the sample length and does not fully converge**, which
is worth saying rather than hiding: this route really does snake -- west, north,
east, north, east, then south to the goal -- so several hundred degrees of it are
real, and the rest is how finely you look. 0.25 m is four cells, which is short
enough to see a doorway as a corner and long enough that a straight run reads as
straight. Over-counting is the safe direction here and the only reason the choice
is not agonised over: this feeds a *backstop*, and a backstop that is too generous
lets a wedged rover grind for longer, while one that is too tight cancels a rover
that was driving perfectly well. The second is the failure this exists to stop.
"""

import math

#: How far apart to take the samples the turning is measured between. Four cells
#: at this map's resolution -- far enough that a staircase reads as the line it
#: approximates, short enough that a real corner is still a corner.
SAMPLE_M = 0.25


def wrap(radians):
    return math.atan2(math.sin(radians), math.cos(radians))


def length_and_turning(points, sample_m=SAMPLE_M):
    """Metres along a route, and degrees of heading change it asks for.

    `points` is a sequence of (x, y). Returns (metres, degrees), both 0.0 for
    anything too short to have a direction -- which is the honest answer for a
    one-pose plan and keeps a caller from budgeting on noise.
    """
    points = [(float(x), float(y)) for x, y in points]
    if len(points) < 2:
        return 0.0, 0.0

    metres = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                 for a, b in zip(points, points[1:]))

    # Resample the route at a fixed spacing before measuring any angle, so the
    # grid's own staircase cannot be mistaken for the route's corners.
    marks = [points[0]]
    run = 0.0
    for a, b in zip(points, points[1:]):
        step = math.hypot(b[0] - a[0], b[1] - a[1])
        run += step
        if run >= sample_m:
            marks.append(b)
            run = 0.0
    if marks[-1] != points[-1]:
        marks.append(points[-1])
    if len(marks) < 3:
        return metres, 0.0

    headings = [math.atan2(b[1] - a[1], b[0] - a[0])
                for a, b in zip(marks, marks[1:])
                if math.hypot(b[0] - a[0], b[1] - a[1]) > 1e-9]
    turning = sum(abs(wrap(b - a)) for a, b in zip(headings, headings[1:]))
    return metres, math.degrees(turning)


def seconds_for(metres, degrees, speed_ms, turn_dps, slack=1.0, floor=0.0):
    """How long to allow a route, out of its two parts.

    Kept here beside the measurement rather than at the call site so that the
    bench and the bridge cannot come to different answers about the same route,
    which is the whole point of this being a module.
    """
    if metres <= 0.0:
        return floor
    driving = metres / max(0.05, speed_ms)
    turning = degrees / max(1.0, turn_dps)
    return max(floor, slack * (driving + turning))


def from_path(path, sample_m=SAMPLE_M):
    """The same, straight off a `nav_msgs/Path`. The only ROS-shaped thing here,
    and it touches no ROS types beyond reading two floats off each pose."""
    if path is None or not getattr(path, "poses", None):
        return 0.0, 0.0
    return length_and_turning(
        [(q.pose.position.x, q.pose.position.y) for q in path.poses], sample_m)
