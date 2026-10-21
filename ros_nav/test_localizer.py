"""The localizer's tracker (localizer.py), on the recorded kitchen-loop map.

Scans are cast from the map with `refit.cast`, so these show that the tracker
follows a rover the map explains, takes out what odometry gets wrong a step at a
time, and refuses what it should; what a real lidar among moved furniture does
was measured on recorded drives (docs/plans/continuous-localization.md).
"""

import math

import numpy as np

import localizer
import refit
from test_refit import SPOTS, _map
from test_harness import check, section


def _points(grid, pose, move=0):
    """The scan at `pose` as rover-frame points; `move` shortens one ray in N to
    60%, which is a person or a chair standing where the map has floor."""
    rays = refit.cast(grid, pose)
    out = []
    for i, r in enumerate(rays):
        if not math.isfinite(r) or r < 0.15:
            continue
        if move and i % move == 0:
            r = max(0.25, r * 0.6)
        a = 2.0 * math.pi * i / len(rays) - math.pi
        out.append((r * math.cos(a), r * math.sin(a)))
    return np.asarray(out)


def _off(a, b):
    return (math.hypot(a[0] - b[0], a[1] - b[1]),
            abs((a[2] - b[2] + 180.0) % 360.0 - 180.0))


def _tracker(grid, pose):
    tracker = localizer.Tracker()
    tracker.set_map(grid)
    tracker.set_pose(pose, (0.0, 0.0, 0.0), 0.0)
    return tracker


def test_the_tracker_takes_out_what_odometry_gets_wrong() -> None:
    section("the tracker against odometry that is wrong")
    grid = _map()
    start = (SPOTS[1][0], SPOTS[1][1], 0.0)

    # Driving 25 cm forward while odometry turns half a degree a step that the
    # rover did not: 5 degrees over the drive, which is what one fast turn left.
    tracker = _tracker(grid, start)
    worst = 0.0
    for k in range(1, 11):
        truth = (start[0] + 0.025 * k, start[1], 0.0)
        odom = (0.025 * k, 0.0, 0.5 * k)
        step = tracker.update(_points(grid, truth), odom, 0.2 * k)
        worst = max(worst, _off(step.pose, truth)[0])
    check("a heading that odometry turns 5 degrees out is held to the truth",
          (worst < 0.04, _off(step.pose, truth)[1] < 1.5), (True, True))

    # Stuck against something while the wheels spin: odometry counts 2 cm a step
    # for ten steps and the rover does not move.
    tracker = _tracker(grid, start)
    for k in range(1, 11):
        step = tracker.update(_points(grid, start), (0.02 * k, 0.0, 0.0), 0.2 * k)
    check("a wheel slip odometry counted as 20 cm of travel is taken out",
          _off(step.pose, start)[0] < 0.04, True)


def test_standing_still_it_does_not_wander() -> None:
    section("the tracker on a parked rover")
    grid = _map()
    start = (SPOTS[1][0], SPOTS[1][1], 0.0)
    tracker = _tracker(grid, start)
    for k in range(1, 6):
        step = tracker.update(_points(grid, start, move=4), (0.0, 0.0, 0.0), 0.2 * k)
    check("a quarter of the scan blocked by something near does not move it",
          _off(step.pose, start)[0] < 0.01, True)

    # Nudged 12 cm while odometry, parked, saw nothing: taken only once the
    # same correction has been found PARKED_VOTES scans running.
    nudged = (start[0] + 0.12, start[1], 0.0)
    moved = []
    for k in range(1, 5):
        step = tracker.update(_points(grid, nudged), (0.0, 0.0, 0.0), 2.0 + 0.2 * k)
        moved.append(_off(step.pose, start)[0] > 0.05)
    check("a nudge is not taken on the first scans that show it",
          moved[:localizer.PARKED_VOTES - 1], [False] * (localizer.PARKED_VOTES - 1))
    check("...and is taken once it has been seen %d times" % localizer.PARKED_VOTES,
          moved[localizer.PARKED_VOTES - 1], True)


def test_one_step_corrects_only_so_far() -> None:
    section("the tracker's step is capped")
    grid = _map()
    truth = (SPOTS[1][0], SPOTS[1][1], 0.0)
    tracker = _tracker(grid, (truth[0] - 0.20, truth[1], 0.0))
    steps = []
    for k in range(1, 5):
        here = (truth[0] + 0.01 * k, truth[1], 0.0)
        step = tracker.update(_points(grid, here), (0.01 * k, 0.0, 0.0), 0.2 * k)
        steps.append(step)
    first = steps[0]
    check("a correction of 20 cm is taken %.0f cm at a time" % (100 * localizer.MAX_STEP_M),
          abs(first.pose[0] - (truth[0] - 0.20 + 0.01)) <= localizer.MAX_STEP_M + 0.015, True)
    check("...and finished in a few steps",
          _off(steps[-1].pose, (truth[0] + 0.04, truth[1], 0.0))[0] < 0.04, True)


def test_what_it_cannot_explain_it_leaves_to_odometry() -> None:
    section("the tracker refuses what it cannot match")
    grid = _map()
    here = (SPOTS[1][0], SPOTS[1][1], 0.0)
    elsewhere = (SPOTS[2][0], SPOTS[2][1], 0.0)
    tracker = _tracker(grid, here)
    step = tracker.update(_points(grid, elsewhere), (0.05, 0.0, 0.0), 0.2)
    check("a scan from another room is not matched", step.accepted, False)
    check("...and the answer is odometry's",
          _off(step.pose, (here[0] + 0.05, here[1], 0.0))[0] < 1e-6, True)

    tracker = _tracker(grid, here)
    step = tracker.update(_points(grid, here), (0.0, 0.0, 30.0), 0.2)
    check("a scan swept turning at 150 deg/s is not matched", step.accepted, False)
    check("...and says why", "too fast" in step.why, True)

    tracker = _tracker(grid, here)
    tracker.set_pose(elsewhere, (0.0, 0.0, 0.0), 0.0)
    step = tracker.update(_points(grid, elsewhere), (0.0, 0.0, 0.0), 0.2)
    check("a pose handed over is where it tracks from", _off(step.pose, elsewhere)[0] < 0.01, True)


TESTS = (
    test_the_tracker_takes_out_what_odometry_gets_wrong,
    test_standing_still_it_does_not_wander,
    test_one_step_corrects_only_so_far,
    test_what_it_cannot_explain_it_leaves_to_odometry,
)
