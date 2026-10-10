#!/usr/bin/env python3
"""Checks on `change_watch`: a synthetic room, scanned the way the D500 would.

The room is walls and boxes as line segments, and a scan is each beam's nearest
crossing with them. That is enough to put the watch through what it was built
for -- an armchair moved between two visits -- and the ways it could go wrong:
the same room twice with the pose a little off, a person walking through, and a
change made while the rover is still in the room.
"""
import math

import numpy as np

import test_harness  # noqa: F401  (paths)
from test_harness import check, section

import change_watch as cw

BEAMS = np.radians(np.arange(0.0, 360.0, 0.72))
ROOM = [((0, 0), (6, 0)), ((6, 0), (6, 4)), ((6, 4), (0, 4)), ((0, 4), (0, 0))]


def box(cx, cy, size=0.8):
    h = size / 2.0
    corners = [(cx - h, cy - h), (cx + h, cy - h), (cx + h, cy + h), (cx - h, cy + h)]
    return [(corners[i], corners[(i + 1) % 4]) for i in range(4)]


def scan(x, y, heading, segments):
    """Range along each beam to the nearest segment, as the lidar would see it."""
    a = BEAMS + heading
    dx, dy = np.cos(a), np.sin(a)
    best = np.full(len(a), np.inf)
    for (x1, y1), (x2, y2) in segments:
        ex, ey = x2 - x1, y2 - y1
        den = dx * ey - dy * ex
        with np.errstate(divide="ignore", invalid="ignore"):
            t = ((x1 - x) * ey - (y1 - y) * ex) / den
            u = ((x1 - x) * dy - (y1 - y) * dx) / den
        hit = (np.abs(den) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1)
        best = np.where(hit & (t < best), t, best)
    return best


def visit(watch, segments, t0, seconds=60.0, noise=None, extra=None):
    """A rover driving slowly round the room for `seconds`, 3 scans a second.

    `noise` is a random generator for a pose a couple of centimetres and half a
    degree out, as slam_toolbox's is; `extra` is (from_s, to_s, segments) for
    something present for part of the visit only."""
    path = [(1.0, 1.0), (5.0, 1.0), (5.0, 3.0), (1.0, 3.0)]
    n = int(seconds * 3)
    for i in range(n):
        f = (i / n) * len(path)
        (ax, ay), (bx, by) = path[int(f) % 4], path[(int(f) + 1) % 4]
        k = f - int(f)
        x, y = ax + k * (bx - ax), ay + k * (by - ay)
        heading = 0.3 * i
        t = t0 + i / 3.0
        seen = list(segments)
        if extra and extra[0] <= i / 3.0 < extra[1]:
            seen += extra[2]
        r = scan(x, y, heading, seen)
        if noise is not None:
            x += noise.normal(0, 0.02)
            y += noise.normal(0, 0.02)
            heading += math.radians(noise.normal(0, 0.5))
        watch.add_scan(x, y, heading, r, BEAMS, t)


def new_watch():
    return cw.ChangeWatch.around(0.0, 0.0, 6.0, 4.0, margin_m=0.5)


def kinds(found, large=True):
    return sorted((c["kind"], round(c["x_m"], 1), round(c["y_m"], 1))
                  for c in found if c["large"] or not large)


def test_an_armchair_moved_between_visits_is_found_at_both_ends() -> None:
    """**The case it exists for, and the one it was measured on.** On 2026-10-10
    the grey armchair by the charger was moved 1.3 m and put back two hours
    later; run against run, the lidar found it gone from one place and standing
    in the other, at its record's position."""
    section("an armchair moved between two visits")
    watch = new_watch()
    visit(watch, ROOM + box(2.0, 2.0), t0=0.0)
    check("a first visit has nothing to compare with", watch.changes(), [])
    visit(watch, ROOM + box(4.0, 2.0), t0=3600.0)
    found = watch.changes(large_only=True)
    check("the second finds it gone from where it was and new where it is",
          sorted(c["kind"] for c in found), ["GONE", "NEW"])
    gone = next(c for c in found if c["kind"] == "GONE")
    new = next(c for c in found if c["kind"] == "NEW")
    check("...where it was, to a tenth of a metre",
          math.hypot(gone["x_m"] - 2.0, gone["y_m"] - 2.0) < 0.3, True)
    check("...and where it is now",
          math.hypot(new["x_m"] - 4.0, new["y_m"] - 2.0) < 0.3, True)
    check("...each the size of furniture",
          (gone["large"], new["large"]), (True, True))
    check("...saying when the place was last seen the other way",
          gone["was_otherwise_at"] < 100.0, True)


def test_the_same_room_twice_is_no_change() -> None:
    """With the pose two centimetres and half a degree out, as the mapper's is,
    the walls and the box must not come out as a change of their own."""
    section("the same room twice, the pose a little out")
    rng = np.random.default_rng(7)
    watch = new_watch()
    visit(watch, ROOM + box(2.0, 2.0), t0=0.0, noise=rng)
    visit(watch, ROOM + box(2.0, 2.0), t0=3600.0, noise=rng)
    check("nothing of any size is reported", kinds(watch.changes(), large=False), [])


def test_a_person_walking_through_is_not_a_change() -> None:
    """Five seconds of a person-sized something on the second visit."""
    section("a person walking through")
    watch = new_watch()
    visit(watch, ROOM, t0=0.0)
    visit(watch, ROOM, t0=3600.0, extra=(20.0, 25.0, box(3.0, 2.0, size=0.4)))
    check("nothing is reported", kinds(watch.changes(), large=False), [])


def test_a_room_is_not_compared_with_itself() -> None:
    """A move made while the rover is still in the room ends no visit, so it is
    not reported until the place has been left and come back to. Reported at
    once, the visit's first half would be the rover's own reference, and a
    person standing still for a while would be a change every time."""
    section("a change made during one visit")
    watch = new_watch()
    visit(watch, ROOM + box(2.0, 2.0), t0=0.0, seconds=60.0)
    visit(watch, ROOM + box(4.0, 2.0), t0=60.0, seconds=60.0)
    check("nothing is reported within the visit", kinds(watch.changes()), [])
    visit(watch, ROOM + box(4.0, 2.0), t0=3600.0)
    check("the next visit, compared with one that saw both, claims nothing: "
          "neither place was clearly one thing then",
          kinds(watch.changes(), large=False), [])


def test_what_it_remembers_survives_a_restart() -> None:
    """The reference is the point of it, so it is written to disk and read back."""
    import os
    import tempfile
    section("the tallies across a restart")
    watch = new_watch()
    visit(watch, ROOM + box(2.0, 2.0), t0=0.0)
    path = os.path.join(tempfile.mkdtemp(), "watch.npz")
    watch.save(path, map_id="abc")
    back, meta = cw.ChangeWatch.load(path)
    check("the map it belongs to comes back with it", meta["map_id"], "abc")
    visit(back, ROOM + box(4.0, 2.0), t0=3600.0)
    check("and a move after the restart is still found",
          sorted(c["kind"] for c in back.changes(large_only=True)), ["GONE", "NEW"])
    check("a file that is not there is no watch", cw.ChangeWatch.load(path + "x"),
          (None, None))


def test_a_map_that_grows_keeps_what_was_learned() -> None:
    """slam_toolbox's map grows as the rover finds more of the flat, and a
    watch sized for the first room must not forget it when it is enlarged."""
    section("a watch enlarged for a bigger map")
    watch = cw.ChangeWatch.around(0.0, 0.0, 6.0, 4.0, margin_m=0.5)
    visit(watch, ROOM + box(2.0, 2.0), t0=0.0)
    bigger = watch.grown_to(-5.0, -3.0, 6.0, 4.0, margin_m=0.5)
    check("it covers the new box", bigger.covers(-5.0, -3.0, 6.0, 4.0), True)
    check("...on the same grid", ((watch.x0 - bigger.x0) / watch.res) % 1.0 < 1e-6, True)
    visit(bigger, ROOM + box(4.0, 2.0), t0=3600.0)
    check("and still finds a move against what the smaller one saw",
          sorted(c["kind"] for c in bigger.changes(large_only=True)), ["GONE", "NEW"])


def test_a_change_is_news_once() -> None:
    """The node logs each change the first time it is reported, so that a run
    can be counted afterwards; the same armchair every thirty seconds would make
    one change look like thirty."""
    section("a change logged once")
    a = {"kind": "GONE", "x_m": 1.0, "y_m": 1.0}
    check("a change nobody has reported is news",
          cw.ChangeWatch.first_seen([a], []), [a])
    moved = dict(a, x_m=1.2)
    check("the same group a little shifted is not",
          cw.ChangeWatch.first_seen([moved], [a]), [])
    other = dict(a, kind="NEW")
    check("a group of the other kind at the same place is",
          cw.ChangeWatch.first_seen([other], [a]), [other])
    far = dict(a, x_m=2.0)
    check("and so is one of the same kind further off",
          cw.ChangeWatch.first_seen([far], [a]), [far])


def visit_misplaced(watch, segments, t0, dx, dy, dturn_deg, seconds=60.0):
    """A visit scanned truly but laid on the map at a pose that is off -- the
    rover moved by hand, or a restore that put it the wrong way round, while
    navigation went on calling the map settled."""
    path = [(1.0, 1.0), (5.0, 1.0), (5.0, 3.0), (1.0, 3.0)]
    n = int(seconds * 3)
    refused = 0
    for i in range(n):
        f = (i / n) * len(path)
        (ax, ay), (bx, by) = path[int(f) % 4], path[(int(f) + 1) % 4]
        k = f - int(f)
        x, y = ax + k * (bx - ax), ay + k * (by - ay)
        heading = 0.3 * i
        r = scan(x, y, heading, segments)
        took = watch.add_scan(x + dx, y + dy, heading + math.radians(dturn_deg), r,
                              BEAMS, t0 + i / 3.0)
        refused += took is False
    return refused, n


def test_a_scan_laid_at_the_wrong_pose_is_refused() -> None:
    """**2026-10-10, 18:06: two furniture-sized changes that were the rover's
    pose.** The owner had driven the rover and rebooted it; the restore put it
    170 degrees from where it stood and the map stayed settled, so the watch
    counted every scan at the wrong place and logged the room as changed. A
    scan whose hits fall on floor the watch has already seen clear is not a
    room that has changed: an armchair is a small part of a scan, and a wrong
    pose is most of it."""
    section("a visit laid at the wrong pose")
    for dx, dy, turn in ((0.5, 0.3, 20.0), (0.0, 0.0, 170.0)):
        watch = new_watch()
        visit(watch, ROOM + box(2.0, 2.0), t0=0.0)
        refused, n = visit_misplaced(watch, ROOM + box(2.0, 2.0), 3600.0, dx, dy, turn)
        check("%.1f m and %.0f degrees out: most scans are refused" % (math.hypot(dx, dy), turn),
              refused > 0.8 * n, True)
        check("...and no furniture-sized change is reported",
              kinds(watch.changes()), [])
    watch = new_watch()
    visit(watch, ROOM + box(2.0, 2.0), t0=0.0)
    refused, n = visit_misplaced(watch, ROOM + box(4.0, 2.0), 3600.0, 0.0, 0.0, 0.0)
    check("an armchair truly moved, at the true pose, is not refused", refused, 0)
    check("...and is still found", sorted(c["kind"] for c in watch.changes(large_only=True)),
          ["GONE", "NEW"])


TESTS = (
    test_a_scan_laid_at_the_wrong_pose_is_refused,
    test_a_change_is_news_once,
    test_a_map_that_grows_keeps_what_was_learned,
    test_an_armchair_moved_between_visits_is_found_at_both_ends,
    test_the_same_room_twice_is_no_change,
    test_a_person_walking_through_is_not_a_change,
    test_a_room_is_not_compared_with_itself,
    test_what_it_remembers_survives_a_restart,
)

if __name__ == "__main__":
    for test in TESTS:
        test()
    print("\n%d passed, %d failed" % (test_harness.PASSED, test_harness.FAILED))
