#!/usr/bin/env python3
"""Where the rover really is, by matching what the lidar sees now against the
map it already has.

Its own module with no ROS in it, for the reason `frontier.py` and `goal_fit.py`
are: the bridge runs this on the rover inside the conda environment, and
`selftest.py` runs it on a workstation with no ROS at all. A copy in the test
would be a copy that drifts, and this one would drift invisibly -- a matcher that
has quietly stopped agreeing with the one on the rover still returns a
plausible-looking pose.

## What this is for

The rover keeps its pose graph across a reboot now, so the map it wakes up with
is the map it had when it was switched off. What it cannot know is whether
anybody moved it in between. A rover pushed thirty centimetres and turned twenty
degrees while it was off comes back believing it is exactly where it was parked,
and every wall it then sees is thirty centimetres from where the map says it
should be.

So this takes the scan the lidar can see *now*, slides it over the map the rover
already has, and reports the position and heading that make the two agree. It is
one function and it has no memory: what to do with the answer is
`nav_bridge.refit`'s problem, and committing it to the graph is `slam_toolbox`'s.

## What it deliberately cannot do

**It cannot find a rover that has been carried.** The search is a window around
where the rover thinks it is -- a metre and forty-five degrees by default -- and
outside that window it reports nothing rather than guessing. That is the whole
safety property: a fit can be wrong by at most the size of the window, so the
worst this can do to a rover is the same order as the error it exists to remove.
Finding a rover that genuinely does not know where it is means searching the
whole house, and on a 2D lidar in a building where one corridor looks much like
another that is a coin toss dressed as an answer. See "Saved-map localization" in
docs/jetson-orin-navigation.md.

**It does not decide whether the answer is worth having on its own.** It reports
one, and it reports the two numbers that say whether to believe it: how well the
winning pose fits, and how well the best *rival* pose fits -- the best score
anywhere in the window that is not the same answer. A corridor that looks like
the corridor next door produces two peaks a few percent apart, and the honest
response to that is to refuse rather than to pick the taller one. `ok` is false
when either number says so, and `why` is the sentence to show a person.

## How the score works

The map is turned into a *field*: 1.0 on a cell the mapper calls a wall, falling
away with distance from one as `exp(-d^2 / 2 sigma^2)`. A candidate pose scores
the mean of that field under every point of the scan, so a scan lying exactly on
the walls scores 1.0 and a scan lying half a metre off them scores near zero.
The smear is what makes the search find anything at all: against bare occupied
cells the score is zero everywhere except at the exact answer, and a search over
a 10 cm grid steps straight over it.

**A scan point over ground nobody has mapped is not counted at all**, in either
half of that average. The map has no opinion about those cells, so scoring them
as misses makes the answer depend on how much of the house has been explored
rather than on where the rover is: measured on the rover with a map five square
metres across, a scan matched against the very map it had just drawn scored 0.72,
because most of its 8 m reach lay outside the map entirely. Excluding them is
safe here only because the search is a window -- over a metre, which candidate
pose is chosen barely changes which points land on mapped ground, so there is
nothing for a pose to gain by drifting into the unknown. A pose with almost
nothing left to score is refused rather than believed; see `MIN_POINTS`.

The search is two passes. A coarse one over the whole window at 10 cm and 3
degrees, which is what finds the answer and what the rival is measured from, and
a fine one over a few centimetres around the winner at 2 cm and half a degree,
which is what makes the answer worth committing. One pass at the fine step over
the whole window would be four hundred times the work for the same answer.

    python3 refit.py fixtures/kitchen-loop.pgm.gz --at 6.3,8.7 --off 0.3,-0.2,15
"""

import math

import numpy as np

#: What counts as a wall. `frontier.py`'s, so that two files in this directory
#: cannot come to different views of one grid; that one has the note about where
#: the number comes from.
from frontier import OCCUPIED_AT, read_pgm

#: How far the rover may have been moved, and how far it may have been turned,
#: before this stops looking. Not a tuning knob so much as the promise the whole
#: module rests on: a fit cannot move the rover further than this, so a wrong one
#: is an error of this size rather than a rover that believes it is in another
#: room. A metre is more than anybody nudges a parked rover by and less than the
#: distance to the next doorway.
WINDOW_M = 1.0
WINDOW_DEG = 45.0

#: The coarse pass. Ten centimetres is half the smear below, which is the
#: coarsest step that cannot step over the peak; three degrees is eight
#: centimetres of arc at the far wall of a room and about a centimetre at the
#: near one.
COARSE_STEP_M = 0.10
COARSE_STEP_DEG = 3.0

#: The fine pass, over one coarse step either way. Two centimetres is under half
#: the 5 cm the map is drawn at, so the limit on the answer is the map rather
#: than the search.
FINE_SPAN_M = COARSE_STEP_M
FINE_SPAN_DEG = COARSE_STEP_DEG
FINE_STEP_M = 0.02
FINE_STEP_DEG = 0.5

#: How far from a wall a scan point still counts as being on it. Ten centimetres
#: is two cells of the map and about the width of the walls slam_toolbox draws,
#: and it is also what `correlation_search_space_smear_deviation` in
#: config/slam_toolbox.yaml smears the mapper's own matcher by -- the same idea,
#: for the same reason, one search finer.
SMEAR_M = 0.10

#: What it takes to believe an answer: the winner must put this share of the scan
#: on a wall, and it must beat the best *rival* -- the best pose in the window
#: more than DISTINCT_M away or DISTINCT_DEG round from it -- by this margin.
#:
#: **These three were measured rather than chosen, and the measurement is the
#: reason they are as strict as they are.** Sixty places on the kitchen-loop map,
#: a scan cast from each, and the rover told it was somewhere it was not:
#:
#:      what happened to the rover        accepted   and wrong
#:      nudged 35 cm and 15 deg             60/60         0
#:      nudged, a quarter of the room moved 56/60         0
#:      nudged, half the room moved          9/60         0
#:      carried 2 m                          3/60         3
#:      carried 3 m                          0/60         0
#:      turned 90 deg                        0/60         0
#:
#: The bottom three rows are outside the window and cannot be answered correctly
#: at all, so every one of them accepted is a lie -- a rover moved confidently to
#: somewhere it is not. Loosening the score to 0.50 takes those three lies to
#: nine and buys back the half-changed room; the trade was taken the other way,
#: because a refusal costs a person one drive with the mapper doing its ordinary
#: job and a lie costs them a rover that believes a wall is a doorway.
#:
#: Read the middle row as the honest limit of the feature rather than as a
#: failure: half of every scan disagreeing with the map is not a rover that has
#: moved, it is a room that has.
MIN_SCORE = 0.60
MIN_MARGIN = 1.25
DISTINCT_M = 0.25
DISTINCT_DEG = 15.0

#: How small a correction is no correction. Five centimetres is one cell of the
#: map, and half a degree of the map's own resolution at the far wall; committing
#: one means reloading the pose graph and jumping the rover on the console for
#: something the mapper's own scan matcher absorbs by itself on the next scan.
#: So a fit this small is reported as agreement rather than applied.
SETTLED_M = 0.05
SETTLED_DEG = 2.0

#: Fewer scan points than this and there is nothing to match, counted twice: the
#: returns the sensor gave at all, and then the ones that landed on ground the map
#: has an opinion about. A revolution this thin is a blocked sensor rather than an
#: empty room -- lidar_node.py refuses to publish one -- and the second count is
#: the case that actually happens, which is a rover looking into a room nobody has
#: mapped yet. Sixty of 360 bins is a sixth of the horizon.
MIN_POINTS = 60


class Fit(object):
    """One answer, and everything needed to decide whether to believe it.

    `ok` is the recommendation and `why` is the sentence behind it, in both
    directions: a refusal says what was wrong and an acceptance says what it
    found, because both end up in front of a person who is looking at a map and
    wondering what just happened to their rover.
    """

    def __init__(self, x_m, y_m, heading_deg, score, rival, guess_score,
                 moved_m, turned_deg, points, ok, why, settled=False,
                 scored=0):
        self.x_m = x_m
        self.y_m = y_m
        self.heading_deg = heading_deg
        self.score = score
        self.rival = rival
        self.guess_score = guess_score
        self.moved_m = moved_m
        self.turned_deg = turned_deg
        self.points = points
        #: How many of those the map had an opinion about, at the winning pose.
        #: The rest were over unmapped ground and were not scored.
        self.scored = scored
        self.ok = ok
        self.why = why
        #: The fit agrees with where the rover already thinks it is, so there is
        #: nothing to commit. `ok` is true and the caller should do nothing.
        self.settled = settled

    def as_dict(self):
        return {"ok": self.ok, "why": self.why, "settled": self.settled,
                "x_m": round(self.x_m, 3), "y_m": round(self.y_m, 3),
                "heading_deg": round(self.heading_deg, 1),
                "moved_m": round(self.moved_m, 3),
                "turned_deg": round(self.turned_deg, 1),
                "score": round(self.score, 3),
                "rival": round(self.rival, 3),
                "guess_score": round(self.guess_score, 3),
                "points": self.points, "scored": self.scored}

    def __repr__(self):
        return ("Fit(%.3f, %.3f, %.1f deg, score %.3f, rival %.3f, %s)"
                % (self.x_m, self.y_m, self.heading_deg, self.score,
                   self.rival, "ok" if self.ok else self.why))


def points_of(ranges, angle_min, angle_increment,
              range_min=0.0, range_max=float("inf")):
    """A LaserScan's ranges as points in the rover's own frame.

    Here rather than in the bridge because it is arithmetic and arithmetic is
    what the selftest can argue with. The conventions are ROS's and this rover's
    together: x is forward, y is left, and `base_link -> laser` is the identity
    on this rover -- the lidar *is* where the rover is, which base_node.py
    explains and which is why nothing rotates or offsets the points here.

    Anything infinite, absent or outside the sensor's own limits is dropped
    rather than clamped. A dropped return is "no echo", which on this sensor is
    about one return in six and is what a black sofa or a glass door looks like;
    laying those on the map at 8 m would be matching the scan against furniture
    that is not there.
    """
    kept = []
    for i, r in enumerate(ranges):
        if r is None:
            continue
        r = float(r)
        if not (range_min <= r <= range_max) or not math.isfinite(r):
            continue
        angle = angle_min + i * angle_increment
        kept.append((r * math.cos(angle), r * math.sin(angle)))
    return np.asarray(kept, dtype=np.float64).reshape(-1, 2)


def field(grid, smear_m=SMEAR_M):
    """The map as something a scan can be scored against, and where it has a view.

    Two `(height, width)` arrays laid out the way the occupancy grid is. The
    first is 1.0 on a wall, falling off as a gaussian of distance from the
    nearest one. The second is 1.0 on every cell the mapper has an opinion about
    -- wall or floor -- and 0.0 on the ones it has never seen, which is what
    decides whether a scan point is scored at all.

    The field is built by taking the maximum of the wall mask shifted every way
    within a few cells and weighted by how far it was shifted, which is a
    max-filter with a gaussian kernel rather than a blur: two walls a few
    centimetres apart must not add up to a stronger wall than either, because a
    scan point can only be on one of them.

    The known mask is smeared the same way and for the same reason. Without it a
    point landing one cell beyond the wall it belongs to -- which is most of what
    a fit is correcting -- would fall in unmapped space and be dropped from the
    average exactly when it is the evidence.
    """
    cells = np.asarray(grid.data, dtype=np.int16).reshape(grid.height, grid.width)
    wall = (cells >= OCCUPIED_AT).astype(np.float32)
    seen = (cells >= 0).astype(np.float32)
    out = wall.copy()
    known = seen.copy()
    # Two sigma. Past that the weight is under 0.14 and the extra shifts cost
    # more than they change; inside it, this is what lets the coarse pass at 10
    # cm find a peak it would otherwise step straight over.
    radius = max(1, int(math.ceil(2.0 * smear_m / grid.resolution)))
    for drow in range(-radius, radius + 1):
        for dcol in range(-radius, radius + 1):
            if drow == 0 and dcol == 0:
                continue
            distance = math.hypot(drow, dcol) * grid.resolution
            weight = math.exp(-(distance ** 2) / (2.0 * smear_m ** 2))
            if weight < 0.05:
                continue
            rows = slice(max(0, -drow), grid.height - max(0, drow))
            cols = slice(max(0, -dcol), grid.width - max(0, dcol))
            into_rows = slice(max(0, drow), grid.height - max(0, -drow))
            into_cols = slice(max(0, dcol), grid.width - max(0, -dcol))
            dst = out[into_rows, into_cols]
            np.maximum(dst, wall[rows, cols] * weight, out=dst)
            dst = known[into_rows, into_cols]
            np.maximum(dst, seen[rows, cols], out=dst)
    return out, known


def _scores(walls, known, grid, points, guess, offsets_x, offsets_y, headings):
    """How much of the scan lies on a wall, for every pose in a grid of them.

    Returns two arrays shaped `(len(headings), len(offsets_y), len(offsets_x))`:
    the share of the scan lying on a wall, and how many points that share was
    taken over -- the ones landing on ground the map has an opinion about. A pose
    whose scan lands entirely on unmapped floor comes back as a zero score over a
    count of none, rather than as a division by it.

    The loop is over headings and everything else is one array operation, because
    rotating the scan is the only part that has to be redone per heading -- the
    translations are then an outer sum, which is what makes a fourteen-thousand
    pose search a tenth of a second rather than a minute.
    """
    width, height = grid.width, grid.height
    inv = 1.0 / grid.resolution
    shape = (len(headings), len(offsets_y), len(offsets_x))
    out = np.empty(shape, np.float32)
    counts = np.empty(shape, np.int32)
    shifts = np.stack(np.meshgrid(offsets_x, offsets_y), axis=-1).reshape(-1, 2)
    for h, heading in enumerate(headings):
        angle = math.radians(heading)
        cos, sin = math.cos(angle), math.sin(angle)
        turned_x = points[:, 0] * cos - points[:, 1] * sin + guess[0]
        turned_y = points[:, 0] * sin + points[:, 1] * cos + guess[1]
        # (poses, points) in cells. floor rather than round: a cell holds
        # [origin + i*res, origin + (i+1)*res), which is `Grid.cell_of`.
        col = np.floor((turned_x[None, :] + shifts[:, 0:1] - grid.origin_x)
                       * inv).astype(np.int32)
        row = np.floor((turned_y[None, :] + shifts[:, 1:2] - grid.origin_y)
                       * inv).astype(np.int32)
        inside = (col >= 0) & (col < width) & (row >= 0) & (row < height)
        # Clipped so the gather is always in bounds; the masks are what actually
        # decide. Off the edge of the grid is the same as over a cell nobody has
        # mapped -- past the map is somewhere nobody has been -- so both are left
        # out of the average rather than counted as misses.
        np.clip(col, 0, width - 1, out=col)
        np.clip(row, 0, height - 1, out=row)
        at = row * width + col
        counted = inside & (known[at] > 0.0)
        found = np.where(counted, walls[at], 0.0).sum(axis=1)
        seen = counted.sum(axis=1)
        out[h] = (found / np.maximum(seen, 1)).reshape(shape[1:])
        counts[h] = seen.reshape(shape[1:])
    return out, counts


def _peak(scores, offsets_x, offsets_y, headings):
    """The best pose in a score block, as `(score, dx, dy, dheading)`."""
    h, r, c = np.unravel_index(int(np.argmax(scores)), scores.shape)
    return (float(scores[h, r, c]), float(offsets_x[c]), float(offsets_y[r]),
            float(headings[h]))


def fit(grid, points, guess, window_m=WINDOW_M, window_deg=WINDOW_DEG,
        min_score=MIN_SCORE, smear_m=SMEAR_M):
    """The pose in the window that best explains this scan, and whether to trust it.

    `guess` is where the rover currently believes it is, as `(x_m, y_m,
    heading_deg)` in the map frame, and the answer is in the same frame. Every
    distance is metres and every angle is degrees, because everything that reads
    this -- the bridge, the daemon, the console -- is in degrees, and one
    conversion at the sensor is cheaper than four in the callers.
    """
    points = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    if len(points) < MIN_POINTS:
        return Fit(guess[0], guess[1], guess[2], 0.0, 0.0, 0.0, 0.0, 0.0,
                   len(points), False,
                   "only %d usable returns in the scan, which is not enough to "
                   "match anything" % (len(points),))

    walls, known = field(grid, smear_m)
    walls, known = walls.reshape(-1), known.reshape(-1)
    steps = int(round(window_m / COARSE_STEP_M))
    offsets = np.arange(-steps, steps + 1) * COARSE_STEP_M
    turns = int(round(window_deg / COARSE_STEP_DEG))
    headings = guess[2] + np.arange(-turns, turns + 1) * COARSE_STEP_DEG
    coarse, seen = _scores(walls, known, grid, points, guess, offsets, offsets,
                           headings)
    # A pose the map can barely see is not a candidate at all. Without this the
    # search could slide the scan off the edge of what has been mapped, where the
    # handful of points still landing on a wall would average to a fine score
    # over almost nothing.
    coarse = np.where(seen >= MIN_POINTS, coarse, 0.0)

    score, dx, dy, heading = _peak(coarse, offsets, offsets, headings)

    # The best answer that is *not* this one, measured before the fine pass
    # because that is the pass that would hide it: a rival half a metre away is
    # exactly as sharp as the winner and only the coarse block can see both.
    away = ((((offsets - dx) ** 2)[None, None, :]
             + ((offsets - dy) ** 2)[None, :, None] > DISTINCT_M ** 2)
            | (np.abs(headings - heading)[:, None, None] > DISTINCT_DEG))
    rival = float(coarse[away].max()) if away.any() else 0.0

    # And how the pose the rover already believes scores, which is reported
    # rather than decided on: it is the number that says how badly the rover was
    # placed, and a person reading "62% on a wall against 21% where it stood"
    # can see the difference between a nudge and a rover that had lost the room.
    guess_score = float(_scores(walls, known, grid, points, guess,
                                np.zeros(1), np.zeros(1),
                                np.array([guess[2]]))[0][0, 0, 0])

    fine_offsets = np.arange(-FINE_SPAN_M, FINE_SPAN_M + 1e-9, FINE_STEP_M)
    fine_turns = np.arange(-FINE_SPAN_DEG, FINE_SPAN_DEG + 1e-9, FINE_STEP_DEG)
    fine, fine_seen = _scores(walls, known, grid, points,
                              (guess[0] + dx, guess[1] + dy),
                              fine_offsets, fine_offsets, heading + fine_turns)
    fine = np.where(fine_seen >= MIN_POINTS, fine, 0.0)
    score, ddx, ddy, heading = _peak(fine, fine_offsets, fine_offsets,
                                     heading + fine_turns)
    scored = int(fine_seen.reshape(-1)[int(np.argmax(fine))])
    x, y = guess[0] + dx + ddx, guess[1] + dy + ddy
    moved = math.hypot(x - guess[0], y - guess[1])
    turned = (heading - guess[2] + 180.0) % 360.0 - 180.0

    if scored < MIN_POINTS:
        return Fit(x, y, heading, score, rival, guess_score, moved, turned,
                   len(points), False,
                   "only %d of the %d returns in this scan land on ground that "
                   "has been mapped, which is not enough to place the rover by: "
                   "it is looking into a part of the room nobody has driven"
                   % (scored, len(points)), scored=scored)
    if score < min_score:
        return Fit(x, y, heading, score, rival, guess_score, moved, turned,
                   len(points), False,
                   "the scan does not fit the map anywhere near here -- the best "
                   "of %d poses put %.0f%% of it on a wall, and a fit needs %.0f%%"
                   % (coarse.size, 100.0 * score, 100.0 * min_score), scored=scored)
    if rival > 0.0 and score < rival * MIN_MARGIN:
        return Fit(x, y, heading, score, rival, guess_score, moved, turned,
                   len(points), False,
                   "the scan fits the map in two different places about equally "
                   "well (%.0f%% here against %.0f%% elsewhere), so which one the "
                   "rover is standing in is not something this can tell"
                   % (100.0 * score, 100.0 * rival), scored=scored)
    if moved < SETTLED_M and abs(turned) < SETTLED_DEG:
        return Fit(x, y, heading, score, rival, guess_score, moved, turned,
                   len(points), True,
                   "the rover is where it thinks it is, to within %.0f cm and "
                   "%.1f degrees, so nothing was moved"
                   % (100.0 * moved, abs(turned)), settled=True, scored=scored)
    return Fit(x, y, heading, score, rival, guess_score, moved, turned,
               len(points), True,
               "the scan fits the map %.0f cm and %.1f degrees from where the "
               "rover thought it was, with %.0f%% of it on a wall against %.0f%% "
               "where it stood" % (100.0 * moved, turned, 100.0 * score,
                                   100.0 * guess_score), scored=scored)


def cast(grid, pose, bins=360, range_max=8.0):
    """The scan a lidar would see standing at `pose` in this map.

    For the selftest and for `main` below, and it is the honest half of a
    simulation rather than the whole of one: it walks each ray until it meets a
    wall the map already holds, so it can prove the search finds a pose the map
    itself explains, and it cannot prove anything about furniture that moved or a
    door that is now shut. Those are what `MIN_SCORE` is set against, and the
    tests get at them by throwing part of the scan away rather than by pretending
    this draws a real room.
    """
    x, y, heading = pose
    step = grid.resolution * 0.5
    out = []
    for i in range(bins):
        angle = math.radians(heading) + 2.0 * math.pi * i / bins - math.pi
        cos, sin = math.cos(angle), math.sin(angle)
        distance = grid.resolution
        while distance <= range_max:
            col, row = grid.cell_of(x + cos * distance, y + sin * distance)
            if not grid.inside(col, row):
                distance = float("inf")
                break
            if grid.at(col, row) >= OCCUPIED_AT:
                break
            distance += step
        out.append(distance if distance <= range_max else float("inf"))
    return out


def main(argv=None):
    import argparse

    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("map", help="a PGM written by map_score.py, .gz allowed")
    ap.add_argument("--at", required=True, metavar="X,Y",
                    help="where the rover really is, in metres")
    ap.add_argument("--heading", type=float, default=0.0)
    ap.add_argument("--off", default="0,0,0", metavar="DX,DY,DDEG",
                    help="how far out the rover thinks it is")
    ap.add_argument("--resolution", type=float, default=0.05)
    args = ap.parse_args(argv)

    grid = read_pgm(args.map, resolution=args.resolution)
    truth = tuple(float(v) for v in args.at.split(",")) + (args.heading,)
    off = tuple(float(v) for v in args.off.split(","))
    guess = (truth[0] + off[0], truth[1] + off[1], truth[2] + off[2])
    scan = cast(grid, truth)
    answer = fit(grid, points_of(scan, -math.pi, 2.0 * math.pi / len(scan)), guess)
    print("truth   %.2f %.2f %.1f" % truth)
    print("guess   %.2f %.2f %.1f" % guess)
    print("fitted  %.2f %.2f %.1f" % (answer.x_m, answer.y_m, answer.heading_deg))
    print(answer.why)
    return 0 if answer.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
