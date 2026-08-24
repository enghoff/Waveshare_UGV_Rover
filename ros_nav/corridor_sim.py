#!/usr/bin/env python3
"""Why DWB refuses all thirty-three of its candidates in a narrow passage.

    python3 corridor_sim.py --plan             # the one that reproduces it
    python3 corridor_sim.py                    # poses, at 1.0 m
    python3 corridor_sim.py --widths 0.9,1.0,1.2,1.5
    python3 corridor_sim.py --run              # the abort-and-clear cycle

**The line this exists to explain.** The rover's own log, a hundred and
seventy-five times, and once for twelve unbroken seconds ending in a client
cancel:

    [controller_server]: Could not find a legal trajectory: No valid
    trajectories out of 33!

Thirty-three is not a round number, it is *this* rover: `vx_samples: 2` and
`vtheta_samples: 16` from config/nav2.yaml, iterated the way DWB iterates
them. When all thirty-three are thrown out the controller returns no command;
`failure_tolerance: 0.3` allows three tenths of a second of that before
`follow_path` aborts; the tree then clears a local costmap that was telling
the truth, replans, and hands back the same situation with the controller's
memory wiped.

**What reproduces it: the plan, not the passage.** `--plan` is the mode that
matters. `PathDist` floods the local costmap outward from the plan and that
flood is stopped by 253, the inscribed ring the inflation layer paints 14 cm
deep along every wall. When *every* cell of the plan is already inside that
ring there is nowhere to flood from, every cell of the costmap comes back
"unreachable", and the critic refuses all thirty-three -- wherever the rover
is standing and whichever way it is facing. It is total, it is independent of
the rover's pose, and it recurs on every replan. A plan within about 6 cm of
a wall does it, which a 0.8 m passage reaches at 0.32 m off the centreline and
a 1.2 m passage does not reach at all.

**What does not reproduce it, having been tried.** A narrow passage on its
own: once the obstacle layer's footprint clearing is modelled -- and it is on,
`footprint_clearing_enabled` was read off the running node -- a rover in a
clean 0.9 m corridor loses all thirty-three at two poses in three hundred and
twelve. Nor does a reverse sample (`min_vel_x` below zero rescues six of
fifty-six), nor a shorter rollout (`sim_time` from 0.8 s down to 0.3 s changes
nothing), nor moving `PathAlign.forward_point_distance` from 0.1 m to 0.32 m,
which is what the rolled-back attempt did.

**The link still to be closed on the rover.** NavFn plans on the *global*
costmap, in the `map` frame, and would not route through its own inflated
ring. `PathDist` tests that plan against the *local* costmap, in the `odom`
frame. The two only agree while `map -> odom` does, and in a metre-wide
passage a few centimetres of disagreement is the whole margin. Confirming
that means watching the transformed plan against the live local costmap
during an episode, or turning `debug_trajectory_details` on and reading which
critic names itself -- `dwb_bench.py` already starts a shadow controller that
does the second without moving the wheels.

**A veto is not a score.** Four of the seven critics refuse candidates by
throwing, and tuning their *scales* does not touch that -- worth saying
plainly, because `ObstacleFootprint.scale: 0.005` reads like a decision to
care very little about obstacles and is nothing of the kind. The four, as the
rover's own `libdwb_critics.so` has them:

  * `ObstacleFootprint` walks the footprint outline at every pose of the
    rollout and throws on 254 or 255. Its `pointCost` compares the cell
    against 0xfe and 0xff and has no third comparison, so the 253 ring is a
    cost here and not contact.
  * `PathDist` and `GoalDist` are the flood described above, and they test the
    rover's centre point rather than its body.
  * `PathAlign` and `GoalAlign` run the same test at a point
    `forward_point_distance` in front of the pose.
  * `Oscillation` latches the sign of the turn once the rover is pivoting and
    throws every candidate that turns back until it has moved 5 cm or turned
    0.2 rad -- and what clears that latch is a new `follow_path` goal, which
    is what the abort above produces.

**What is not modelled.** The scores themselves rank survivors and cannot
rescue a tick with none. `--run` does pick a winner by the config's weights,
so the cycle it prints is the shape of the real one rather than its exact
heading history.
"""

from __future__ import annotations

import argparse
import collections
import heapq
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import goal_fit

# --- config/nav2.yaml, controller_server -------------------------------------
# Copied rather than parsed, the way dwb_bench.py and steering_sim.py copy them.
# Named after the keys so a change there that is not made here is findable.
MIN_VEL_X = 0.0
MAX_VEL_X = 0.40
MAX_VEL_THETA = 0.78
VX_SAMPLES = 2
VTHETA_SAMPLES = 16
SIM_TIME = 0.8
LINEAR_GRANULARITY = 0.05
ANGULAR_GRANULARITY = 0.025
ACC_LIM_X = 4.0
ACC_LIM_THETA = 8.0
CONTROLLER_FREQUENCY = 10.0
FAILURE_TOLERANCE_S = 0.3
XY_GOAL_TOLERANCE = 0.22

PATH_ALIGN_SCALE = 32.0
PATH_DIST_SCALE = 32.0
GOAL_ALIGN_SCALE = 24.0
GOAL_DIST_SCALE = 24.0
OBSTACLE_SCALE = 0.005
FORWARD_POINT_DISTANCE = 0.1

# dwb_critics defaults, which the config does not override.
OSCILLATION_RESET_DIST = 0.05
OSCILLATION_RESET_ANGLE = 0.2
X_ONLY_THRESHOLD = 0.05

#: local_costmap: a 3 m rolling window at 5 cm, obstacle + inflation.
RESOLUTION = 0.05
INFLATION_RADIUS = 0.45
COST_SCALING_FACTOR = 3.0

#: The footprint, and the inscribed radius nav2 derives from it -- the distance
#: from the body's origin to the nearest edge, which is what gets inflated to
#: 253 and therefore what stops the PathDist flood.
FOOTPRINT = [(0.20, 0.14), (0.20, -0.14), (-0.16, -0.14), (-0.16, 0.14)]
INSCRIBED_M = 0.14

#: lidar_slam/nav_types.py, applied by drive_mixer: a standing turn slower than
#: this does not clear stiction, so it is lifted to this. It is why a two-degree
#: correction leaves as a twelve-degree one.
MIN_TURN_DPS = 12.0

#: What the rover's log prints when it gives up, so a sample set that stops
#: matching it is noticed rather than quietly simulated.
CANDIDATES = 33

#: behavior_server, read off the running node. `simulate_ahead_time` is the
#: number that decides whether a wedged rover turns at all: Spin and BackUp
#: project their whole motion this far forward and return FAILED if any of it
#: touches a lethal cell, rather than moving as far as they safely can.
CYCLE_FREQUENCY = 10.0
SIMULATE_AHEAD_S = 2.0
MAX_ROTATIONAL_VEL = 0.5
MIN_ROTATIONAL_VEL = 0.2
ROTATIONAL_ACC_LIM = 1.5

#: What the recovery subtree of the behaviour tree asks those two for.
SPIN_DIST = 1.57
BACKUP_DIST = 0.30
BACKUP_SPEED = 0.15

OBSTACLE_SCORE = -1.0
UNREACHABLE_SCORE = -2.0


# --- DWB's sample set ---------------------------------------------------------
def one_d_velocities(current, low, high, acc_limit, acc_time, samples):
    """`nav_2d_utils::OneDVelocityIterator`, including the zero it inserts.

    Two things here are easy to get wrong and both change the count. The band
    is clipped to what one control period of acceleration can reach rather
    than to the configured limits; and when the band straddles zero, zero is
    *added* to the samples rather than being one of them -- sixteen evenly
    spaced samples between -0.78 and +0.78 never land on it.
    """
    current = min(max(current, low), high)
    top = min(high, current + acc_limit * acc_time)
    bottom = max(low, current - acc_limit * acc_time)
    if samples == 1:
        return [bottom]
    step = (top - bottom) / (samples - 1)
    out = [bottom + i * step for i in range(samples)]
    if bottom < 0.0 < top and not any(abs(v) < 1e-9 for v in out):
        out.append(0.0)
    return out


def twists(vx_now=0.0, wz_now=0.0):
    """Every candidate DWB scores this tick.

    The pair (0, 0) is dropped by `nav_2d_utils::isValidSpeed`, which is the
    difference between thirty-four and the thirty-three the log names. Standing
    perfectly still is not a candidate: whatever DWB picks, the rover either
    turns or drives.
    """
    period = 1.0 / CONTROLLER_FREQUENCY
    xs = one_d_velocities(vx_now, MIN_VEL_X, MAX_VEL_X, ACC_LIM_X, period,
                          VX_SAMPLES)
    thetas = one_d_velocities(wz_now, -MAX_VEL_THETA, MAX_VEL_THETA,
                              ACC_LIM_THETA, period, VTHETA_SAMPLES)
    return [(vx, wz) for vx in xs for wz in thetas
            if abs(vx) > 1e-9 or abs(wz) > 1e-9]


def rollout(x, y, yaw, vx, wz):
    """`StandardTrajectoryGenerator`: constant velocity, granularity steps.

    Not `discretize_by_time`, which is off by default, so how many poses a
    candidate has comes from how far it travels and how far it turns rather
    than from the clock. A 0.40 m/s sample is seven poses and a full-rate
    pivot is twenty-five, and `ObstacleFootprint` checks every one of them.
    """
    steps = max(1, int(math.ceil(max(
        abs(vx) * SIM_TIME / LINEAR_GRANULARITY,
        abs(wz) * SIM_TIME / ANGULAR_GRANULARITY))))
    dt = SIM_TIME / steps
    poses = []
    px, py, pyaw = x, y, yaw
    for _ in range(steps):
        px += vx * math.cos(pyaw) * dt
        py += vx * math.sin(pyaw) * dt
        pyaw = wrap(pyaw + wz * dt)
        poses.append((px, py, pyaw))
    return poses


def wrap(radians):
    return math.atan2(math.sin(radians), math.cos(radians))


# --- the costmap --------------------------------------------------------------
def _lower_envelope(values):
    """One dimension of Felzenszwalb's exact distance transform.

    An approximate transform is not good enough here. The threshold that
    matters is 253, which nav2 paints at exactly the inscribed radius, and a
    cell put one centimetre the wrong side of it is the difference between a
    candidate DWB refuses and one it scores. A nearest-source flood gets some
    of those cells wrong; this does not.
    """
    n = len(values)
    hull = [0] * n
    edge = [0.0] * (n + 1)
    k = 0
    hull[0] = 0
    edge[0] = -1e20
    edge[1] = 1e20
    for q in range(1, n):
        while True:
            p = hull[k]
            crossing = ((values[q] + q * q) - (values[p] + p * p)) / (2.0 * q - 2.0 * p)
            if crossing <= edge[k]:
                k -= 1
                if k < 0:
                    k = 0
                    break
            else:
                break
        k += 1
        hull[k] = q
        edge[k] = crossing
        edge[k + 1] = 1e20
    out = [0.0] * n
    k = 0
    for q in range(n):
        while edge[k + 1] < q:
            k += 1
        p = hull[k]
        out[q] = (q - p) * (q - p) + values[p]
    return out


def inflate(width, height, lethal):
    """`nav2_costmap_2d::InflationLayer`, over a set of lethal cells.

    Everything within the inscribed radius becomes 253 and everything out to
    the inflation radius decays from 252. In a metre-wide passage the whole
    floor is inside the second and a strip 14 cm deep along each wall is
    inside the first -- a preference and a barrier respectively, and the
    difference between the two is most of this file.

    The distance every cell is inflated by is exact, and cheap enough to
    rebuild for every pose -- which this has to do, because the obstacle layer
    clears whatever the footprint is standing on before the inflation layer
    ever runs.
    """
    far = float(width * width + height * height)
    field = [far] * (width * height)
    for col, row in lethal:
        field[row * width + col] = 0.0
    for col in range(width):
        column = _lower_envelope([field[row * width + col] for row in range(height)])
        for row in range(height):
            field[row * width + col] = column[row]
    data = [0] * (width * height)
    reach_cells = INFLATION_RADIUS / RESOLUTION
    inscribed_cells = INSCRIBED_M / RESOLUTION
    for row in range(height):
        base = row * width
        line = _lower_envelope(field[base:base + width])
        for col in range(width):
            away = math.sqrt(line[col])
            if away == 0.0:
                data[base + col] = goal_fit.LETHAL
            elif away <= inscribed_cells:
                data[base + col] = goal_fit.INSCRIBED
            elif away <= reach_cells:
                data[base + col] = int((goal_fit.INSCRIBED - 1) * math.exp(
                    -COST_SCALING_FACTOR * (away * RESOLUTION - INSCRIBED_M)))
    return data


def corridor(width_m, length_m=4.0, doorway=False, clear_at=None):
    """A passage of that width running east, walls both sides.

    The rover's y is measured from the centreline, so an offset of zero is a
    body exactly between the walls. `clear_at` is a pose whose footprint the
    obstacle layer has set to free space, which the live costmap does
    (`footprint_clearing_enabled` is true on this rover, checked) and which
    matters more than it sounds: without it a body that has clipped a wall
    stays clipped for ever, and a simulation that leaves it out invents a
    deadlock the real rover does not have.
    """
    margin = 0.9
    height_m = width_m + 2.0 * margin
    width = int(round(length_m / RESOLUTION))
    height = int(round(height_m / RESOLUTION))
    origin_x, origin_y = -1.2, -height_m / 2.0
    blank = goal_fit.CostGrid(width, height, RESOLUTION, origin_x, origin_y,
                              [0] * (width * height))
    spared = set() if clear_at is None else goal_fit.covered(
        blank, FOOTPRINT, clear_at[0], clear_at[1], clear_at[2])
    lethal = []
    for col in range(width):
        x = origin_x + (col + 0.5) * RESOLUTION
        for row in range(height):
            y = origin_y + (row + 0.5) * RESOLUTION
            outside = abs(y) > width_m / 2.0
            if doorway:
                outside = 0.95 <= x <= 1.05 and abs(y) > width_m / 2.0
            if outside and (col, row) not in spared:
                lethal.append((col, row))
    return goal_fit.CostGrid(width, height, RESOLUTION, origin_x, origin_y,
                             inflate(width, height, lethal))


def cleared(grid):
    """The costmap the behaviour tree's recovery leaves behind: empty.

    `ClearEntireCostmap` does not clear "what was wrong"; it clears
    everything, and the obstacle layer then refills from the next scan. For
    one update period every candidate is legal, which is why the rover twitches
    rather than standing still.
    """
    return goal_fit.CostGrid(grid.width, grid.height, grid.resolution,
                             grid.origin_x, grid.origin_y,
                             [0] * (grid.width * grid.height))


# --- the critics that can throw -----------------------------------------------
def line_cells(col0, row0, col1, row1):
    """Bresenham, as `nav2_util::LineIterator` walks a footprint edge."""
    cells = []
    dc, dr = abs(col1 - col0), abs(row1 - row0)
    step_c = 1 if col1 > col0 else -1
    step_r = 1 if row1 > row0 else -1
    err = dc - dr
    col, row = col0, row0
    while True:
        cells.append((col, row))
        if col == col1 and row == row1:
            return cells
        err2 = 2 * err
        if err2 > -dr:
            err -= dr
            col += step_c
        if err2 < dc:
            err += dc
            row += step_r


def obstacle_footprint(grid, poses):
    """`ObstacleFootprintCritic`: the outline, every pose, 254 and 255 only.

    Verified against the rover's own `libdwb_critics.so` rather than assumed:
    `pointCost` compares the cell against 0xfe and 0xff and has no third
    comparison, so the 253 ring the inflation layer paints along every wall is
    a cost here and not a refusal. The refusal comes from the map-grid critics
    below, which is a much easier thing to miss.
    """
    worst = 0.0
    for x, y, yaw in poses:
        col, row = grid.cell_of(x, y)
        if not (0 <= col < grid.width and 0 <= row < grid.height):
            return None, "ObstacleFootprint: trajectory goes off grid"
        corners = goal_fit.corners(FOOTPRINT, x, y, yaw)
        for i, (ax, ay) in enumerate(corners):
            bx, by = corners[(i + 1) % len(corners)]
            c0, r0 = grid.cell_of(ax, ay)
            c1, r1 = grid.cell_of(bx, by)
            for col, row in line_cells(c0, r0, c1, r1):
                if not (0 <= col < grid.width and 0 <= row < grid.height):
                    return None, "ObstacleFootprint: footprint goes off grid"
                cost = grid.cost(col, row)
                if cost == goal_fit.LETHAL:
                    return None, "ObstacleFootprint: trajectory hits obstacle"
                if cost == goal_fit.UNKNOWN:
                    return None, "ObstacleFootprint: trajectory hits unknown"
                worst = max(worst, float(cost))
    return worst, ""


def flood(grid, seeds):
    """`MapGridCritic`'s propagation: distance in cells, stopped by 253.

    Seeded from the path or from the goal and flooded four-connected, exactly
    as `CostmapQueue` does it. A cell it refuses to enter is marked obstacle
    and a cell it never reaches stays unreachable, and the critic throws on
    both -- so a strip 14 cm deep along every wall is a hard refusal for the
    rover's *centre point*, and anything the flood cannot get to behind it is
    a refusal too.
    """
    values = [UNREACHABLE_SCORE] * (grid.width * grid.height)
    queue = collections.deque()
    for col, row in seeds:
        if not (0 <= col < grid.width and 0 <= row < grid.height):
            continue
        index = row * grid.width + col
        if goal_fit.blocked(grid.cost(col, row)) \
                or grid.cost(col, row) == goal_fit.UNKNOWN:
            values[index] = OBSTACLE_SCORE
            continue
        if values[index] == UNREACHABLE_SCORE:
            values[index] = 0.0
            queue.append((col, row, 0.0))
    while queue:
        col, row, distance = queue.popleft()
        for dcol, drow in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nc, nr = col + dcol, row + drow
            if not (0 <= nc < grid.width and 0 <= nr < grid.height):
                continue
            index = nr * grid.width + nc
            if values[index] != UNREACHABLE_SCORE:
                continue
            cost = grid.cost(nc, nr)
            if goal_fit.blocked(cost) or cost == goal_fit.UNKNOWN:
                values[index] = OBSTACLE_SCORE
                continue
            values[index] = distance + 1.0
            queue.append((nc, nr, distance + 1.0))
    return values


def map_grid_score(grid, values, x, y, name):
    """`MapGridCritic::scorePose`, which is a point test, not a body test."""
    col, row = grid.cell_of(x, y)
    if not (0 <= col < grid.width and 0 <= row < grid.height):
        return None, "%s: trajectory goes off grid" % name
    value = values[row * grid.width + col]
    if value == OBSTACLE_SCORE:
        return None, "%s: trajectory hits obstacle" % name
    if value == UNREACHABLE_SCORE:
        return None, "%s: trajectory hits unreachable area" % name
    return value, ""


def forward_pose(x, y, yaw, distance=FORWARD_POINT_DISTANCE):
    """Where `PathAlign` and `GoalAlign` actually look: ahead of the nose."""
    return x + distance * math.cos(yaw), y + distance * math.sin(yaw)


class Oscillation(object):
    """`OscillationCritic`, which is the memory the abort keeps throwing away.

    Once the rover is turning below `x_only_threshold` of forward speed, the
    critic watches only the turn, and after one command it will not consider a
    turn the other way until the rover has moved 5 cm or turned 0.2 rad from
    where it started standing. In a passage where the wall refuses one
    direction that is half the sample set gone for a reason unrelated to the
    wall.
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self.positive_only = False
        self.negative_only = False
        self.stationary = None

    def veto(self, vx, wz):
        if self.positive_only and wz < 0.0:
            return "Oscillation: trajectory is oscillating"
        if self.negative_only and wz > 0.0:
            return "Oscillation: trajectory is oscillating"
        return ""

    def debrief(self, x, y, yaw, vx, wz):
        """After a command is chosen: latch, or clear because the rover moved."""
        if self.stationary is None:
            self.stationary = (x, y, yaw)
        sx, sy, syaw = self.stationary
        if math.hypot(x - sx, y - sy) >= OSCILLATION_RESET_DIST \
                or abs(wrap(yaw - syaw)) >= OSCILLATION_RESET_ANGLE:
            self.reset()
            self.stationary = (x, y, yaw)
        if abs(vx) > X_ONLY_THRESHOLD:
            return
        if wz > 0.0:
            self.positive_only = True
        elif wz < 0.0:
            self.negative_only = True


def evaluate(grid, path, goal, x, y, yaw, vx_now=0.0, wz_now=0.0,
             oscillation=None, path_values=None, goal_values=None):
    """Score every candidate, or say which critic threw it out.

    Returns the survivors as (score, vx, wz) and a tally of refusals by
    critic. `aggregation_type` is "last" for the four map-grid critics, which
    is their default and matters: a pivot's last pose is where the rover
    already stands, so a rover whose centre is inside the 253 ring has every
    one of its sixteen pivots refused before the wall is even consulted.
    """
    if path_values is None:
        path_values = flood(grid, [grid.cell_of(px, py) for px, py in path])
    if goal_values is None:
        goal_values = flood(grid, [grid.cell_of(*goal)])
    kept = []
    refused = collections.Counter()
    for vx, wz in twists(vx_now, wz_now):
        if oscillation is not None:
            reason = oscillation.veto(vx, wz)
            if reason:
                refused[reason] += 1
                continue
        poses = rollout(x, y, yaw, vx, wz)
        obstacle, reason = obstacle_footprint(grid, poses)
        if reason:
            refused[reason] += 1
            continue
        end_x, end_y, end_yaw = poses[-1]
        total = OBSTACLE_SCALE * obstacle
        failed = ""
        for name, values, point, scale in (
                ("PathDist", path_values, (end_x, end_y), PATH_DIST_SCALE),
                ("GoalDist", goal_values, (end_x, end_y), GOAL_DIST_SCALE),
                ("PathAlign", path_values,
                 forward_pose(end_x, end_y, end_yaw), PATH_ALIGN_SCALE),
                ("GoalAlign", goal_values,
                 forward_pose(end_x, end_y, end_yaw), GOAL_ALIGN_SCALE)):
            value, reason = map_grid_score(grid, values, point[0], point[1],
                                           name)
            if reason:
                failed = reason
                break
            total += scale * value
        if failed:
            refused[failed] += 1
            continue
        kept.append((total, vx, wz))
    kept.sort()
    return kept, refused


# --- the sweep ------------------------------------------------------------------
def straight_path(grid, x, y, ahead=2.0):
    """A route down the middle of the passage, which is what NavFn produces."""
    steps = int(ahead / RESOLUTION)
    return [(x + i * RESOLUTION, 0.0) for i in range(steps + 1)]


def sweep(width_m, headings, doorway=False):
    """How many candidates survive at each pose, footprint clearing and all.

    The costmap is rebuilt for every pose because the obstacle layer clears
    whatever the body is standing on before the inflation layer runs, so the
    map a rover sees genuinely depends on where the rover is. Leaving that out
    is what makes a clean corridor look like a trap.
    """
    limit = width_m / 2.0 - 0.14
    offsets = [round(-0.35 + 0.05 * i, 2) for i in range(15)]
    offsets = [o for o in offsets if abs(o) < limit]
    rows = []
    for offset in offsets:
        counts, blames = [], []
        for heading in headings:
            yaw = math.radians(heading)
            grid = corridor(width_m, doorway=doorway,
                            clear_at=(0.0, offset, yaw))
            path = straight_path(grid, 0.0, 0.0)
            kept, refused = evaluate(grid, path, (path[-1][0], 0.0),
                                     0.0, offset, yaw)
            counts.append(len(kept))
            blames.append(refused)
        rows.append((offset, counts, blames))
    return rows


def render(width_m, headings, rows):
    print("a %.2f m passage, thirty-three candidates a tick, no oscillation "
          "latch yet" % width_m)
    print("      offset  " + "".join("%5d" % h for h in headings)
          + "   deg off the passage")
    for offset, counts, _ in rows:
        marks = "".join(("    ." if c == 0 else "%5d" % c) for c in counts)
        print("      %+5.2f m %s" % (offset, marks))
    print("      '.' is a tick with nothing to drive -- the log's "
          "'No valid trajectories out of 33'")
    forward_only = [(o, h) for o, counts, _ in rows
                    for h, c in zip(headings, counts) if c == 16]
    if forward_only:
        print("      a count of 16 is the sixteen pivots surviving and no "
              "forward candidate at all")


def blame(rows):
    """Which critic did the refusing, over the whole sweep."""
    tally = collections.Counter()
    for _, _, blames in rows:
        for refused in blames:
            tally.update(refused)
    return tally


# --- the cycle ------------------------------------------------------------------
def plant(vx, wz):
    """`drive_mixer`: a standing turn under 12 deg/s does not clear stiction.

    So it is lifted to 12, and a rover asked for a two-degree correction gets
    a twelve-degree one. On a rover with no creep forward speed either, this
    is why "nearly lined up" is not a state this chassis can hold.
    """
    if abs(vx) < 0.05:
        dps = math.degrees(abs(wz))
        if dps < 1e-3:
            return 0.0, 0.0
        return 0.0, math.copysign(math.radians(max(MIN_TURN_DPS, dps)), wz)
    return vx, wz


def run(width_m, seconds=12.0, doorway=False, start_offset=0.30,
        start_heading_deg=90.0):
    """The controller, the abort and the costmap clear, at 10 Hz.

    The default start is the one the sweep says is the trap: 30 cm off the
    centreline of a metre-wide passage, which leaves 6 cm between the body and
    the wall, facing across it. That is a pose the rover reaches by drifting,
    not one anybody drives it to on purpose, and once it is there the
    controller cannot get it out.

    Everything with a clock in it runs at the rate the config gives it -- the
    controller at 10 Hz, the behaviour tree's replan at 1 Hz, the obstacle
    layer refilling a cleared costmap in one 5 Hz update, and `follow_path`
    aborting after 0.3 s with no command at all.
    """
    path = straight_path(corridor(width_m, doorway=doorway), 0.0, 0.0)
    goal = (path[-1][0], 0.0)
    oscillation = Oscillation()

    x, y, yaw = 0.0, start_offset, math.radians(start_heading_deg)
    real = corridor(width_m, doorway=doorway, clear_at=(x, y, yaw))
    grid = real
    vx_now = wz_now = 0.0
    dt = 1.0 / CONTROLLER_FREQUENCY
    dead_since = None
    refill_at = None
    aborts = clears = dead_ticks = flips = 0
    last_sign = 0
    events = []
    t = 0.0
    while t < seconds:
        # The obstacle layer clears the footprint every update, so the map
        # follows the rover rather than standing still.
        real = corridor(width_m, doorway=doorway, clear_at=(x, y, yaw))
        if refill_at is not None and t >= refill_at:
            refill_at = None
        grid = cleared(real) if refill_at is not None else real
        path_values = flood(grid, [grid.cell_of(px, py) for px, py in path])
        goal_values = flood(grid, [grid.cell_of(*goal)])
        kept, refused = evaluate(grid, path, goal, x, y, yaw, vx_now, wz_now,
                                 oscillation, path_values, goal_values)
        if not kept:
            dead_ticks += 1
            if dead_since is None:
                dead_since = t
            elif t - dead_since >= FAILURE_TOLERANCE_S:
                aborts += 1
                clears += 1
                worst = refused.most_common(1)[0][0] if refused else "?"
                events.append("%5.1f s  no command for %.1f s -- %s; "
                              "follow_path aborts, local costmap cleared"
                              % (t, t - dead_since, worst))
                # The recovery: clear the costmap, and start a fresh
                # follow_path, which resets every critic's memory.
                refill_at = t + 0.2
                oscillation.reset()
                dead_since = None
            vx_cmd = wz_cmd = 0.0
        else:
            dead_since = None
            _, vx_cmd, wz_cmd = kept[0]
        vx, wz = plant(vx_cmd, wz_cmd)
        sign = 0 if abs(wz) < 1e-6 else (1 if wz > 0 else -1)
        if sign and last_sign and sign != last_sign:
            flips += 1
            events.append("%5.1f s  turn reverses, now %s at %.0f deg/s"
                          % (t, "left" if sign > 0 else "right",
                             math.degrees(abs(wz))))
        if sign:
            last_sign = sign
        oscillation.debrief(x, y, yaw, vx, wz)
        x += vx * math.cos(yaw) * dt
        y += vx * math.sin(yaw) * dt
        yaw = wrap(yaw + wz * dt)
        vx_now, wz_now = vx, wz
        t += dt
    return {"x": x, "y": y, "yaw_deg": math.degrees(yaw), "aborts": aborts,
            "clears": clears, "dead_ticks": dead_ticks, "flips": flips,
            "events": events, "ticks": int(seconds * CONTROLLER_FREQUENCY)}


def render_run(width_m, result, offset, heading):
    print("%.0f s in a %.2f m passage, starting %+.2f m off the centreline "
          "and %.0f deg off it" % (result["ticks"] / CONTROLLER_FREQUENCY,
                                   width_m, offset, heading))
    for line in result["events"][:14]:
        print("  " + line)
    if len(result["events"]) > 14:
        print("  ... and %d more" % (len(result["events"]) - 14))
    print("  ticks with no command at all   %d of %d"
          % (result["dead_ticks"], result["ticks"]))
    print("  follow_path aborts             %d" % result["aborts"])
    print("  local costmaps cleared         %d" % result["clears"])
    print("  turn reversals                 %d" % result["flips"])
    print("  ended at (%.2f, %.2f) facing %.0f deg"
          % (result["x"], result["y"], result["yaw_deg"]))


def plan_sweep(widths, offsets):
    """What happens when the plan itself runs inside the inflated ring.

    `PathDist` floods outward from the plan and is stopped by 253. When every
    cell of the plan is *already* 253 there is nowhere to flood from, so every
    cell of the local costmap comes back "unreachable" and the critic refuses
    all thirty-three -- wherever the rover is standing and whichever way it is
    facing. This is the one failure that does not care about the rover's pose
    at all, which is why it looks like a rover that cannot move rather than a
    rover that cannot fit.
    """
    print("all thirty-three refused, by how far the plan runs off the centreline")
    print("      plan off centre  " + "".join("%8.1f m" % w for w in widths))
    for plan_y in offsets:
        cells = []
        for width_m in widths:
            if plan_y >= width_m / 2.0:
                cells.append("       -")
                continue
            dead = total = 0
            for offset in (-0.2, -0.1, 0.0, 0.1, 0.2):
                if abs(offset) >= width_m / 2.0 - 0.14:
                    continue
                for degrees in range(-180, 180, 30):
                    yaw = math.radians(degrees)
                    grid = corridor(width_m, clear_at=(0.0, offset, yaw))
                    path = [(i * RESOLUTION, plan_y) for i in range(31)]
                    kept, _ = evaluate(grid, path, path[-1], 0.0, offset, yaw)
                    total += 1
                    if not kept:
                        dead += 1
            cells.append("%5d/%2d" % (dead, total))
        print("         %+.2f m       %s" % (plan_y, "".join("%8s" % c
                                                             for c in cells)))
    print("      a plan inside the 14 cm inscribed ring leaves PathDist nothing")
    print("      to flood from, and every candidate is refused everywhere")


# --- the recoveries -------------------------------------------------------------
def collision_free(grid, x, y, yaw):
    """`CostmapTopicCollisionChecker::isCollisionFree`, at one pose.

    The footprint outline, refused at 254 and at nothing lower. Read off the
    rover's own `libnav2_costmap_2d_client.so`, where the double compared
    against is 0x406FC00000000000 -- 254.0 -- so the recoveries use the same
    true-lethal threshold the controller does, and are not being stopped by
    inflation they could safely sit in.
    """
    worst, _ = obstacle_footprint(grid, [(x, y, yaw)])
    return worst is not None


def room_to_turn(grid, x, y, yaw, limit=SPIN_DIST):
    """How far the body could rotate from here before it actually touched."""
    step = MAX_ROTATIONAL_VEL / CYCLE_FREQUENCY
    turned = 0.0
    while turned < limit:
        if not collision_free(grid, x, y, wrap(yaw + turned + step)):
            return turned
        turned += step
    return limit


def room_behind(grid, x, y, yaw, limit=BACKUP_DIST):
    """How far the body could reverse from here before it actually touched."""
    step = BACKUP_SPEED / CYCLE_FREQUENCY
    moved = 0.0
    while moved < limit:
        back = moved + step
        if not collision_free(grid, x - back * math.cos(yaw),
                              y - back * math.sin(yaw), yaw):
            return moved
        moved += step
    return limit


def spin_recovery(grid, x, y, yaw, target=SPIN_DIST,
                  simulate_ahead_s=SIMULATE_AHEAD_S):
    """`nav2_behaviors::Spin`, and why a wedged rover does not turn at all.

    Each cycle it works out the speed it wants, then projects that rotation
    forward `cycle_frequency * simulate_ahead_time` cycles and tests the
    footprint at every projected heading. One collision anywhere in that
    projection returns FAILED for the whole behaviour -- it does not turn as
    far as it can and stop there. At 0.5 rad/s over a 2 s horizon the
    projection is 0.95 rad, so a rover with twenty degrees of room is asked
    whether it has fifty-four, and told no.
    """
    max_cycles = int(CYCLE_FREQUENCY * simulate_ahead_s)
    turned = 0.0
    for _ in range(int(CYCLE_FREQUENCY * 60)):
        remaining = target - turned
        if remaining < 1e-6:
            return turned, "completed"
        speed = math.sqrt(2.0 * ROTATIONAL_ACC_LIM * remaining)
        speed = min(max(speed, MIN_ROTATIONAL_VEL), MAX_ROTATIONAL_VEL)
        for cycle in range(max_cycles):
            ahead = speed * (cycle / CYCLE_FREQUENCY)
            if remaining - abs(ahead) <= 0.0:
                break
            if not collision_free(grid, x, y, wrap(yaw + turned + ahead)):
                return turned, "collision ahead"
        turned += speed / CYCLE_FREQUENCY
    return turned, "out of time"


def backup_recovery(grid, x, y, yaw, target=BACKUP_DIST,
                    simulate_ahead_s=SIMULATE_AHEAD_S):
    """`nav2_behaviors::BackUp`, which is `DriveOnHeading` with a sign.

    The same shape and the same trap: 0.15 m/s over a 2 s horizon projects
    0.29 m, which is the whole 0.30 m the tree asked for, so a rover with ten
    centimetres behind it reverses none of them.
    """
    max_cycles = int(CYCLE_FREQUENCY * simulate_ahead_s)
    moved = 0.0
    for _ in range(int(CYCLE_FREQUENCY * 60)):
        remaining = target - moved
        if remaining < 1e-6:
            return moved, "completed"
        for cycle in range(max_cycles):
            ahead = BACKUP_SPEED * (cycle / CYCLE_FREQUENCY)
            if remaining - abs(ahead) <= 0.0:
                break
            back = moved + ahead
            if not collision_free(grid, x - back * math.cos(yaw),
                                  y - back * math.sin(yaw), yaw):
                return moved, "collision ahead"
        moved += BACKUP_SPEED / CYCLE_FREQUENCY
    return moved, "out of time"


def recovery_sweep(width_m, horizons):
    """What the recoveries get out of a rover pressed up against a wall.

    The offsets are the ones a rover in a metre-wide passage actually reaches:
    the body is 0.28 m across and the walls are a metre apart, so 0.30 m off
    the centreline leaves six centimetres, which is a drift rather than a
    driving error. `room` is what the geometry allows; the columns beside it
    are what each look-ahead lets the behaviour take of it.
    """
    limit = width_m / 2.0 - 0.14
    offsets = [o for o in (round(0.10 + 0.05 * i, 2) for i in range(9))
               if o < limit]
    wasted = 0
    print("a %.2f m passage, body 0.28 m across, walls at %.2f m"
          % (width_m, width_m / 2.0))
    print()
    print("  Spin, asked for 90 deg")
    print("      off centre  clearance   room  " +
          " ".join("  @%.1fs" % h for h in horizons))
    for offset in offsets:
        yaw = 0.0
        grid = corridor(width_m, clear_at=(0.0, offset, yaw))
        room = math.degrees(room_to_turn(grid, 0.0, offset, yaw))
        cells = []
        for horizon in horizons:
            turned, _ = spin_recovery(grid, 0.0, offset, yaw,
                                      simulate_ahead_s=horizon)
            cells.append("%5.0f  " % math.degrees(turned))
        first = math.degrees(spin_recovery(grid, 0.0, offset, yaw,
                                           simulate_ahead_s=horizons[0])[0])
        if room > 5.0 and first < 1.0:
            wasted += 1
        print("      %+5.2f m     %.2f m   %4.0f deg %s"
              % (offset, width_m / 2.0 - offset - 0.14, room, "".join(cells)))
    print()
    print("  BackUp, asked for 0.30 m, rover across the passage")
    print("      off centre  clearance   room  " +
          " ".join("  @%.1fs" % h for h in horizons))
    for offset in offsets:
        yaw = math.pi / 2.0
        grid = corridor(width_m, clear_at=(0.0, offset, yaw))
        room = room_behind(grid, 0.0, offset, yaw)
        cells = []
        for horizon in horizons:
            moved, _ = backup_recovery(grid, 0.0, offset, yaw,
                                       simulate_ahead_s=horizon)
            cells.append("%5.2f  " % moved)
        print("      %+5.2f m     %.2f m   %.2f m %s"
              % (offset, width_m / 2.0 - offset - 0.14, room, "".join(cells)))
    print()
    print("      'room' is what the body could do; the columns are what the")
    print("      behaviour takes of it at each simulate_ahead_time. A row where")
    print("      room is real and the first column is zero is a rover that")
    print("      could have moved and reported 'Collision Ahead' instead.")
    return wasted


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--width", type=float, default=1.0)
    parser.add_argument("--widths", default=None)
    parser.add_argument("--doorway", action="store_true",
                        help="a gap in a wall rather than a tube")
    parser.add_argument("--run", action="store_true",
                        help="drive the cycle rather than sweep the poses")
    parser.add_argument("--plan", action="store_true",
                        help="sweep how far off centre the plan runs")
    parser.add_argument("--recover", action="store_true",
                        help="what Spin and BackUp manage, by look-ahead")
    parser.add_argument("--horizons", default="2.0,0.5",
                        help="simulate_ahead_time values to compare")
    parser.add_argument("--seconds", type=float, default=12.0)
    parser.add_argument("--offset", type=float, default=0.30,
                        help="metres off the passage centreline to start")
    parser.add_argument("--heading", type=float, default=90.0,
                        help="degrees off the passage to start")
    args = parser.parse_args()

    if len(twists()) != CANDIDATES:
        print("the sample set is %d, not the %d the rover's log names -- "
              "config/nav2.yaml has moved" % (len(twists()), CANDIDATES))
        return 1

    if args.recover:
        horizons = [float(h) for h in args.horizons.split(",")]
        wasted = recovery_sweep(args.width, horizons)
        print()
        print("%d of the poses swept have room to turn and get nothing at "
              "%.1f s of look-ahead." % (wasted, horizons[0]))
        return 0

    if args.plan:
        plan_sweep([0.8, 0.9, 1.0, 1.2, 1.5],
                   [round(0.20 + 0.04 * i, 2) for i in range(8)])
        return 0

    if args.run:
        result = run(args.width, args.seconds, args.doorway,
                     args.offset, args.heading)
        render_run(args.width, result, args.offset, args.heading)
        if result["dead_ticks"]:
            print("\nreproduced: %d ticks with nothing to drive, %d aborts, "
                  "%d turn reversals" % (result["dead_ticks"],
                                         result["aborts"], result["flips"]))
            return 0
        print("\nnot reproduced: the controller had a command every tick")
        return 1

    widths = [float(w) for w in args.widths.split(",")] if args.widths \
        else [args.width]
    headings = list(range(0, 91, 15))
    dead = 0
    for width_m in widths:
        rows = sweep(width_m, headings, args.doorway)
        render(width_m, headings, rows)
        stuck = sum(1 for _, counts, _ in rows for c in counts if c == 0)
        total = sum(len(counts) for _, counts, _ in rows)
        dead += stuck
        print("      dead ticks %d of %d poses the body fits in" % (stuck, total))
        for reason, count in blame(rows).most_common(6):
            print("      %-52s %d" % (reason, count))
        print()

    print("%d of the poses swept lose all %d candidates. A clean passage is "
          "not the fault:" % (dead, CANDIDATES))
    print("the pose has to be all but touching a wall, and the obstacle layer "
          "clears the footprint")
    print("every update, so the rover carves itself free. Run --plan for the "
          "one that reproduces.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
