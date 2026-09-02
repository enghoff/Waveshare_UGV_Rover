#!/usr/bin/env python3
"""Costmaps: building one, inflating it, and asking what is where.

The inflation is the part worth having exactly right, because it is what turns a
wall into a region the controller will not enter, and its shape decides whether a
metre-wide passage has any legal middle at all. `flood` answers what is reachable,
`base_obstacle` and `obstacle_footprint` are the two critics that read the grid,
and `corridor` builds the passage the fault was found in.

Re-exported by corridor_sim.py, which is what `import corridor_sim as dwb` gets.
"""

import collections
import math

import goal_fit
from dwb_config import (
    COST_SCALING_FACTOR, FOOTPRINT, INFLATION_RADIUS, INSCRIBED_M, OBSTACLE_SCORE,
    RESOLUTION, UNREACHABLE_SCORE, _lower_envelope, special_scores
)


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


def base_obstacle(grid, poses):
    """`BaseObstacleCritic`, which is the right critic once the body is a circle.

    It reads the one cell the robot's centre is in and refuses anything that is
    not free. `isValidCost` in `libdwb_critics.so` is a single comparison --
    `cmp w1, #0xfc; cset w0, ls` -- so a cost is valid exactly when it is 252 or
    less, and 253, 254 and 255 are all refusals.

    That is a *point* test, and it is only a collision test while the inflation
    layer's inscribed ring is as big as the whole robot. With the measured
    rectangle it was not, which is why `ObstacleFootprint` replaced it. With a
    circular `robot_radius` it is again, exactly -- and it is now stronger than
    the footprint critic ever was, because that one only refused on contact at
    254 while this refuses at 253, the ring. The ring is 0.244 m deep, so the
    controller can no longer drive the rover into the band where it would not
    be able to turn round.

    `aggregation_type` is "last" and `sum_scores` is off, so every pose can
    refuse the candidate but only the final one's cost is scored.
    """
    last = 0.0
    for x, y, _yaw in poses:
        col, row = grid.cell_of(x, y)
        if not (0 <= col < grid.width and 0 <= row < grid.height):
            return None, "BaseObstacle: trajectory goes off grid"
        cost = grid.cost(col, row)
        if cost > 252:
            return None, "BaseObstacle: trajectory hits obstacle"
        last = float(cost)
    return last, ""


def reinflate(grid, inscribed_m=None, inflation_m=None, scaling=None):
    """Rebuild a recorded costmap's inflation at a different inscribed radius.

    A recording carries the costmap the rover had, inflated under the settings
    of the day. Testing a change to the robot's *shape* against it means
    redoing that arithmetic, because the shape is what sets the ring. Only the
    lethal cells are kept -- they are the observations; everything else in the
    grid was derived from them and can be derived again.

    The formula is `InflationLayer::computeCost` and it was checked against a
    recorded map before being trusted: fitting cost against distance over 2931
    gradient cells returned a scaling factor of 3.00 and an inscribed radius of
    0.149 m, against the 3.0 and 0.14 the config asked for.
    """
    inscribed_m = INSCRIBED_M if inscribed_m is None else inscribed_m
    inflation_m = INFLATION_RADIUS if inflation_m is None else inflation_m
    scaling = COST_SCALING_FACTOR if scaling is None else scaling
    lethal = [(c, r) for r in range(grid.height) for c in range(grid.width)
              if grid.cost(c, r) == goal_fit.LETHAL]
    unknown = set((c, r) for r in range(grid.height) for c in range(grid.width)
                  if grid.cost(c, r) == goal_fit.UNKNOWN)
    reach = int(math.ceil(inflation_m / grid.resolution))
    data = [0] * (grid.width * grid.height)
    best = {}
    for lc, lr in lethal:
        for dr in range(-reach, reach + 1):
            for dc in range(-reach, reach + 1):
                nc, nr = lc + dc, lr + dr
                if not (0 <= nc < grid.width and 0 <= nr < grid.height):
                    continue
                away = math.hypot(dc, dr) * grid.resolution
                if away > inflation_m:
                    continue
                if away < best.get((nc, nr), 1e9):
                    best[(nc, nr)] = away
    for (c, r), away in best.items():
        if away == 0.0:
            data[r * grid.width + c] = goal_fit.LETHAL
        elif away <= inscribed_m:
            data[r * grid.width + c] = goal_fit.INSCRIBED
        else:
            data[r * grid.width + c] = int((goal_fit.INSCRIBED - 1) * math.exp(
                -scaling * (away - inscribed_m)))
    for c, r in unknown:
        data[r * grid.width + c] = goal_fit.UNKNOWN
    return goal_fit.CostGrid(grid.width, grid.height, grid.resolution,
                             grid.origin_x, grid.origin_y, data)


def obstacle_footprint(grid, poses):
    """`ObstacleFootprintCritic`: the outline, every pose, 254 and 255 only.

    Verified against the rover's own `libdwb_critics.so` rather than assumed:
    `pointCost` compares the cell against 0xfe and 0xff and has no third
    comparison, so the 253 ring the inflation layer paints along every wall is
    a cost here and not a refusal. The refusal comes from the map-grid critics
    below, which is a much easier thing to miss.
    """
    # `BaseObstacleCritic::scoreTrajectory` with `sum_scores` off -- which is
    # the default and is what this rover runs -- keeps only the *last* pose's
    # cost, however bad the ones on the way there were. Every pose can still
    # refuse the candidate outright; only the score is the last one's. Taking
    # the worst instead quietly makes the wall matter along the whole rollout.
    last = 0.0
    for x, y, yaw in poses:
        worst = 0.0
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
        last = worst
    return last, ""


def flood(grid, seeds):
    """`MapGridCritic`'s propagation: distance in cells, and **walls do not stop it**.

    Seeded from the path or from the goal and flooded four-connected, exactly
    as `CostmapQueue` does it, `propogateManhattanDistances` writing
    `|dx| + |dy|` from the source into every cell the queue reaches.

    **The obstacle check that every account of this critic describes is not in
    the library this rover runs**, and modelling it was the largest error this
    file has had. The upstream version of `MapGridQueue::validCellToQueue`
    reads the costmap and refuses 253, 254 and 255, marking them
    `obstacle_score_`; in the `libdwb_critics.so` installed here it is two
    instructions::

        dwb_critics::MapGridCritic::MapGridQueue::validCellToQueue:
            mov  w0, #0x1
            ret

    and nothing in the whole library calls `MapGridCritic::setAsObstacle`. So
    the flood runs straight through walls and out the other side, no cell is
    ever marked obstacle, and once a critic has one seed on the window every
    cell of it has a real distance. `unreachable_score_` therefore survives
    only when a critic gets *no* seed at all -- the whole plan off the window
    for `PathDist`, or `getLastPoseOnCostmap` finding nothing for `GoalDist` --
    and then it lands on every candidate at once rather than picking between
    them.

    Modelled the other way, with the flood stopped at the inflated ring, the
    model refused all twenty-nine candidates on 329 of the 511 ticks of
    `recordings/trap-2026-08-25-spin.json`, because the plan's far end lay
    inside the ring and the rover was sealed off from it. The rover itself
    commanded a pivot on every one of those ticks and its log has no "could
    not find a legal trajectory" in the minute. Correcting the flood took the
    model's agreement with that drive from 15% to 84%.
    """
    values = [UNREACHABLE_SCORE] * (grid.width * grid.height)
    queue = collections.deque()
    for col, row in seeds:
        if not (0 <= col < grid.width and 0 <= row < grid.height):
            continue
        index = row * grid.width + col
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
            values[index] = distance + 1.0
            queue.append((nc, nr, distance + 1.0))
    return values


def map_grid_score(grid, values, x, y, name, stop_on_failure=True):
    """`MapGridCritic::scorePose`, which is a point test, not a body test.

    **Only two of the four map-grid critics can refuse anything, and that was
    the model's largest error.** `MapGridCritic::onInit` sets `stop_on_failure_`
    true, and both align critics overwrite it with zero immediately after
    calling their base -- in `libdwb_critics.so`, at offset 176 of the critic::

        MapGridCritic::onInit    mov  w0, #0x1 ; strb w0, [x19, #176]
        PathAlignCritic::onInit  blr  x1      ; strb wzr, [x19, #176]
        GoalAlignCritic::onInit  blr  x1      ; strb wzr, [x19, #176]

    `scoreTrajectory` reads that byte twice. It decides whether the obstacle
    and unreachable values throw, and it decides how much of the rollout is
    looked at: with the flag clear the loop starts at `size - 1`, so an align
    critic sees the last pose and nothing else.

    Going off the grid is different and refuses either way, because that throw
    lives in `scorePose` where the flag cannot reach it.
    """
    col, row = grid.cell_of(x, y)
    if not (0 <= col < grid.width and 0 <= row < grid.height):
        return None, "%s: trajectory goes off grid" % name
    value = values[row * grid.width + col]
    obstacle, unreachable = special_scores(grid)
    if value == OBSTACLE_SCORE:
        if stop_on_failure:
            return None, "%s: trajectory hits obstacle" % name
        return obstacle, ""
    if value == UNREACHABLE_SCORE:
        if stop_on_failure:
            return None, "%s: trajectory hits unreachable area" % name
        return unreachable, ""
    return value, ""


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
