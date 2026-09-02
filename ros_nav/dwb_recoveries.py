#!/usr/bin/env python3
"""The recovery behaviours, and the room they need to run in.

Nav2's stock spin and back-up assume space that a rover wedged in a doorway does
not have, which is what `escape_spin` and `escape_drive_on_heading` exist to fix:
they ask the costmap how far they can actually turn or reverse before committing,
and give up cleanly instead of grinding. `room_to_turn` and `room_behind` are that
question.

Re-exported by corridor_sim.py, which is what `import corridor_sim as dwb` gets.
"""

import math

from dwb_config import (
    BACKUP_DIST, BACKUP_SPEED, CIRCULAR, CYCLE_FREQUENCY, MAX_ROTATIONAL_VEL,
    MIN_ROTATIONAL_VEL, ROTATIONAL_ACC_LIM, SIMULATE_AHEAD_S, SPIN_DIST, wrap,
)
from dwb_grid import collision_free, corridor


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


def drive_on_heading(grid, x, y, yaw, target=0.5, sign=1.0, speed=BACKUP_SPEED,
                     simulate_ahead_s=SIMULATE_AHEAD_S):
    """`nav2_behaviors::DriveOnHeading`, forward when `sign` is +1.

    `backup_recovery` above is this with the sign flipped -- that is literally
    how Nav2 builds it, one template instantiated for each action -- and it is
    written out separately here because the fault worth studying is what happens
    when the rover is asked to drive *away* from what is blocking it.
    """
    max_cycles = int(CYCLE_FREQUENCY * simulate_ahead_s)
    moved = 0.0
    for _ in range(int(CYCLE_FREQUENCY * 60)):
        remaining = target - moved
        if remaining < 1e-6:
            return moved, "completed"
        for cycle in range(max_cycles):
            ahead = speed * (cycle / CYCLE_FREQUENCY)
            if remaining - abs(ahead) <= 0.0:
                break
            gone = moved + ahead
            if not collision_free(grid, x + sign * gone * math.cos(yaw),
                                  y + sign * gone * math.sin(yaw), yaw):
                return moved, "collision ahead"
        moved += speed / CYCLE_FREQUENCY
    return moved, "out of time"


# --- the escape behaviours ----------------------------------------------------
#
# `behaviors/` replaces Nav2's Spin, DriveOnHeading and BackUp with subclasses
# that differ in exactly one state, and these model that difference so it can be
# checked without a rover. The state is: Nav2 has refused with COLLISION_AHEAD
# *and* the rover's own pose is already in collision.
#
# Everywhere else the models above still apply, and that is the point -- the
# plugins call Nav2's implementation and return its answer untouched unless both
# of those hold.


def escape_spin(grid, x, y, yaw, target=SPIN_DIST,
                simulate_ahead_s=SIMULATE_AHEAD_S, circular=None):
    """`ugv_behaviors::EscapeSpin`: a circular-footprint rover always turns.

    The footprint is a circle centred on `base_link`, which is the point the
    rover turns about, so a rotation maps the body exactly onto itself and every
    heading covers the same ground. If the rover fits where it stands it fits at
    every heading; if it does not, no heading helps. Nav2's check can therefore
    only agree with the current pose or disagree with it wrongly -- and it does
    disagree, because it does not test a circle. A radius becomes a sixteen-sided
    polygon whose outline is walked across a 5 cm grid, so rotating it clips a
    slightly different set of cells and one marginal cell refuses the turn.
    Watched on the rover: a 180 degree turn refused while it stood in open floor.

    `circular` defaults to the footprint this module is configured with, and the
    plugin measures the same thing off the published footprint rather than
    trusting a setting. A non-circular body keeps Nav2's check except when it is
    already in contact, where refusing would trap it.
    """
    turned, why = spin_recovery(grid, x, y, yaw, target, simulate_ahead_s)
    if why != "collision ahead":
        return turned, why
    if circular is None:
        circular = CIRCULAR
    if not circular and collision_free(grid, x, y, yaw):
        return turned, why
    return target, "escaped"


def escape_drive_on_heading(grid, x, y, yaw, target=0.5, sign=1.0,
                            speed=BACKUP_SPEED,
                            simulate_ahead_s=SIMULATE_AHEAD_S):
    """`ugv_behaviors::EscapeDriveOnHeadingAction`, and `...BackUpAction`.

    A rover in contact may drive a heading whose projection *ends* clear, which
    is the arithmetic for "this motion leads out of the contact rather than
    deeper into it". Driving forward off something behind passes; reversing into
    that same thing does not, and neither does driving forward into a wall while
    something is behind -- the wedged case, where no is still the honest answer.
    """
    moved, why = drive_on_heading(grid, x, y, yaw, target, sign, speed,
                                  simulate_ahead_s)
    if why != "collision ahead":
        return moved, why
    if collision_free(grid, x, y, yaw):
        return moved, why
    max_cycles = int(CYCLE_FREQUENCY * simulate_ahead_s)
    for cycle in range(max_cycles - 1, 0, -1):
        gone = speed * (cycle / CYCLE_FREQUENCY)
        if collision_free(grid, x + sign * gone * math.cos(yaw),
                          y + sign * gone * math.sin(yaw), yaw):
            return target, "escaped"
    return moved, why


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
