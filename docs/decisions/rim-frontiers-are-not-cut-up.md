# A rim of unknown round the rover is not fixed by cutting it into pieces

Status: closed 2026-09-08, having been implemented on 2026-09-07 and reverted the
next day. The fault it was aimed at is open:
[R-NAV-6](../requirements/navigation.md#r-nav-6) is `failing`.

A rover that has just been given a cleared map stands on a small island of mapped
floor with unknown ground on every side of it. Exploring writes a frontier off
once it has driven to it, and offers a clump one goal at the member cell nearest
the clump's centre of mass — so the whole rim of the island is one frontier, its
centre of mass is the middle of the ring, which is where the rover is standing,
and one arrival at a goal 3.5 cm away retires ten metres of unexplored edge. The
rover then reports a finished house with 97% of the map unknown, having driven
3.5 cm. That happened on the Orin on 2026-09-07 and the grid is kept as
`ros_nav/fixtures/ringed-2026-09-07.json.gz`.

**The obvious remedy is to cut the rim up, and it was implemented, measured,
shipped and taken back out inside a day.** If one goal cannot stand for ten
metres of boundary, cut any boundary over a cap into pieces with a goal and a
blacklist entry each, so that an arrival retires a piece of a rim rather than the
whole of one. That is right about where the fault is. What it gets wrong is what
the pieces then are.

Cut at 2.00 m, the recorded rim becomes six arcs, and the geometry that made the
uncut version wrong makes the cut version wrong in the other direction: the rim
is *at arm's length all the way round*, so all six goals are under a metre from
the rover — 0.44 to 0.95 m — and they point in six different directions, at 34,
-55, -63, 128, -135 and 148 degrees. Driven, that is five goals, 3.92 m of path
and **869 degrees of turning** to finish 0.44 m from where it started, still on
the same two square metres, and then a report that there is nothing left to drive
to anyway because the sixth arc fell inside the blacklist radius of one already
driven. Nearly two and a half revolutions bought 44 cm. The numbers and the
method are in
[2026-09-08-rim-frontiers-pirouette.md](../progress/2026-09-08-rim-frontiers-pirouette.md).

**Nothing aboard catches it**, which is what settles the argument rather than
taste. The stall watcher abandons a goal that has not got 0.5 m further on in
25 seconds; each of these steps moves 0.44 m to 0.95 m, so the anchor resets and
the rover is correctly judged to be moving. It is moving in a circle. A rover
turning 869 degrees to cross two square metres is a worse thing for a person to
watch than a rover that reports a finished house and stops, and it is not a
better answer.

**The evidence the cutting shipped on was real and did not cover this.** The cap
was chosen by running the policy round the recorded `kitchen-loop` house from
twelve starting spots and finding that coverage did not move and driving moved by
less than one map's geometry is worth. That measurement was sound and is still
true. It could not see this cost, because `explore_sim.py`'s mapper marks a cell
free or wall for ever and never leaves unknown ground beside the rover, so it
cannot build an island for the rover to stand in the middle of — the shipping
commit said as much about the fault and did not draw the conclusion for the
remedy. And the fixture replay that did have the right geometry held the rover
still, so it measured which goals were offered and never what driving to them in
turn does. **A remedy measured on a model that cannot represent the fault's
geometry is measured on the wrong thing**, however careful the measurement is.

On a well-mapped room the cap is inert in both directions, which is worth knowing
before anybody reintroduces it hoping for a general gain. The rover's live map on
2026-09-08 had 256 frontier cells in 25 clumps with one over the cap: uncut it
drove 8 goals for 33.94 m, cut it drove 9 for 32.98 m, ending in the same place.

## What would reopen it

Not a different cap. The pirouette is not a tuning failure — every cap tight
enough to divide a ten-metre rim produces goals inside the rim, because that is
where the rim is. Reopening this needs one of:

- **A retirement rule instead of a division rule.** An arrival writes off the
  part of the frontier the rover can be shown to have seen from where it
  stopped, rather than the clump it was aimed at. That attacks the actual faulty
  assumption and produces no extra goals at all.
- **Not treating the rim of the rover's own island as frontier** until the
  mapper has grown the island past some size, so the first seconds after a map
  clear are spent letting the map settle rather than driving round a two-metre
  patch.

Either has to be measured on a model where the map grows as the rover drives,
against the ringed geometry. Neither the recorded grid — which cannot grow — nor
`explore_sim` — which cannot ring the rover — can do that today, and building
that is part of the price of the next attempt. A candidate has to show, on the
ringed fixture with growth, that the run leaves the island, and on
`kitchen-loop` that coverage and driving are no worse than the 16 goals and
82.4 m the uncut policy gets there.

## Requirements

Supports [R-NAV-6](../requirements/navigation.md#r-nav-6), which this leaves
`failing` rather than retiring: the requirement that no frontier be longer than
one arrival can account for is still the right requirement, and the rover does
not currently meet it. Retires nothing. [R-NAV-11](../requirements/navigation.md#r-nav-11)
— navigation faults are reproducible without the rover — is the one this episode
is a warning about: both models used here are reproducible and neither could
represent the thing being changed.
