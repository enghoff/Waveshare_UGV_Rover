# Cutting a rim of frontier into pieces makes the rover pirouette, so it came back out

**Date:** 2026-09-08
**Moved:** [R-NAV-6](../requirements/navigation.md#r-nav-6) to `failing`

The question was whether the fix shipped on 2026-09-07 for a rover ringed by
unknown floor — cut any boundary over 2.00 m into pieces with a goal each — does
what it was meant to on the floor. It does not. It replaces a rover that reports
a finished house after 3.5 cm with a rover that turns 869 degrees to get 44 cm
from where it started, and then reports a finished house anyway. The cutting has
been reverted; the fault it was aimed at is open again and is now recorded as
such rather than as settled.

## What was measured

`ros_nav/fixtures/ringed-2026-09-07.json.gz`, replayed through
`python ros_nav/selftest.py` and through the driving model described below. That
fixture is the occupancy grid taken off
the running bridge on the Orin on 2026-09-07 while exploring was reporting a
finished map: about two square metres of mapped floor with unknown ground on
every side of the rover, 850 free cells against 26789 unknown, and the whole
boundary a single eight-connected clump of 209 cells — 10.45 m of it.

Three things were run against it, plus one live check on the rover.

**The chooser, cut and uncut.** `frontier.survey` on the recorded grid, at the
recorded pose, with the cap at 2.00 m and with it effectively off.

**The exploring loop, driven.** `frontier.Explorer` asked for a goal, moved to
it, wrote it off, and asked again, counting path, net displacement and the
turning `goto` would perform — the turn onto the bearing of the goal, then the
turn onto the arrival heading the frontier asked for. The recorded grid cannot
grow, so this is the pessimistic case: on the rover each arrival redraws the map.

**The whole-house simulation.** `explore_sim.py` over
`fixtures/kitchen-loop.pgm.gz`, cut and uncut, with the modelled lidar shortened
to 8.0, 4.0, 3.0, 2.0 and 1.5 m to try to make the island geometry appear.

**The rover as it stands.** The live occupancy grid saved off the Orin with
`map_saver_cli` at 07:30 and surveyed with the rover's own deployed
`frontier.py`, at the pose `nav_status` reported.

## The numbers

On the recorded rim, driven:

| | goals | path | turning | ends up from the start |
|---|---:|---:|---:|---:|
| uncut, which is where it is now | 1 | 0.03 m | 205 deg | 0.03 m |
| cut at 2.00 m, as shipped | 5 | 3.92 m | 869 deg | 0.44 m |

Cut, the rim comes out as six arcs, and every one of them is under a metre from
the rover: 0.44, 0.62, 0.79, 0.82, 0.85 and 0.95 m, of 1.20 to 2.10 m of
boundary each, facing 34, -55, -63, 128, -135 and 148 degrees. Six goals ringing
the rover pointing six different ways is what the 869 degrees is: the rover
spends nearly two and a half revolutions crossing its own two square metres, and
after five of the six it has run out — the sixth falls inside the half-metre
blacklist radius of one already driven — and reports nothing left to drive to.

**Nothing aboard would stop it.** The stall watcher gives up on a goal that has
not got 0.5 m further on in 25 seconds, and every one of these five steps is
between 0.44 m and 0.95 m, so the anchor resets on almost all of them. The rover
is genuinely moving; it is moving in a circle, and the watcher cannot tell the
difference.

The whole-house simulation shows none of this and cannot be used to argue either
way. Cut and uncut both finish `kitchen-loop` at every sight range tried, with
coverage between 99.5% and 99.8% and no separation in how far the rover gets from
its start in its first five metres of path:

| modelled sight | uncut | cut at 2.00 m |
|---|---|---|
| 8.0 m | 16 goals, 82.4 m, 99.6% | 18 goals, 78.6 m, 99.7% |
| 4.0 m | 15 goals, 83.8 m, 99.8% | 21 goals, 80.2 m, 99.8% |
| 3.0 m | 18 goals, 84.4 m, 99.7% | 22 goals, 83.0 m, 99.8% |
| 2.0 m | 24 goals, 86.0 m, 99.6% | 26 goals, 91.2 m, 99.6% |
| 1.5 m | 31 goals, 104.4 m, 99.5% | 31 goals, 94.2 m, 99.5% |

That is the same blindness the shipping commit named: `explore_sim`'s mapper
marks a cell free or wall for ever and never leaves unknown ground beside the
rover, so it cannot build an island for the rover to be ringed by. The cap was
measured round this house from twelve starting spots and found to cost nothing,
which was true and was not the question.

On the rover's live map this morning the cap does nothing either, in either
direction. The grid had 24322 free cells against 63452 unknown, 256 frontier
cells in 25 clumps with the biggest at 3.10 m — one clump over the cap out of
twenty-five. Uncut it offers 10 candidates and drives 8 goals for 33.94 m of
path; cut it offers 11 and drives 9 for 32.98 m, ending in the same place. A
well-mapped room does not care about this parameter. The island does.

## What was not measured, and it matters

**The pirouette was not observed on the rover.** It was reported from the
console, and it is reproduced here on a recorded grid that cannot grow. The
rover's map does grow at each arrival, so the replay is the pessimistic bound
rather than a prediction — what says the growth does not rescue the run is the
observation from the console, not this measurement. Reproducing it live would
mean clearing the map, and by the time this was looked at the rover had already
remapped the room.

## Which requirements moved

[R-NAV-6](../requirements/navigation.md#r-nav-6) — no frontier is longer than
one arrival can account for — goes from `settled` to `failing`. It was settled
on 2026-09-07 on the strength of the fixture replay and the twelve-start survey,
and neither of those looked at what the resulting goals do to a rover standing
in the middle of them. The requirement is still the right requirement. Cutting
the boundary up is not the way to meet it.

[R-NAV-5](../requirements/navigation.md#r-nav-5) is unaffected: exploring still
chooses reachable frontiers and still abandons a goal going nowhere, and the
stall watcher's own checks pass unchanged.

## What has to happen next

1. A rule that stops one arrival retiring a whole rim without producing a
   handful of sub-metre goals. The two candidates worth measuring are retiring
   only the part of a frontier the rover can be shown to have *seen* from where
   it stopped, rather than the whole clump it was aimed at; and declining to
   treat the rim of the rover's own island as frontier at all until the mapper
   has grown the island past some size. Neither has been tried.
2. Whichever is tried needs a driving model that lets the map grow, because the
   fault and the remedy both live in what happens after the first arrival, and
   neither the recorded grid nor `explore_sim` can show that today. This is the
   gap that let a fix ship on evidence that could not see its own cost.
3. Until then, exploring started from a freshly cleared map will report a
   finished house without having driven. That is on the record in
   `ros_nav/README.md` under known limits.
