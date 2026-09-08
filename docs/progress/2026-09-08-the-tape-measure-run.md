# Four measurements against a tape, and the geometry passes its declared tolerance

**The rover placed three named objects and got every separation right to within
12 cm, against a tolerance of 30 cm written down before it moved.** That is the
first time this rover's geometry has been tested the way
[M0](../plans/autonomous-curiosity.md) asks — predeclared tolerances, on a
recording the thresholds had never seen — and it passes. It is also the first
time the vertical has ever been checked at all.

Identity did not pass, on the same recording.

## What was measured

The owner put out three objects and measured between them with a tape before
driving. The store was cleared and the map kept and confirmed settled first; the
run is 293 looks over 103 seconds from 28 standing places inside about two metres
square, archived as `~/.ugv/archive/world-2026-09-08-targets.db`.

| pair | tape | the rover | out by |
|---|---|---|---|
| green tissue box to bucket, across the floor | 0.75 m | 0.837 m | **+0.087** |
| green tissue box to bucket, in height | 0.75 m | 0.859 m | **+0.109** |
| green tissue box to brown box, across the floor | 1.20 m | 1.082 m | **−0.118** |
| green tissue box to brown box, in height | 0.00 m | 0.088 m | **+0.088** |

All four inside the 0.30 m declared in
[the acceptance runbook](../runbooks/m0-acceptance-drive.md), and all four inside
what the rover claimed for itself — it put the pairs at 0.25 m and 0.22 m of
combined uncertainty and beat both. The two floor errors have opposite signs, so
this is not a scale error with a single cause.

**The height result matters more than its size suggests.** Nothing had ever
checked the vertical half of the geometry, and
[R-WS-11](../requirements/world-state.md#r-ws-11) is still `open` because height
above the *floor* is unavailable. Height *between two things* is now measured and
right to 11 cm, which says the elevation half of a ray works and what is missing
is only the datum.

Each target was also correctly one thing: the green box 7 looks from 4
viewpoints, the bucket 10 from 5, the brown box 4 from 3, and none of the three
holds a look at anything else.

## Identity still fails, on a recording the remedies had not seen

Four of the 29 things hold looks at two different objects — a cabinet pooled with
framed pictures, chairs pooled with pictures, and **twice again a person sitting
down joined to the armchair they are sitting in**. Against 221 attachment
decisions that is a criterion 3 failure, and it is the honest verdict on the
three remedies deployed today: the ratio test, conservative learning and the
collapse test were all running, and the fault rate fell from 22% of things to 14%
rather than to zero.

The person-into-furniture case is now seen on three separate recordings and is
the single most repeated fault this rover has.

## What this run cannot say

It is 103 seconds inside a two-metre box, not the fifteen minutes the runbook
asks for, so:

- **the range check is thin and reads worse**: 22 of 37 ranges within half a
  metre, 59%, against 69% on the morning's drive and a 70% pass mark. Short
  baselines make weak parallax ground truth, and the worst offenders are a bright
  window read at 5 m where the crossing says 2.1 m. Thirty-seven ranges is too
  few to move anything either way.
- **there was no navigation restart**, so nothing here touches criterion 11 —
  which is settled already.
- 160 of 293 looks, 55%, could not be ranged because the region sat outside the
  depth camera's view.

## Requirements

- M0's geometry tolerance passes on held-out trials for the first time. The
  criterion also wants the whole envelope exercised, which two metres of driving
  does not do, so this is evidence toward criterion 7 rather than a pass of it.
- [R-WS-11](../requirements/world-state.md#r-ws-11) stays `open`; relative height
  is now measured good to 11 cm and the missing piece is the floor datum.
- [R-WS-13](../requirements/world-state.md#r-ws-13) stays `open`: 4 of 29 things
  are two objects, with all three remedies deployed.
- [R-WS-10](../requirements/world-state.md#r-ws-10) is untouched. The bearing
  spread on this recording is contaminated by those four merges.

## Next

The same three targets, driven properly — fifteen minutes, the whole room, each
target from standing places metres rather than centimetres apart. This run proves
the measurement works; it is too small to certify anything that needs a
population. The tape numbers should be reused rather than remeasured, because
they are the part that cannot be recovered afterwards.
