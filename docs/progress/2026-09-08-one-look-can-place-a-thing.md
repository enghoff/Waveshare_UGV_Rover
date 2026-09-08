# A measured distance can now stand a thing up on its own

**The rover can place something it has only seen from one place, if it measured
how far away it was.** Until today a thing came into existence only where two
bearings crossed, which needs two viewpoints far enough apart and far enough
turned; a look that measured the distance in millimetres could not found
anything, and the reading was recorded, shown in the console and then dropped.

Deployed to the Orin at `012cdf9` and verified there: the running daemon now
holds 127 things, 109 from crossings and **18 asserted by a single ranged look**,
found in its own backlog after the restart.

## Why it changed

[The drive of this morning](2026-09-08-acceptance-drive-two.md) made the case
concrete. A spray can on the floor was seen twice, four seconds apart, from
standing places 0.37 m apart with 1.1 degrees between the bearings — no crossing
exists there at any tolerance, and the resolver was right to refuse it. Both
looks had measured the distance to it, 1.077 m and 0.585 m. Across that drive 38
of 104 things were never ranged at all and 387 of 1328 looks attached to nothing.

`locate.py` had argued against this in a comment, on the grounds that everything
the component knows about identity was learnt from crossings and a one-look rule
is a different application from the one that was measured. That caution is
answered by keeping the two apart rather than by refusing: a thing placed this
way records `rays_agreeing` 1 and `viewpoints` 1, both of which the entity
listing already returns, so nothing downstream can mistake it for a thing several
rays agree on.

## What it does

One ray plus its measured distance gives a point. The error is reported as the
ellipse it really is rather than as a radius: centimetres along the sight line,
where the depth camera is good, and the bearing's own error opened out over the
whole distance across it — at three metres and 1.5 degrees that is already 8 cm.
On the rover just now, `object:110` sits 0.037 m wide from a measured 1.371 m and
`object:127` 0.45 m wide from 5.853 m, which is that widening doing its work.

Two rules keep it in its place. It runs **only after every crossing in the pool
is exhausted**, so better evidence always wins; and it refuses to stand a thing
up inside an existing thing's uncertainty, because a duplicate is worse than the
silence it replaces — it looks like knowledge.

## What it cost, measured on the drive

Replaying the whole recording, against the same recording replayed by the
deployed build:

| | things | looks attached | the spray can |
|---|---|---|---|
| crossings only | 120 | 974 of 1328 | not placed |
| with one-look placement, no duplicate guard | 217 | 1086 | placed |
| with the guard | 155 | 1025 | placed, both looks on one thing |

The guard is what makes it usable: without it, 57 of the 100 new things sat
within half a metre of a thing a crossing had already placed. With it, 45 things
of which 16 do — and 27 of the 45 went on to collect a second look, which is a
thing being confirmed rather than clutter accumulating.

**Two costs are worth stating plainly.** Eighteen of the 45 still hold exactly
one look and nothing has confirmed them. And the crossings themselves drop from
117 to 110, because a look that founds a thing early is a look no longer waiting
around to cross with something later — the change does not only add.

## Requirements

- [R-WS-13](../requirements/world-state.md#r-ws-13) stays `open`. This is about
  what can be placed, not about whether an association is right, and the three
  merges that recording contains are untouched by it.
- No M0 criterion moves. Criterion 2's "usable ranges where expected" is helped
  in spirit, but the acceptance measurement is a driven recording and this
  changes what a future one would contain rather than what the last one did.
- Counts: world_state 806 passed, 0 failed, which includes the two checks the
  Phase 1 generation token brought with it.

## Next

The one-look things want looking at by eye on the next drive. Eighteen of them
exist on the rover now and nobody has seen what they are; the question is whether
a thing asserted by one ranged look is usually a real object the rover simply
never got a second angle on, or usually a region that should not have been a
thing at all. That is a review, and it is the same contact-sheet method the
merges were found with.
