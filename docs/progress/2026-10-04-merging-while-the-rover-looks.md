# Merging things while the rover is still looking makes the world worse; merging after does not

**Joining duplicate things once, after the last look of a session, is a clean gain, and
joining them on any schedule while looks are still arriving is not.** Replayed on map
session 67, one pass at the end kept 216 more pairs of looks at one object together and
put no further pair of different objects together. Every schedule that merged during the
session split more objects than it joined. That held when merging every minute, every
five minutes, only things quiet for five or fifteen minutes, only after the rover had been
idle for ten minutes, and without re-fitting the merged thing's position. The automatic
merge pass that was built for the daemon was therefore not deployed. The merge step
deployed this morning, which proposes and waits for a person, is unchanged.
[R-WS-13](../requirements/world-state.md#r-ws-13) stays `open`; this is the measurement
behind [R-WS-17](../requirements/world-state.md#r-ws-17).

Scripts and outputs are in `captures/2026-10-04-association-likelihood/`
(`replay_merge.py`, `merge_pw_*.txt`).

## How it was measured

The session was replayed look by look through the deployed resolver, with
`world_state/merging.py` applying every proposal on the schedule under test and once more
after the last look. Its appearance weights were refitted without the labelled things
being scored, in two halves. The labels are those of
[the morning's entry](2026-10-04-one-score-for-appearance-and-position.md).

The measure is symmetric, because the morning's per-thing measure is relative to the
things the deployed resolver built and stops meaning much once merging changes what it
builds. Every pair of labelled looks whose relation is known is counted: 4,357 pairs of
looks at one object, which should end up in one thing, and the pairs at two different
objects, which should not. Dining chairs against dining chairs are left out.

| Merging | Same-object pairs together (of 4,357) | Different-object pairs together | Things |
|---|---:|---:|---:|
| none (the resolver as deployed) | 3,721 | 710 | 182 |
| one pass after the last look | 3,937 | 710 | 155 and 160 |
| every 5 min | 3,325 | 488 | 143 and 145 |
| every 1 min | 3,306 | 545 | 145 and 147 |
| every 5 min, things quiet for 15 min | 3,526 | 425 | 144 and 150 |
| every 5 min, things quiet for 5 min | 3,065 | 394 | 143 and 145 |
| every 5 min, quiet 15 min, position not re-fitted | 3,332 | 504 | 145 and 153 |
| after the rover is idle for 10 min | 3,412 | 406 | 144 and 147 |

Merging during the session also keeps more wrong looks away from their object, which is
why the different-object pairs fall. But it parts far more right ones, so none of these
schedules improves on the resolver alone at both ends, and the pass after the last look
does.

## How the conclusion was reached

Each step was a replay of the same session, and each was chosen by what the one before
it showed.

1. **One pass after the resolver had finished** was tried first, in
   [the morning's addendum](2026-10-04-one-score-for-appearance-and-position.md): the
   look-alike pairs within 0.5 m fell from 24 to 1, three of the six labelled duplicate
   halves were rejoined, and no two labelled objects were put together. That is what
   made an automatic version worth building.
2. **The automatic version was replayed on a clock**, every five minutes and every
   minute, applying every proposal unreviewed. By the morning's measure, which compares
   each labelled thing with the thing that now holds most of its right looks, it kept
   fewer wrong looks with their object but parted 45 and 58 right ones, rejoined none of
   the labelled duplicates, and every minute put labelled objects together (table
   below).
3. **That measure was then distrusted**, because it is anchored to the things the
   deployed resolver built, and merging during the session changes what the resolver
   builds afterwards. The symmetric pairwise measure above was written and every
   schedule replayed again under it. It confirmed the first reading: the pass after the
   last look gained 216 same-object pairs at no cost, and both clocks lost about 400.
4. **Three explanations were tested, each by removing one cause.** If the harm came from
   merging things still being looked at, waiting until both things were quiet should
   remove it: quiet for five minutes lost 656 pairs and quiet for fifteen lost 195. If it
   came from merging while the rover was active at all, waiting until it had been idle
   for ten minutes should remove it: that lost 309. If it came from the merged thing's
   position being re-fitted, leaving the position alone should remove it: that lost 389.
5. **None did, so the conclusion is about when merging happens relative to the
   resolver, not about which clock.** Every schedule that lets the resolver go on
   building on merged things comes out below the resolver alone on same-object pairs,
   and only the pass after the last look comes out above it. The automatic pass was not
   deployed.

The per-thing measures of step 2, for every schedule, summed over the two halves:

| Merging | Wrong looks kept with their object (of 46) | Right looks parted from it (of 366) | Labelled objects put together | Other objects' right looks pulled in | Look-alike pairs within 0.5 m |
|---|---:|---:|---:|---:|---:|
| none | 46 | 0 | 0 | 0 | 24 and 24 |
| one pass after the last look | 46 | 0 | 0 | 0 | 1 and 1 |
| every 5 min | 29 | 45 | 0 | 5 | 4 and 4 |
| every 1 min | 26 | 58 | 3 | 16 | 5 and 6 |
| quiet for 15 min | 24 | 35 | 0 | 1 | 2 and 4 |
| quiet for 5 min | 25 | 49 | 0 | 1 | 3 and 4 |
| quiet 15 min, not re-fitted | 28 | 38 | 0 | 1 | 2 and 3 |
| idle for 10 min | 25 | 39 | 0 | 1 | 4 and 3 |

Under this measure the deployed resolver can part no right looks and keep every wrong
one by construction, since the labels were drawn from the things it built, which is why
step 3 was needed before drawing a conclusion.

## Why, as far as is known

Not traced. One mechanism fits, and it is untested. The region finder often draws two
boxes on one object, and the resolver lets a thing take one region per picture. While
the object is two things, each box has a home. Once they are one thing, the second box in
every later picture has none, so it waits or founds another thing, and that thing shares
pictures with the merged one, which the merge rule never joins.

The merged thing's position was the other suspect. Re-fitted from its newest 24 looks it
can move, but leaving it unchanged did no better.

## The merge rule sums what a settled requirement keeps apart

[R-WS-8](../requirements/world-state.md#r-ws-8) holds that geometry decides where a thing
can be before appearance chooses, because a single fused score let strong appearance
rescue impossible geometry in September. The merge score adds the two. Its rule that the
crops on their own must favour one object does not stop that: two things standing far
apart could still be joined on appearance. A hard limit on the ellipse distance, applied
before scoring, would keep to the requirement. It changes none of the 18 proposals the
rover made today, and it is owed before any further merging code is deployed.
