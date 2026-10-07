# Why one object becomes many records, and two changes that did not fix it

**Duplicates of one object are founded where the object's bearings happen to
cross, and those crossings scatter far more than the resolver assumes.** On the
labelled looks of map session 67, a look's bearing misses its own object by a
median 1.5°, but a quarter miss by more than 3.8° and one in ten by more than
7°. The resolver assumes 1.5° throughout. About 70% of the miss is shared by
every region of one picture, and it is 2.5 times worse in pictures taken while
the rover moved. For things behind low furniture there is a second cause. The
lidar sees the legs of the dining table and chairs, and a crossing is not
allowed beyond the first leg. So the dining painting is placed over the chairs,
somewhere different from each viewpoint. Two changes aimed at these causes were
replayed over the whole session. Neither is better than the bench's own noise
on balance. R-WS-17 and R-WS-18 stay proposed.

## The bench

`replay_session.py` (new) replays all 2,094 looks and 15,624 regions of map
session 67 through today's resolver. It keeps observation identifiers, so the
labels apply, and it logs every attachment, placement and founding.
`score_session.py` (new) scores a replay against three frozen label sets:
2026-10-03 (25 things, 412 looks), the 2026-10-05 depth drive, and the
2026-10-07 trial. It also counts how many records hold four objects that are
certainly the same across days. The replay is not the rover: it uses one
archived map and today's code. It made 374 records where the rover's store has
395, with similar scores. The cosine similarity runs in numpy on the bench,
which changed none of 779 owners over 150 looks.

Its noise was measured by running the baseline twice more, each leaving out a
different random 2% of unlabelled regions:

| | split of 16 (10-03) | wrong looks (10-03) | wrong links | green painting: records, largest |
|---|---|---|---|---|
| baseline | 6 | 12.3% | 10 | 11, 25% |
| noise, seed 1 | 6 | 11.0% | 5 | 10, 36% |
| noise, seed 2 | 6 | 10.1% | 5 | 10, 42% |
| heading correction | 10 | 12.4% | 43 | 10, 33% |
| see past legs | 9 | 11.1% | 20 | 9, 50% |

## What was measured

- **Bearings.** Among 22 labelled objects with views from separated places
  (296 looks): median miss 1.47°, 75th percentile 3.8°, 90th 7.2°, 95th 11.9°.
  Measured against their own stated error, 15% of still looks and 26% of moving
  ones miss by more than two sigma, against an expected 5%. The miss does not
  follow pan angle (r = -0.01) and barely follows place in the frame.
- **Shared per picture.** In 45 looks holding labelled regions of two or more
  objects, the two objects' signed misses correlate at 0.49. The per-look shift
  varies with a spread of 3.5°, and what is left within a look 2.1°.
- **Duplicates are founded apart, not on top.** Of the 10-03 objects' duplicate
  records, most were founded while the original stood further from the new
  crossing than either record's stated uncertainty. Where they were close, the
  founding looks were refused by geometry or appearance. No founding broke the
  resolver's own rules.
- **Reach stops at legs.** Three quarters of ranged labelled looks measured
  their object more than 0.3 m past the lidar's first obstacle. For the dining
  painting it was 2.3 m, with the depth point on the wall. `locate.fix` refuses
  crossings beyond reach, so the painting's records sat at y -12.0 to -13.4,
  0.5 to 1.2 m up; the painting hangs on the wall at y -11.0, 2 m up.
- **Appearance is weak for this painting.** Labelled views of it score a median
  0.67 against each other, and a fifth of pairs fall below the 0.55 floor at
  which the resolver calls two crops different things.

## The two changes

- **Heading correction** (`look_alignment`): a look whose regions match two or
  more well-established things that agree on a shift is turned back by it before
  matching. It shifted 1,092 looks by a median 2.15°. It splits more and links
  four times as many different objects, beyond the noise. A likely cause is that
  the anchors are themselves misplaced, so a look gets turned by the wrong
  amount.
- **See past legs** (`legs_transparent`): lidar blobs no wider than 0.30 m
  (legs and poles) do not end a bearing, while walls and furniture still do. It
  put the painting's main record on the wall at the right height, with 18 of 36
  looks against 9. But it splits more objects and doubles the wrong links,
  beyond the noise.

Thresholds for both were fixed before scoring. Neither is deployed.

## What it means

Gate-level changes to an online resolver that decides each look once trade one
failure for another. That matches the earlier experiments listed in
[the world state plan](../plans/semantic-world-state.md#one-thing-per-object-and-only-that-objects-looks).
The causes found here are real and measured. The bench is now able to tell a
change from its own noise, so the next proposal can be judged in an afternoon.
