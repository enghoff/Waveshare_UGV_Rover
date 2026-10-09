# M4 scores a placement by its distance to the thing, not to one taped point on it

Status: proposed 2026-10-09, for the owner. Changes how
[m4-measures-where-things-are.md](m4-measures-where-things-are.md) scores a
placement against the tape; its comparison, its re-look and its other criteria
stand, as does [m4-asks-of-things-out-of-reach.md](m4-asks-of-things-out-of-reach.md).

M4 scores each placement against the owner's tape. Until now the tape was one
point per thing. Under this proposal, a thing with a size is scored by the
placement's distance to the thing itself, zero anywhere on it: a disc for a lamp,
a stretch of wall for a picture or a door, and a box for furniture. The sizes are
recorded in the truth file (`footprint`) and scored by
`experiments/m4/score_attempts.py`'s `distance_to`.

- **The sizes may be estimated.** The owner asked for no more tape for them. They
  come from the map's lines, the owner's earlier readings and the photographs,
  and the truth file says which.
- **The first acceptance block counts.** Its eleven attempts were made on frozen
  targets and labels. This changes how every attempt is scored, development and
  acceptance alike, not which attempts count.
- **The comparison keeps the re-look.** Scored this way, re-looks improve their
  thing less often than chosen viewpoints (3 against 5 in the first block, 1
  against 5 out of reach in development), so the move to a nearest-viewpoint
  baseline proposed after the first block is not needed.
- **Acceptance goes on** to 43 pairs, the count these rates give, from runs
  opened away from their things.

## Why

**A look on a thing is a placement of it.** Scored against one point, every
acceptance attempt that came out worse was a ranged look at a wardrobe, a
footboard or a cabinet. Each landed 0.45 to 0.61 m from the taped centre of the
thing's front, and on the thing itself it was 0.05, 0.12 and 0.25 m off
([the re-scoring](../progress/2026-10-09-m4-scored-from-the-thing.md)). The
rover's 0.20 m claim was honest about the thing. It was scored against a point on
the thing the look had no reason to land on. "Where is the wardrobe" has an answer
anywhere on the wardrobe.

**It does not tune the answer.** The sizes come from the map and photographs, not
from the rover's placements. Scoring changes the same way for both arms. The
development results, mostly paintings, barely move: out of reach the difference
goes from +0.48 to +0.69 and the count from 42 to 41.

## What was considered

- **Widening the rover's claim by the size of what it ranged**, proposed after the
  first block. Rejected: ranged regions were as wide on paintings that landed 0.02
  to 0.18 m from their tape as on the furniture. A claim from region width would
  have turned every honest 0.2 m painting claim into a 0.5 m one.
- **Taping every thing's size.** The owner judged the estimates good enough.
  Their error moves a distance by half of it at most, a few centimetres against
  the half-metre this corrects.
- **Dropping big furniture from the set.** That chooses targets by their result,
  and leaves M4 saying nothing about the things a person most often asks after.

## What would reopen it

- A thing's footprint taped and found further from the estimate than the
  distances it decides.
- Re-looks improving their thing as often as chosen viewpoints over the
  acceptance attempts. The comparison would then move to the nearest reachable
  viewpoint, as the earlier decisions said.

## Requirements

Relies on [R-WS-13](../requirements/world-state.md#r-ws-13), as before. Adds and
retires no requirement.
