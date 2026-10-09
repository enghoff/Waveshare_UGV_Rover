# Scored from the thing rather than one taped point, M4's first acceptance block favours the chosen viewpoint

**Re-scored by each placement's distance to the thing itself, the first eleven
acceptance attempts have the chosen viewpoint improving its thing 5 times and
the re-look 3, with one re-look worse and no chosen look worse.** The paired
difference was +0.78, 95% interval -0.08 to +1.70, over six things. The tape was
inside the stated figure 9 times in 11 after both arms. Scored against one point
per thing, [the same attempts](2026-10-09-m4-acceptance-first-block.md) had
three chosen looks worse. Every one of those was a look landing on a wardrobe, a
footboard or a cabinet half a metre from the taped centre of its front, and on
the thing itself it was 0.05, 0.12 and 0.25 m off. The rover's claim was right
about the thing; the tape named a point on it that the look did not land on.
Development evidence and acceptance alike are re-read below. No requirement
moved, and [R-WS-13](../requirements/world-state.md#r-ws-13) stays open.

## How it was scored

`experiments/m4/score_attempts.py` now takes a `footprint` on a truth file's
thing and scores the distance to it, zero anywhere on it (`distance_to`): a disc
for the floor lamp, a stretch of wall for pictures, the door and the panel cover,
and a box for furniture. A thing without one is scored from its point, as
before. Reproduced first in `test_score_attempts.py`: a narrow claim on a
wardrobe's side is inside, not overconfident.

The sizes are estimates, not tape, at the owner's word ("can't you trust your own
estimates by now?"). The black cabinet's width (0.96 m), the footboard's (1.63 m)
and every direction come from the live map's lines. The black cabinet's, the brown
wardrobe's and the kitchen cabinet's depths come from the owner's earlier
readings, and the rest from the photographs. They are recorded in both truth files
under `footprints`. An error of 0.1 to 0.2 m in a size moves a distance by at most
half that, against the half-metre misses in question.

## The three blocks

All on the rover, against the same store copies
(`captures/2026-10-09-m4-acceptance/fp-dev.json`, `fp-acc.json`).

| | Pairs | Chosen improved, worse | Re-look improved, worse | Difference, 95% interval | Inside the claim, chosen / re-look |
|---|---|---|---|---|---|
| Development, in reach | 17 | 3, 0 | 4, 0 | -0.08 (-0.27 to +0.05) | 15 / 15 |
| Development, out of reach | 13 | 5, 0 | 1, 0 | +0.69 (+0.02 to +1.40) | 13 / 13 |
| Acceptance, block 1 | 11 | 5, 0 | 3, 1 | +0.78 (-0.08 to +1.70) | 9 / 9 |

- **The development results barely moved.** Their things are mostly paintings,
  and a painting's centre is where a look on it lands. Out of reach, the count
  needed is 41 against 42 before.
- **The furniture cases.** Brown wardrobe (1416): 0.53 to 0.56 m from the centre
  of its front, 0.02 to 0.05 m from the wardrobe, in both arms. Footboard (1414):
  0.45 m from its centre, 0.12 m from the bed. Kitchen cabinet (1411): 0.61 m
  from the centre of its doors, 0.25 m from the cabinet, against 0.51 before the
  look.
- **What is still wrong is the rover's.** A re-look at the mirrored wardrobe
  (1415) placed it 1.5 m off, and another attempt's record of it (1423) stood 1.5 m
  off before and after. One chosen look and two re-looks were overconfident.
- **The count.** At these rates the comparison needs 43 pairs, in line with the 42
  the development attempts gave.

## What it means

The comparison M4 asks for is answerable on this flat, and the first block points
the way the development attempts did. The claim fix proposed after the first block
is not needed; the scoring was at fault for big things. Whether these sizes become
part of how M4 is judged, and whether the first block counts, are for the owner
([the proposal](../decisions/m4-scores-from-the-thing.md)).
