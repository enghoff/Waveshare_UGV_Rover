# Ranging a region from its own outline: better readings, and a modest gain in placements

**Reading a region's distance from the depth under its own outline is better than reading it
from its box.** At the taped objects, it puts 41 readings within 25 cm of the tape where the
box puts 36, and the median error falls from 0.27 to 0.19 m. The box falls back in where the
outline cannot measure. Replayed through the resolver, the worst placement among the taped
objects falls from 0.87 to 0.38 m, and the average from 0.22 to 0.18 m. No things are lost.
It is not on the rover yet: the region finder's outlines would have to reach the daemon,
which ranges each region today.

[R-WS-10](../requirements/world-state.md#r-ws-10) stays `failing`. Scripts and results are
in `captures/2026-10-02-single-look-rules/`: `outline_ranges.py`, `score_ranges.py` and
`outline_replay.py`.

## How it was replayed

The rover keeps the depth map beside each picture whenever any region in it was ranged. That
is 65 looks on 2026-10-01, 14 on each 2026-10-02 drive, and 110 from today's run. For each
look, the region finder was run again on the stored picture under ONNX Runtime, and every
stored region matched its regenerated outline, all but 3 of 1,395 at an overlap of 0.5 or
more. The outline is cut to its own box, because the finder's outline is not: the red
toolbox's outline also covered every other red thing in the picture. Each outline pixel is
carried into the depth map through `oak`'s own mount and lens. The box method was recomputed
from the same stored maps as a control, and it matches the rover's own reading within 2 cm
for 88 to 91% of regions.

Two statistics were tried under the outline: the median, and the depth server's own nearest
surface (the 20th percentile, then the median of the band behind it). The nearest surface won.
Where the outline leaves too few depth pixels, the box's reading is kept. That happens for a
small or edge-of-view region, 6 to 13% of those the box ranges. Without that fallback, the
toolbox lost its distances on 2026-10-01 and was placed from crossings alone, 0.61 m out.

## Against the tape

There are 124 looks at the six taped objects with a depth map kept, scored as
`captures/m0-2026-10-02/residuals.py` scores them:

| Reading | Ranged | Within 0.25 m | Within 0.5 m | Off 1 m or more | Median error |
|---|---|---|---|---|---|
| Box (deployed) | 73 | 36 | 54 | 6 | 0.27 m |
| Outline, box where it cannot measure | 73 | 41 | 59 | 4 | 0.19 m |
| Outline trimmed by 3 px, box where it cannot | 73 | 42 | 60 | 5 | 0.17 m |

Where the two disagree by over half a metre, the outline was nearer the tape at the paintings
behind something: 2.45 m against the box's 1.22 for a painting taped at 2.54. It was further
from the tape at the cabinet and the toolbox on 2026-10-01, by 0.1 to 0.2 m.

Trimming the outline's edges looks slightly better here and fails badly elsewhere. Today's
glass table read 4.0 m trimmed, 1.5 m untrimmed and 0.97 m from the box, because its only
solid pixels are its frame, which the trim removes. The untrimmed outline is the one to use.

## Replayed into placements

The 13 placements of taped objects across the three drives, and every thing placed:

| Ranges | Average error | Median | Worst | Within 0.35 m | Things (10-01, 10-02 first, redo, today) |
|---|---|---|---|---|---|
| Box (deployed) | 0.22 m | 0.17 m | 0.87 m | 12 of 13 | 73, 19, 27, 46 |
| Outline, box fallback | 0.18 m | 0.15 m | 0.38 m | 11 of 13 | 77, 19, 26, 46 |

The landscape painting behind the dining chairs goes from 0.87 to 0.36 m. The painting above
the cabinet goes from 0.29 to 0.38 m and from 0.25 to 0.31 m, although its readings improved
(2.09 m against a true 2.20, where the box read 2.00). That shift comes from the fit, not the
reading.

## What it gets wrong

- **The left edge of the depth map.** Today's bedroom wardrobe stands in the strip at the
  depth map's left edge, where the stereo reads it at about 3.2 m. The outline reports that,
  and the wardrobe becomes one thing placed 0.34 m behind its wall. The box got 1.81 m, which
  fits the wall, but only because its nearest surface was the corner of the bed inside the
  box. Both readings come from a strip of the depth map that should not be trusted.
- **Transparent things.** A glass table reads through to what is behind it except at its
  frame. The untrimmed outline keeps the frame, and still reads half a metre beyond the box.
- **Mirrors**, as before: the depth is the reflection's, whichever pixels it is taken from.

## What putting it on the rover needs

The perception sidecar computes each region's outline and returns only its box. The daemon
then asks the depth server to range the box. So the sidecar must return the outline, cut to
its box, in a compact form. The daemon must range it against the depth map it already fetches
whenever a look keeps depth, and keep the box as the fallback. The outline should be stored
with the observation, so that this can be replayed without running the region finder again
([R-WS-1](../requirements/world-state.md#r-ws-1)).

## Requirements

None moved. [R-WS-10](../requirements/world-state.md#r-ws-10) stays `failing`.
