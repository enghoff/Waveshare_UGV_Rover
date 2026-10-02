# What to expect from the hardware, and an elevation bias corrected

**Every look reads its elevation 4.9 degrees too high. Correcting that, and letting a
height claim no more than its looks agree on, makes every taped height honest in replay
without hurting how the rover tells things apart.** The claims for bearings, ranges and
one-look placements are still too small. Making them honest loosens matching, because
the same numbers decide both. The change is committed as `59c7e43` but not deployed,
because the rover was offline. This follows [the decision](../decisions/p0-measures-the-hardware.md)
that P0 measures the hardware rather than holding it to bars. The measured table is in
[world_state/README.md](../../world_state/README.md#what-to-expect-from-it).

## How it was measured

Every look attached to one of the six taped targets was compared with the direction,
elevation and distance from the camera to the target's taped position. Targets were
matched to things by eye on contact sheets. Three drives were used: 2026-10-01
(development), and the first drive and redo of 2026-10-02 (held out). The scripts and
results are in `captures/m0-2026-10-02/` (`residuals.py`, `eval_honesty.py`, `residuals.json`).
The rover's own position is part of every residual, and it agreed with the tape to
about 4 to 9 cm at the parking spots.

## The elevation bias

On 2026-10-01, 30 looks at the painting above the cabinet and the toolbox, both in plain
view, read a median 4.9 degrees above the taped direction. That held at two heights and
at distances from 1 to 2.5 m, so it is a constant angle rather than a lens shape. It put
raised things 0.25 to 0.6 m too high. Held out on the 16 such looks of 2026-10-02,
removing 4.9 degrees left a median 1.4 to 1.8 degrees, and none past 4.

It is not the fisheye lens. Redrawing the 2026-09-08 recording through the lens refitted
on 09-30 moved its elevations by a median of only 0.7 degrees. Every look was at the rest
tilt of 20 degrees, so only that tilt is corrected. A look at any other tilt, such as a
hypothesis check's look at level, keeps its elevation as recorded, and its bias is
unmeasured.

The change, in `locate.py`:

- `elevation_of` takes the bias out at the tilt it was measured at.
- The elevation sigma becomes the measured 2.2 degrees, instead of borrowing the
  bearing's 1.5.
- A height's sigma is either its best look's or the spread of its looks, whichever is
  larger. It used to be the best look's alone.

Replayed on the three drives, taped heights fell inside their claims 7 times in 7,
against 1 in 7 before. On the redo drive the height errors went from +0.51, +0.69 and
+0.27 m to +0.18, +0.29 and +0.08 m.

## Identity, checked on the labelled drive

The 76 things of 2026-09-08 that were labelled by eye can only judge a change through
the lens the rover flies now. So the recording was first redrawn with `relens.py` on a
copy. Merges of genuinely different objects were then counted, ignoring the labelled
splits of one object and the six identical chairs:

| Build | Different objects merged |
|---|---:|
| as deployed | 10 |
| with the elevation bias corrected | 10 |
| with the bias, elevation sigma and height spread (committed) | 8 |
| ...and the bearing sigma widened to the measured 2.2 deg | 13 |
| ...or a 0.5 m floor on one-look placements instead | 13 |

Replayed through the lens the labels were made under, the same build appeared to make 19
merges against none. That comparison mixes the lens change in with this one, and is not
the test.

## What stays overconfident

- **Bearings.** Still looks measured 1.2 degrees typical and 3.8 at worst, against a
  claim of 1.5. Moving looks that kept a bearing measured 2.6 typical and 11.6 at worst.
- **Ranges.** Still looks in plain view were off by about a tenth of the distance,
  against a claim of 0.07 to 0.13 m. A range taken through something in front lands on
  that thing.
- **One-look placements.** Their errors were 0.09 to 1.77 m against claims of 0.05 to
  0.24.

Placements from two or more viewpoints in plain view are honest: a median 0.18 m,
11 of 15 inside the claim and all 15 inside twice it. The three overconfident claims
share one cause: the uncertainty the rover states is also the tolerance the resolver
matches with. Widening either claim took merges of different objects from 8 to 13.
Separating the two numbers is the work that would make them honest.

## Requirements

None moved. [R-WS-11](../requirements/world-state.md#r-ws-11) stays `open` until the
change is on the rover and heights are measured there.
[R-WS-10](../requirements/world-state.md#r-ws-10) stays `failing`: the bearing claim is
still smaller than the measured error.
