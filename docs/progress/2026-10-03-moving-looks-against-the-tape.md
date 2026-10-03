# Against the tape: still looks claim about what they deliver, moving looks claim half of it

**On a drive past six taped paintings and the cabinet, the 14 still looks pointed a
median 1.0 degree off the tape, against the 1.5 they claim. The 112 moving looks were
3.2 degrees off, and still claimed 1.5: only 69% fell inside twice their claim.** The
heading check on the move helps, but not enough to make up the difference: checked
looks were 2.8 degrees off and unchecked ones 3.8.
[R-WS-10](../requirements/world-state.md#r-ws-10) stays `failing`, now on moving looks.

This is the measurement
[the heading check on the move](2026-10-02-depth-matched-and-heading-checked-on-the-move.md)
was waiting for. The drive ran on the deployed `2d6d912` (world_state, rover_daemon,
ros_nav) and oak_depth `86222ef`. The manifest, the targets, the scripts and the
recording are in `captures/2026-10-03-targets/`.

## How it was measured

The temporary objects of the last two days were gone, so the targets were chosen from
the rover's own looks of that morning, not from a new drive. They are the cabinet, the
painting above it, the two other paintings on the same wall, the landscape painting on
the end wall and a painting at the hallway mouth. The owner taped each one. The tape
frame came from the room's two walls fitted in the rover's map. It was checked by
taping the rover where it was parked, before the drive and after it: the first agreed,
and the second was 0.05 m out on one axis.

Claude drove seven stops. At each stop the rover turned to face each target, measured
its facing by a scan and stood still for 6 s. Two of the legs drove straight at
targets, so that moving looks would see them. Every leg arrived, every facing was
measured, and the position stayed trusted throughout. Targets were matched to the
rover's things by eye on contact sheets, never by position.

## Directions

| | Looks | Median error | 90th percentile | Inside 1 claim | Inside 2 claims |
|---|---:|---:|---:|---:|---:|
| still | 14 | 1.0° | 3.9° | 71% | 79% |
| moving | 112 | 3.2° | 6.9° | 33% | 69% |
| moving, heading checked on the move | 84 | 2.8° | 6.9° | | 64% |
| moving, not checked | 28 | 3.8° | 6.7° | | 82% |

Both looks' claims were mostly the 1.5 degree floor. Two things drive the moving-look
error, and neither on its own explains it:

- **Distance from the middle of the picture.** Looks within 10 degrees of the middle
  were 1.6 degrees off. Further out they were 3 to 4. Still looks face their target,
  so they are nearly all in the middle.
- **Motion itself.** Moving looks within 15 degrees of the middle, with the box clear
  of the edge, were still 2.9 degrees off.

Within one picture, the errors at different targets differ by a median 4.2 degrees.
So most of the moving-look error is not the whole picture's heading being wrong.

## Ranges and placements

Every range that came back, still or moving, was within 0.5 m of the tape: 7 of 7
still looks and 18 of 18 moving ones. The paintings were within 0.03 to 0.27 m. The
cabinet read 0.2 to 0.4 m long every time, against claims of 0.04 to 0.10 m, probably
because the depth reaches into its open shelves. Of 62 moving looks in range of a
target, 21 had the target outside the depth picture and 20 were dropped for turning.

| Target | Placed off the tape | Claimed | Height off |
|---|---:|---:|---:|
| cabinet | 0.04 m | 0.23 | |
| painting above it | 0.38 m, behind the wall | 0.35 | +0.19 |
| dining-side painting | 0.44 and 0.50 m (two things) | 0.30, 0.38 | -0.28, +0.14 |
| window-side painting | 0.21 and 0.52 m (two things) | 1.29, 0.54 | +0.09, +0.07 |
| hallway painting | 0.48 m | 0.25 | +0.52 |
| landscape painting | 0.55 m | 0.30 | -0.50 |

Four of the six principal placements are outside their claim, and two of the paintings
were split into two things each.

## What it means

A still look is worth about what it says. A moving look is worth about twice what it
says, and it is most of what the rover records: here, eight looks in nine. Widening
the claim to the measured 3 degrees is what honesty asks for, but the claim is also
the tolerance the resolver matches with, and widening it merged different objects on
[2026-10-02](2026-10-02-what-to-expect-from-the-hardware.md). The next step is to
separate a look's stated bearing uncertainty from the matching tolerance, as
`4342c37` did for placements, and to state moving looks at what they measured here.

## Requirements

None moved. [R-WS-10](../requirements/world-state.md#r-ws-10) stays `failing`: moving
looks claim 1.5 degrees and measure 3.2.
