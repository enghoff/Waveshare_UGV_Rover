# Both remedies are on the rover, and the depth camera's blind edge is now visible

Three changes deployed, at `16621ea` and `ed2a7eb`, and proved on the Orin.
None is an acceptance result — the recording that will judge them has not been
driven yet — but each does on the rover what it did on the bench.

## The rover asks the appearance question twice

Every look now embeds each region a second time from the same crop with
everything but the region's own pixels blanked out, and the resolver refuses a
match whose score falls by 0.20 or more between the two. The measurement behind
that number is in [the collapse test](2026-09-07-masking-the-crop.md); what is
new here is that it runs.

Proved on the rover with a look through the deployed daemon, reading back the
rows it wrote:

| row | region score | mask covers | masked against plain |
|---|---:|---:|---:|
| 34782 | 0.34 | 40% | 0.616 |
| 34783 | 0.70 | 64% | 0.393 |
| 34784 | 0.20 | 47% | 0.478 |

All three carried a second vector. **Both ends of that column would be a bug**:
a masked vector identical to the plain one would mean nothing was masked out, and
one unrelated to it would mean the mask had removed the thing itself. Between a
third and two thirds of each crop survived, which is the range the recording
predicted.

It costs 24 ms on that look, reported beside the others as `dino_alone_ms`
against `dino_ms` of 30 — the second pass is cheaper than the first because it
reuses crops already in hand. The whole look's model time was 102 ms.

## A missing distance now says which silence it is

"No range" was three situations and nothing could tell them apart:

- **outside the depth camera's view** — the region was never going to be
  measured, because that camera sees about 70 degrees across where the gimbal
  sees 99;
- **nothing in the box could be measured** — the camera looked and the surface
  gave no disparity;
- **the depth camera did not answer** — switched off, or restarting.

The reason is stored on the observation, the per-look line counts the first kind,
and `WorldStore.ranging` rolls it up per thing so that *this has never had its
distance measured* and *no look from anywhere the rover has stood could ever have
measured it* are separate questions with separate answers. The entity list and
the entity detail both carry it.

Proved live by turning the gimbal while the depth camera stayed pointed straight
ahead:

| gimbal pan | what the look said |
|---:|---|
| 0 | 4 of 4 ranged |
| 12 | **3 of 4 ranged by the depth camera, 1 outside its view** |

The region that failed at pan 12 sat at 0.95 of the frame width — the far right
edge — and its row reads `outside the depth camera's view`. That is the geometry
the acceptance drive measured, now reported at the moment it happens instead of
being recoverable only by arithmetic afterwards.

## The depth behind a look is kept now, and it reads back true

Deployed at `ed2a7eb`. The depth service gained `GET /depth.raw`, which hands
back the map as the device made it -- one unsigned 16-bit millimetre per pixel,
with the shape, the units and the frame's age on the headers -- and a look saves
it gzipped beside its frame, description first so the file can be read in a year
without the code that wrote it.

**The claim is that a distance can be recomputed later, so that is what was
checked** rather than merely that a file appeared. Running the depth service's
own arithmetic over the saved buffer -- its percentile, its band, its off-axis
correction, its constants -- against what the rover actually recorded:

| observation | recorded | recomputed from the saved map | apart |
|---|---:|---:|---:|
| 34827 | 1.808 m | 1.815 m | 0.007 |
| 34826 | 1.723 m | 1.727 m | 0.004 |
| 34825 | 1.750 m | 1.755 m | 0.005 |
| 34824 | 2.957 m | 2.982 m | 0.025 |

The residual is the map being one frame newer than the one the service sampled,
which is 67 ms at 15 fps. A first attempt that skipped the off-axis correction
disagreed by 20% and would have been the wrong thing to report as agreement.

It costs 39 kB gzipped for a 320x180 map -- the depth stream runs at half the
colour camera's width -- against a 38 kB frame, and only where a range was
actually measured. A parked rover with the camera off keeps nothing.

## Nothing is refused for being outside the view, and that is deliberate

Refusing a range drawn from a box the depth camera can barely see looked
obviously right, and the recording refutes it. 36% of the drive's 234 ranges came
from a box less than a quarter inside the depth picture; against the parallax
ground truth those land within half a metre **58% of the time, against 69% for
the best-covered**. Refusing them drops a third of the ranges and moves the
overall figure from 66% to 70%.

So the strip that survives clipping is usually still on something at about the
right distance, and this reports and counts rather than gating. The full table is
in [the collapse test](2026-09-07-masking-the-crop.md).

## Also

A check in the inspection suite compared two timestamps taken inside one 16 ms
tick of the workstation's clock and failed about one run in three. Fixed, and the
suite now runs to the same number four times running.

## Requirements

- [R-WS-13](../requirements/world-state.md#r-ws-13) stays `open`. Its remedy is
  deployed and its threshold frozen; what it owes is a held-out recording.
- [R-WS-12](../requirements/world-state.md#r-ws-12) is untouched.
- M0 criterion 2's "unsupported patches are refused and counted separately" is
  now half met: they are counted and named per look and per thing. Criteria 3 and
  8 have a deployed remedy and no fresh evidence.
- M0 criterion 2 also asked for the evidence a later question could be put to,
  which the acceptance recording did not keep. It is kept now.
- Counts: world_state 798 passed 0 failed, rover_daemon 856, drive_web 582.

## Next

1. ~~Make a bare wall or window patch ineligible as somewhere worth driving
   to.~~ Withdrawn: the fault does not reproduce on this recording and the two
   entities it was to be written against turn out to be a ceiling light and a
   doorway. See [the re-review](2026-09-07-no-bare-patches.md).
2. Count splits, which nothing reports the way a merge is reported.
3. Widen the recorded position uncertainty to what the drive measured: the
   sideways miss is flat at about 15 cm from half a metre out to five, while the
   rover records its own position as good to 4 cm.
4. The navigation-restart demonstration, then the held-out drive.
