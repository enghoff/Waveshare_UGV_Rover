# Masking the crop prevents all four picture-into-chairs merges

**Every one of the four wrong attachments the acceptance drive found is
prevented, and the rule that does it costs 7% of the correct ones.** Not by
sampling the range on the mask, which turned out to be untestable on the
recording, but by asking the appearance question twice: once of the crop as it
is and once of the crop with everything but the object blanked out. A look that
only resembles its entity because of what is standing in front of it is the look
whose score collapses when the intruder is removed.

Two other things were measured on the way and both matter. **The deployed mount
was checked against the room rather than against a calibration board for the
first time**, and it is good to 2.6 degrees across and 0.4 degrees vertically.
And **a coverage gate that looked obviously right is refuted**: a third of the
drive's ranges came from a box less than a quarter of which the depth camera
could see, and those ranges are no less accurate than the rest.

## What the collapse test is, and what it catches

The resolver decides identity on appearance: a look's DINOv2 vector against the
middle of what the entity has already shown, admitted at 0.55. The four wrong
looks all sat above that line. Recomputing each look's vector from the crop with
the region model's own mask applied — background set to the same grey the
letterbox pads with — and asking the same question again:

| entity | the picture look | as the rover scored it | with only the object's pixels | drop |
|---|---|---:|---:|---:|
| `object:1` | 34245 | 0.628 | **0.203** | 0.425 |
| `object:3` | 34333 | 0.573 | **0.206** | 0.367 |
| `object:9` | 34235 | 0.561 | **0.308** | 0.253 |
| `object:21` | 34344 | 0.698 | **0.469** | 0.229 |

The gate is 0.55. All four were above it; all four fall below it.

**Swapping the vector outright is not the way to use that.** A mask makes every
comparison harder, not only the wrong ones: over all 394 reviewed attachments,
92% clear the gate against their own entity unmasked and only 74% do masked.
That is 73 attachments lost to catch 4.

The drop is the discriminator instead. Keep the plain vector as the score, and
refuse an attachment whose score falls by too much when the intruder is removed:

| refuse a look whose score drops by | wrong caught | correct lost |
|---|---:|---:|
| 0.15 or more | 4 of 4 | 61 of 360 (17%) |
| **0.20 or more** | **4 of 4** | **25 of 360 (7%)** |
| 0.25 or more | 3 of 4 | 14 of 360 (4%) |
| 0.30 or more | 2 of 4 | 6 of 360 (2%) |
| 0.35 or more | 2 of 4 | 1 of 360 (0%) |

The correct looks drop by a median of 0.061, with a 95th percentile of 0.222 and
a 99th of 0.314. So 0.20 sits at about the 93rd percentile of ordinary looks and
below every one of the four faults.

**A refusal here is a split and not a merge**, which is the direction
[the plan's own rule](../plans/autonomous-curiosity.md) asks for: an unresolved
answer is preferable to a confident false merge. A look that fails the collapse
test does not join that entity on appearance; the geometry that placed it is
untouched.

## Why 0.20 is a candidate and not a result

**Four positives cannot validate a threshold, and this one was chosen after
seeing them.** The plan forbids exactly that — do not tune a threshold against a
test case and then count the case as independent evidence — so 0.20 is a
development candidate owed a held-out acceptance run, on a fresh recording, with
the threshold frozen first. What is established without a held-out set is the
mechanism: the wrong looks collapse and ordinary looks do not, by a margin of
about three to one.

Two of the six faulted entities are also untested by this. `object:5` mixes two
different paintings, a blank white patch and a dark cabinet, and `object:29` an
armchair, a person with a laptop, a bright window and a dark television; their
wrong looks were never individually identified, so the rule has not been put to
them.

## What it costs to run

Measured on the Orin, on a real frame with twelve regions:

| | |
|---|---:|
| decoding twelve masks | 8.2 ms |
| twelve crops through DINOv2, as one batch | 75.5 ms |
| the same twelve one at a time | 114.1 ms |

So a second, masked appearance pass adds about **84 ms to a look whose median
was 550 ms** over the drive's own 89 inferences — 15%, keeping a look inside the
one second the capture cadence allows. The 2 MB prototype tensor the mask needs
is already copied back from the GPU on every look and thrown away.

## The range question could not be asked, and that is a gap in the recording

The [masks entry](2026-09-07-masks-are-already-there.md) said sampling the range
on the masked pixels was testable on the preserved recording. **It is not.** The
recording keeps the camera's pictures and the single distance the depth service
computed for each box; the depth map that distance came from is never saved
anywhere. So no offline re-sampling is possible, and no future recording can
answer a depth question after the fact either. That is worth fixing before the
next acceptance drive: keeping each region's own depth patch, or the frame's
depth map beside its JPEG, is what would make range remedies testable without
driving.

Live in front of the same scene the fault came from — a dining chair in front of
a framed picture — the mask does separate them, and the whole box does read the
chair: the picture's box came back at 1.760 m where the largest rectangle inside
its mask read 3.801 m. But both figures come from a sliver at the very top of the
depth camera's picture, because from where the rover stands the picture spans 27
to 40 degrees of elevation and the depth camera's field ends at about 28. **The
picture is above the depth camera's view**, so this scene cannot settle the range
question either. It needs the pair inside the depth camera's field, which means
about two metres further back, or a foreground/background pair at chair height.

## The mount, checked against the room

The first check of the deployed OAK-to-gimbal transform that does not use the
calibration board. A feature both cameras can see — the pair of shoes on the
floor — read off each picture against a fine grid at the same moment, with the
distance measured by asking the depth camera about its *own* box, which needs no
mount at all:

| | |
|---|---|
| the shoes, in the gimbal's picture | (0.405, 0.439) |
| the shoes, in the depth camera's picture | (0.305, 0.545) |
| where the mount puts them | (0.337, 0.555) |
| out by | 2.56 degrees across, 0.44 degrees vertically |

The distance was 3.140 m on 134 valid pixels. The horizontal 2.6 degrees is
inside the gimbal's own known pointing error — about 1.5 degrees of backlash plus
a gain error — and reading a feature off a grid is itself worth nearly a degree,
so this is agreement rather than a discrepancy. It is not a substitute for the
board measurement; it is a check that the transform means on the room what it
means on the target, and it passed.

It also disposed of a suspicion of mine. The depth camera's picture looks much
lower than the gimbal's at the same tilt, which reads like a sign error on the
mount's 6.256-degree pitch. It is not: it is the two fields of view, 43 degrees
vertically against about 74, and the fisheye's compression at the edges.

## The coverage gate that does not work

`oak.box_for` accepts any projected box that overlaps the depth picture and then
clips it to the frame, so a box 95% of which lies above the top edge still comes
back — as a thin strip along the bottom of the object, where whatever is standing
in front of it lives. On the recording, computed from the stored boxes and poses
alone:

| how much of the box the depth camera could see | boxes | of which ranged |
|---|---:|---:|
| no overlap at all | 249 | 1 |
| under a quarter | 99 | 84 |
| a quarter to a half | 36 | 36 |
| a half to three quarters | 31 | 25 |
| three quarters or more | 97 | 88 |

**36% of the drive's 234 ranges came from a box less than a quarter inside the
picture**, and the look that caused `object:1`'s merge — 34245 — was measured
from **4%** of its box.

That looked like a gate worth having. Against the parallax ground truth it is
not:

| how much of the box was visible | ranges | median error | within 0.5 m |
|---|---:|---:|---:|
| under a quarter | 57 | 0.326 m | 33 (58%) |
| a quarter to a half | 27 | 0.369 m | 19 (70%) |
| a half to three quarters | 20 | 0.358 m | 15 (75%) |
| three quarters or more | 70 | 0.295 m | 48 (69%) |

Refusing everything under a quarter drops 57 of 174 checkable ranges and moves
the share landing within half a metre from 66% to 70%. **A third of the ranges
for four points**, which is not a trade worth making, and the honest reading is
that the strip that survives clipping usually still sits on something at about
the right distance. The finding is kept because it is true and because it says
where *not* to look for the depth remedy.

## Requirements

- [R-WS-13](../requirements/world-state.md#r-ws-13) stays `open`. Its remedy is
  rewritten from "sample the range on the mask" to the collapse test, with 0.20
  named as a development candidate and a held-out run named as what it needs.
- [R-WS-11](../requirements/world-state.md#r-ws-11) stays `open`, unchanged. The
  room check corroborates the adopted transform but measures nothing this
  requirement is blocked on, which is still the gimbal camera's offset from the
  pose SLAM reports.
- No requirement changed state, no code changed, and nothing was deployed.

## Next, in order

1. **Implement the collapse test** in `perceive.py` and `resolve.py`: decode the
   mask for each kept region, embed the masked crop as a second vector, store
   it, and refuse an attachment whose appearance score falls by 0.20 or more.
   Freeze the threshold in the run manifest before collecting acceptance data.
2. **Keep the depth evidence.** Save each region's depth patch, or the depth map
   beside its frame, so a range remedy can be tested offline at all.
3. **A held-out acceptance drive** with the threshold frozen, the same two named
   targets, and something at the frame edges. Review the same way, by eye, on
   the crops.
4. The range question needs its own arrangement: a foreground/background pair
   inside the depth camera's 43-degree vertical field, which the framed picture
   in this room is not.
