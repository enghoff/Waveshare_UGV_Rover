# DINOv2 does not feed back into the region mask

Status: closed 2026-10-19, rejected offline before anything was built for the
rover. Nothing runs.

The rover's perception passes one way. YOLOE proposes a region and its mask,
DINOv2 embeds the crop, and nothing DINOv2 sees changes the mask. The owner
asked whether the two could instead settle on one segmentation and one
embedding that both agree with. DINOv2 already gives the means. The rover's engine
produces 256 patch tokens for every crop and the code keeps only the summary
token, so a loop that reconciles the mask with the patch tokens would cost
arithmetic, not another network.

## What was tried

A two-class fit in DINOv2's patch-token space with the mask as the prior, run
until it stops moving: GrabCut's loop with DINOv2 features in place of colour
([settle.py](../../experiments/dino_feedback/settle.py)). It was scored on the
412 looks of the per-look identity review against a bar committed before
scoring ([the measurement](../progress/2026-10-19-dino-feeding-back-into-the-mask.md)).

- **It told strays from a thing's own looks no better than what runs**: 0.934
  against 0.924 for the stored summary vector, an interval of the difference
  running from -0.019 to +0.040. Added to the two stored vectors in a
  held-out regression it moved 0.938 to 0.939.
- **The loop added nothing to averaging the patch tokens under the mask
  without it**: -0.004 (-0.011 to +0.001).
- **Feeding the agreed mask back through DINOv2 was worse**, not better: 0.910
  on strays and 0.942 between objects, against 0.937 and 0.958 for the
  patch tokens averaged under the mask.

The reason is visible in the crops. YOLOE's mask already leaves out what stands
in front of a thing, and that is the case the idea was aimed at: the dining
chairs before the landscape painting are out of its mask before DINOv2 sees
anything. DINOv2's grid of 16 by 16 cells is coarser than the mask it would
correct, so what the loop mostly does is round the mask out by a cell.

## What would reopen it

- **A region source whose masks are worse than YOLOE's**, such as boxes alone or
  a model that does not separate a thing from what stands in front of it. Then a
  correcting step has something to correct. Show the failures first, counted on
  frames not used to build the fix.
- **Patch features finer than the mask**: DINOv2 at a larger input, or a
  learned feature upsampler such as FeatUp (2024), timed on the Orin within the
  7.4 GB it shares with the GPU.
- **A labelled set of regions that hold two things** the mask failed to separate.
  The review used here labels looks by which thing they show, not by what else
  is in the region, so it cannot show a gain on that failure if one exists.

Averaging the patch tokens under the mask, with no loop, is a separate question.
It named a look's own object more often in one descriptive count, but mostly on
one pair of paintings, and passed none of the bar's tests for strays. It would
need its own held-out test.

## Requirements

Supports nothing new.
[R-WS-13](../requirements/world-state.md#r-ws-13) stays open: identity-dependent
actions still need independently validated associations, and this offers no new
evidence for any.
