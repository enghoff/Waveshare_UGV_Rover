# Monocular ranges file more pictures but fold no duplicates, and they make wrong merges

**Depth Anything V2 Small, scaled in each picture to that picture's own depth
readings, puts things a median 0.36 to 0.48 m from where they are, about as
well as the depth camera's box readings. Given to version 4 for the pictures
that had neither a depth nor a wall place, it files more of them (63% against
56%) but leaves as many duplicates as before. Every variant made one or two
wrong merges, where version 4 and its five noise runs make none.** The world
model is pass/fail on honesty, so this fails as configured. Nothing deployed,
offline on this desktop. [R-WS-17](../requirements/world-state.md#r-ws-17)
and [R-WS-18](../requirements/world-state.md#r-ws-18) stay proposed.

## Why it was tried

72% of the gimbal pictures without a range are outside the depth camera's
view. It covers the middle 65 by 40 degrees of a picture about 160 degrees
across. 12% were taken while turning and 11% found the depth camera not
answering. All of these pictures postdate the depth camera's move onto the
gimbal on 2026-09-30.

A model run on the gimbal picture itself covers the whole frame. But version 4
already places a raised thing with no range where its sight line meets a wall.
Counted that way, 36% of posed regions are placed by bearing alone and 11% only
by a lidar floor. Those are filed 56% and 53% of the time, against 83% for
placed ones. Only 9 of the 70 duplicates of the never-moved objects
([the same day's count](2026-11-12-depth-would-not-fold-occluded-duplicates.md))
began from two pictures with neither kind of place.

## What was done

- **Readings.** Depth Anything V2 Small (onnx-community's export, Apache 2.0)
  was run on CPU over the 3,494 pictures on this desktop that hold a region of
  map session 67 with no depth or wall place, or a depth anchor. That is half of
  all pictures; three would not open. Each region gets the model's relative
  disparity under its own outline (`mono_run.py`).
- **Metres.** Per picture, inverse range was fitted as an affine function of
  disparity on the regions the depth camera ranged there. With a second anchor
  set, it also used the raised regions placed on a wall 2 m or more away
  (`mono_scale.py`). Inverse depth along the camera's axis was also tried and
  fails off-axis, so the fisheye model's output behaves like range along the
  ray.
- **Accuracy**, on readings the fit did not use:

  | Check | n | Median error | Within 0.3 m | Within 20% |
  |---|---|---|---|---|
  | depth camera's ranges, leave one out | 6,134 | 0.36 m | 44% | 62% |
  | wall places, from depth-camera anchors | 2,358 | 0.48 m | 34% | 58% |
  | ... of them 50-70 degrees off axis | 1,198 | 0.50 m | 32% | 56% |

  Version 4 gives a depth box a 0.5 m spread, so a monocular range went in as
  a box (`mono_apply.py`). A reading more than half the anchors' spread beyond
  them was refused.
- **Filing.** Frozen version 4 (`cabe003`) re-filed every picture to
  2026-10-20 three ways:
  - depth-camera anchors (2,926 regions ranged);
  - depth-camera and wall anchors (4,659);
  - strict (1,253): three or more anchors, the picture's leave-one-out spread
    at most 0.4 m, little extrapolation, and never nearer than the lidar
    allows.

## What came out

"Duplicates" counts the standing records within 0.8 m and 0.5 m of height of
a never-moved object's main record, or holding a labelled picture of it
(`mono_compare.py`). "Records holding" is `score_all.py`'s sum, over the 11
objects and every role, of records holding two or more labelled pictures of
one object.

| Filing | Records | Formerly unplaced filed | Duplicates | Records holding | Wrong merges |
|---|---|---|---|---|---|
| version 4 | 658 | 55.6% | 70 | 99 | 0 |
| five noise runs (1% left out) | | | | 98-103 | 0 in all five |
| depth-camera anchors | 696 | 60.2% | 67 | 104 | 2 |
| depth-camera and wall anchors | 708 | 63.0% | 70 | 107 | 1 |
| strict | 658 | 57.5% | 72 | 103 | 1 |

Every variant made the same wrong merge: the cabinet's record took a picture
of the snowy-tree painting that hangs above it. The anchors-only variant also
filed eleven pictures of the black bean bag in session 8 to an armchair's
record. The records involved held 10 to 14% monocularly ranged pictures.
Moves followed changed both ways, within what one session's armchairs can
move.

## What it means

The duplicates do not come from pictures that lack a place: 61 of the 70
began from pictures with a depth or wall place. A place half a metre good
added to the pictures that had none makes more candidates, not fewer records.
For deciding identity, monocular depth at this accuracy adds wrong merges.

Not tested:

- the other half of the pictures, which are on the rover;
- a larger model (Base and Large are CC BY-NC);
- retuning version 4's bars to a monocular spread. Every session has been
  seen, so that would be development, not a test.

## Reproduce

`experiments/depth_for_duplicates/`, with `VISIT_MODEL` set as in the
[occlusion entry](2026-11-12-depth-would-not-fold-occluded-duplicates.md):

    python mono_run.py MODEL.onnx FRAMES.txt FRAME_PATHS.txt STORE.db MONO.npz
    python mono_scale.py LOOKS.npz MONO.npz SCALED.npz [oak+wall] [--z]
    [STRICT=1] python mono_apply.py LOOKS.npz SCALED.npz LOOKS-MONO.npz [MAX_EXT]
    python occl_run.py LOOKS-MONO.npz RUN.pkl
    python mono_compare.py LOOKS.npz base=BASE.pkl mono=RUN.pkl
