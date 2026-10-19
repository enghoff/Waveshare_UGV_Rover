# Letting DINOv2 feed back into the region mask changes nothing worth having

**A loop that makes the region model's mask and DINOv2's patch features agree
before a look is embedded tells the same object from a different one no better
than the vectors the rover already stores, and no better than the same patch
features averaged under the mask with no loop at all.** On the 412 looks of the
per-look identity review, the settled embedding separated a record's own looks
from its strays at 0.934 against 0.924 for the stored summary vector (95%
interval of the difference -0.019 to +0.040), and the loop itself was worth
-0.004 against no loop (-0.011 to +0.001). The test was set before any look was
scored, and by that bar the idea is rejected. Nothing ran on the rover.
[R-WS-13](../requirements/world-state.md#r-ws-13) is unchanged.

## What was tried

The owner asked on 2026-10-19 whether DINOv2 could feed back into YOLOE, so that
the two agree on the most likely segmentation and embedding instead of passing
one way. The rover's DINOv2 engine already produces 256 patch tokens per crop
besides the summary token it keeps: the engine is built from the full export
(`world_state/install_perception.sh`), which was read but not checked on the rover. The
candidate, `experiments/dino_feedback/settle.py`, fits object and background in
those patch tokens with the mask as the prior, reassigns every patch by which
it resembles more, and repeats until nothing moves: GrabCut's loop, with DINOv2
features where GrabCut had colour. It yields an embedding of the object under
the agreed assignment, which costs arithmetic only, and an agreed mask that
can be fed back through DINOv2 for a second summary vector, which costs a
second pass as `dino_alone` does now.

The bar, the grouping of records into physical objects and every setting of
the loop were committed before scoring, in
`experiments/dino_feedback/PREREGISTRATION.md` (commit ad8a180). One look,
61656, was left out under the benchmark's own rule for it.

## How it was measured

The 25 records of the per-look identity review in
`experiments/perception_bench`, snapshot `merge_20261003`: 366 looks a person
called the record's own object and 46 called something else. All 260 frames
are on this desktop. `replay.py` re-runs YOLOE and DINOv2 on the CPU from the
exports the rover's engines are built from. It reproduces the stored summary
vector at a median cosine of 0.9999, the stored masked vector at 0.9987, and the
mask's share of the crop to 0.000 in the median. Two tests:

- **Between objects:** every pair of the review's own looks, same physical
  object or not, scored by cosine. That gave 4,096 same pairs and 55,148
  different ones over 21 objects, with the dining chairs and the uncertain
  duplicates left out.
- **Strays within a record:** each look's median cosine to the record's own
  looks, as the resolver takes the middle of a thing's exemplars. That gave
  203 own looks and 46 strays in 14 records.

Both are biased, in opposite directions, because every look here had already
been attached by the resolver on the stored vector. The bar asked for no loss
on the first test, a clear gain on the second, and a gain beyond the stored
vectors together in a regression with each record held out in turn.

## What it found

| | between objects | strays within a record |
|---|---:|---:|
| stored summary vector (what runs) | 0.954 | 0.924 |
| stored masked vector (what runs) | 0.944 | 0.905 |
| patch tokens averaged under the mask, no loop | 0.958 | 0.937 |
| **settled embedding (the loop)** | **0.957** | **0.934** |
| summary vector of the crop under the settled mask | 0.942 | 0.910 |

Areas under the ROC curve. None of the gains on strays clears zero over 2,000
resamples of records. The stored two vectors together give 0.938 held out;
adding the settled embedding gives 0.939. Halving or doubling the loop's
temperature moved no figure by more than 0.002.

**The loop has little to resolve.** It moved the median own look's assignment
by a tenth from the mask, and drawn over the crops what it mostly does is round
the mask out by one 14-pixel cell. It fills the gaps between a chair's slats and
the space round its legs, and adds the wall next to a painting's frame. The case
the idea was aimed at is already handled before DINOv2 sees anything. In front
of the landscape painting the region model's mask leaves the dining chairs out,
and the loop keeps them out. Under the floor lamp it does trim the scraps of
armchair the mask had taken, and that is the only intruder it was seen to remove.
The 16-by-16 patch grid is coarser than the mask it is meant to correct. How far the loop moved a
look's assignment flagged strays at 0.614, under the 0.65 set as useful.

One number outside the bar points elsewhere. Asked which object a look
belongs to, by its median cosine to each object's looks, the patch tokens
averaged under the mask named the right one for 343 of 353 looks against 327
for the stored summary vector. That is descriptive and unconfirmed: the same
signal's gain between objects was +0.004 (-0.020 to +0.023), and 14 of the 20
looks it got right where the summary vector did not are one confusion, the cow
painting against the grey painting in the gold frame, in both directions. It
rests on one pair of pictures, and it would need a test of its own on looks not
used here before it says anything.

## Requirements

- [R-WS-13](../requirements/world-state.md#r-ws-13) held: open. No new
  identity evidence came out of this.
