# Does letting DINOv2 feed back into the region mask help? — what will count

Written 2026-10-19, before any labelled look was scored. Committed before the
analysis ran, so the commit date is the evidence that the bar was not moved.

## The idea under test

The rover's perception is one way: YOLOE proposes a region and its mask, DINOv2
embeds the crop, and nothing DINOv2 sees changes the mask. The candidate makes
them agree: [settle.py](settle.py) fits object and background in DINOv2's own
patch tokens with the mask as the prior, reassigns each patch, and repeats until
it settles. Its products are a settled embedding of the object and a settled
assignment. If it is worth anything, it should make the appearance vector better
at the one job the resolver gives it (R-WS-13): telling the same object from a
different one, and in particular refusing the strays that get attached to a
thing.

## Evidence

The 25 records of the per-look identity review in the frozen perception
benchmark (`experiments/perception_bench`, snapshot `merge_20261003`): 366 looks
a person called the record's own object and 46 called something else. All 412
frames are on this desktop. The replay ([replay.py](replay.py)) reproduces the
stored summary vector at a median cosine of 0.9999, using the fp16 DINOv2 export
the rover's TensorRT engine is built from.

## What is compared

| | signal | cost on the rover |
|---|---|---|
| B1 | summary token of the crop (`dino_blob`, as stored) | what runs now |
| B2 | summary token of the masked crop (`dino_alone_blob`, as stored) | what runs now |
| C1 | mean of the patch tokens under the mask, no feedback (round 0) | arithmetic only |
| **C2** | **settled embedding, temperature 0.1** | arithmetic only |
| C3 | summary token of the crop masked by the settled assignment (trim only) | a second DINOv2 pass, as B2 is now |

Temperatures 0.05 and 0.2 are reported as a sensitivity check and play no part
in the verdict. No setting in `settle.py` is changed after scoring starts.

## The two tests

**T1, identity between objects.** Looks are grouped into physical objects from
the labels and a check of the photographs made before scoring: 246 and 351 are
one painting, as are 249 and 340, and 383 and 400. Pairs of main looks in the
same group are positives and pairs in different groups negatives, scored by
cosine; the measure is the area under the ROC curve. Left out: the floor record
259, which is not an object; every pair between the three dining-chair records
247, 302 and 350, and within them, because which chair cannot be told; the
owner seated (332) against the owner's head (385); and two pairs of records that
might be one painting with the camera rolled, 343 against 249, 340 and 335, and
387 against 383 and 400.

**T2, strays within a record.** For every look in the 14 records with strays,
its median cosine to the record's main looks (leaving itself out), as
`appearance.appearance` takes the middle of a thing's exemplars. The measure is
the area under the ROC curve for main against stray, pooled.

Intervals are 95% bootstrap intervals from 2,000 resamples of groups (T1) or
records (T2), with every method scored on the same resample.

**Both tests are biased, in opposite directions, and the bar is built for it.**
Every look in the evidence was attached by the resolver, so it had already
passed B1's gate against the thing's exemplars. In T1 that flatters B1, because
it had to see each positive as similar. In T2 it flatters any signal other than
B1, because strays that passed B1 regress to the mean on another signal. So T1
asks only that nothing is lost, T2 asks for a clear gain, and a third check asks
for a gain beyond B1 and B2 together, which conditions on the signals the
selection used.

## The bar

C2 is **worth building** only if all four hold:

1. **No loss of identity:** T1 area under the curve for C2 is at least that of
   the better of B1 and B2, less 0.01.
2. **Strays caught better:** T2 area for C2 exceeds that of the better of B1 and
   B2 by at least 0.03, and the bootstrap interval of the difference against
   each of B1 and B2 lies above zero.
3. **Something the resolver does not already have:** in a logistic regression
   fitted with each record left out in turn, the area of the held-out
   predictions from B1, B2 and C2 together exceeds that from B1 and B2 by at
   least 0.03.
4. **The feedback is what helps:** T2 area for C2 exceeds that for C1 with a
   bootstrap interval above zero, and its T1 area is no more than 0.01 below
   C1's.

If 1 to 3 hold for C1 or C3 but 4 fails, the verdict is that **the feedback loop
is rejected** and pooling the patch tokens under the mask is reported as a
separate lead. Otherwise the idea is **rejected**.

Worth building means a held-out check next (the M5 audit's labels or the record
review), not a rover change.

## Also reported, descriptive

- How far the settled assignment moves from the mask: one minus the soft overlap
  between the two, over picture cells. As a stray flag, the area under the curve
  for main against stray, judged useful at 0.65 or more.
- Top-1 retrieval of each main look's own object by median cosine, as the
  resolver's outclassed test asks it.
- Per-record areas.
- A sheet of settled assignments drawn over their crops, to see whether the loop
  does what it claims: trims an intruder out of a painting's region and keeps
  the painting.
