# The masks the rover already computes and throws away

The region model is a segmentation model and its masks separate a chair from the
picture on the wall behind it. **They are computed on every look, copied back to
host memory, and discarded.** Decoding them costs 6 ms against a 550 ms look, so
the fix for the identity fault the
[acceptance drive](2026-09-07-m0-acceptance-drive.md) found is not a new
measurement or a new model — it is reading an output that is already there.

The idea is the owner's, offered as a question: could this be solved by getting
and using the object's mask. It can, and the evidence is below.

## What the engine already returns

The built TensorRT engine on the rover, `yoloe.plan`:

```
INPUT  images  (1, 3, 512, 512)
output regions (1, 37, 5376)
output output1 (1, 32, 128, 128)
```

`regions` is one row per anchor: four box numbers, a score, and **32 mask
coefficients**. `output1` is the **32 mask prototypes**, at a quarter of the
input resolution. A mask is the coefficients weighted against the prototypes,
through a sigmoid — the standard YOLO segmentation head.

`world_state/engines.py` copies *both* outputs back from the GPU on every
forward pass, because it copies everything the engine declares. Then
`perceive.py` keeps one of them:

```python
return self._regions.run({"images": blob})["regions"]
```

The 2 MB prototype tensor arrives in host memory and is dropped. The code says
why, and it was a reasonable call at the time: *"The masks are not decoded: the
box is all that a bearing and a crop need, and the prototypes are the expensive
half."* The first half of that is what the acceptance drive disproved — the box
is **not** all a range needs.

## It cuts the chair out

Three looks from the drive, each one a crop that is plainly a framed picture,
each attached by the resolver to an entity that is otherwise dining chairs. For
each, the anchor that produced the stored box was recovered by overlap and its
mask decoded:

| look | entity | anchor recovered at | score | mask covers |
|---|---|---:|---:|---:|
| 34245 | `object:1` | IoU 1.00 | 0.79 | 66% of the box |
| 34333 | `object:3` | IoU 1.00 | 0.25 | 54% of the box |
| 34235 | `object:9` | IoU 1.00 | 0.81 | 74% of the box |

The anchor is recovered exactly, so this is repeatable rather than a search that
happened to work. And the mask claims only half to three quarters of the box —
**the part it excludes is the chair**, checked by eye on all three:

- 34235: a chair back intrudes from below and the mask has a notch in exactly
  that shape.
- 34245: a chair crosses the picture diagonally; the mask follows the diagonal.
- 34333: two chair backs stand in front of the picture and the mask comes out as
  a clean "T" — the picture above them, and the strip of picture visible
  between them — with both chairs cut out as rectangles.

## What it costs

Decoding twelve masks — a busy look — takes **6.1 ms** in numpy on the Orin's
CPU, against a median look of 550 ms and a 95th percentile of 828 ms measured
over the drive's own 89 inferences. So it is about 1% of a look, and the network
compute and the 2 MB device-to-host copy are already being paid.

Doing it on the GPU instead would be cheaper still, and is not needed.

## Why this is the right lever rather than one of several

The acceptance drive left two faults and both trace to one cause: a box
containing two objects at different depths. It merges the chair with the picture
behind it, and it corrupts the range that would otherwise separate them, because
the depth patch is sampled over the whole box.

A mask addresses the cause rather than either symptom, in three places at once:

1. **The range** is sampled only on the object's own pixels, so the picture's
   range is the picture's.
2. **The appearance vectors** are computed from a crop that currently contains
   both objects. `object:1`'s painting crop and its chair crops score as similar
   partly because they share pixels.
3. **The extent** — how wide a thing is — is currently the box, which for a
   chair in front of a picture is both of them.

The alternative that was written down before this, splitting a region by its own
depth histogram, guesses which depths belong to the object from the depth data
itself. A mask says which *pixels* do, from the image, independently of the
depth camera — which matters because the depth camera is the thing being checked.

## What has not been shown

- **That fixing the range fixes the merge.** The drive showed range cannot
  separate these cases *as measured through the whole box*. Whether a
  mask-sampled range separates them is the next thing to test, and it can be
  tested on the preserved recording without driving.
- **That the masks are good on things other than pictures and chairs.** Three
  cases, all the same kind. A wider check over the recording is cheap.
- **Anything about cost inside the sidecar.** 6 ms was measured standalone; the
  sidecar's own overhead per look is not measured here.
- No code has been changed. `perceive.py` still discards `output1`.

## Requirements

- [R-WS-13](../requirements/world-state.md#r-ws-13) stays `open`. Its remedy is
  rewritten to point at the masks rather than at a depth histogram.
- [R-WS-12](../requirements/world-state.md#r-ws-12) stays `open` and may benefit:
  a mask over a bare floor patch is a large featureless region, which is a
  cheaper thing to recognise than the patch itself.
- No requirement changed state.

## Next, in order

1. Sample the range on the masked pixels only, on the preserved recording, and
   see whether the wrong attachments separate.
2. Compute the appearance vectors from the masked crop and see what it does to
   the similarity between a picture and the chair in front of it.
3. Then, and only then, change the running rover.
