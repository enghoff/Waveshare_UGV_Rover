# Blind depth recall: the stored depth predicts too little of a nearby view, and the first experiment stops the plan as configured

**Re-projecting depth the rover stored earlier predicted only 5-32% of what
its depth camera measured from 0.7-0.9 m away. Two of six targets reached the
30% bar against the three the protocol required, so its stop rule triggered.**
This was the first experiment of
[perceiving by prediction](../plans/perceiving-by-prediction.md#the-order-of-work),
designed so that it could stop the plan. Re-projection itself works: it beat
copying the old depth map unmoved on five of six targets, by a median of 12
points. What fails is the depth: a single earlier look covers too little of the
new view, and where both looks measured a surface they agree within tolerance
less than half the time.

The leading suspect is the OAK-D-Lite's passive stereo as the rover configures
it. That is a suspect, not a proven cause. As the plan says, nothing builds on
geometry recall (bins, epochs, objects) until a bounded step on the depth
itself has run.

## What was done

Offline, on this desktop, nothing on the rover. The evidence was the 27 looks
of 2026-10-03, 09:13-09:17, that kept both a picture and an OAK depth map, with
their poses from a read-only copy of the world store. All were taken at pan 0
and tilt 20 with the OAK on the gimbal.

The pairs, the labels, the predictor, the scorer and the stop rules were
committed before any target's depth map was opened. The predictions were
committed before scoring. The work is on branch `depth-recall`: protocol
`efd1c2f`, predictions `d384bf4`, scores `0c9c9c8`.

- **Pairs**: every later look within 1 m and 20 degrees of an earlier one and at
  least 60 s after it. That gave 10 pairs, 0.73-0.94 m apart, covering six
  targets, which share their sources.
- **Labels**: from the photographs. A person sits on the window armchair in one
  source and is gone in every target, so that armchair is excluded from four
  targets' scores. No other change was seen.
- **Prediction**: each source's depth becomes points on the map, through the
  OAK's lens, its measured mount, the gimbal's pan and tilt, the camera's taped
  position and the stored pose.
  - Only patches that are one surface are kept, from 0.3 to 6 m, and depth edges
    are dropped.
  - The points are drawn into the target camera, nearest surface first.
  - The target's own depth was never read.
- **Scoring**: over the target's *measured surface*, meaning pixels whose depth
  agrees with its neighbours (the rule the sources pass), outside the labels.
  - *Useful* means predicted within the larger of 15 cm and 5% of depth.
  - The copy baseline is the nearest source's depth map, unmoved.
  - The surface rule was chosen on seven development pairs among looks that
    are not targets, before the freeze.
  - "Beats copying" was made precise as better on at least four targets with a
    positive median difference; the plan said only "beats".

## What came out

| Target | Measured area | Useful | Unavailable | Wrong | Median error | Copy useful |
|---|---|---|---|---|---|---|
| 091624 | 57% | 5% | 79% | 16% | 0.36 m | 11% |
| 091626 | 53% | 17% | 55% | 27% | 0.27 m | 15% |
| 091637 | 59% | 15% | 61% | 24% | 0.26 m | 14% |
| 091652 | 54% | 24% | 41% | 35% | 0.24 m | 3% |
| 091657 | 45% | 32% | 32% | 37% | 0.21 m | 4% |
| 091705 | 68% | 31% | 35% | 33% | 0.18 m | 4% |

Scored on all valid depth without the surface rule, none of the six reaches 30%.

## Why, as far as it can be told

These checks came after scoring, and they decide nothing.

- **Not the pose.** A heading and tilt search that used the target's own depth
  still left four of six targets under 30%. The best target gained 15 points
  with a 3-degree turn and a 3-degree tilt; the others gained 6 points or less.
  - On the development pairs, heading changed little on four pairs and was
    worth 6-7.5 degrees on the other three. One of those looks had a
    10.5-degree correction in its stored pose.
  - A common tilt error of up to 8 degrees on both looks, or the camera 10 cm
    off, moved accuracy by a few points.
- **Not systematic.** Wrong predictions split about evenly between too near
  and too far.
- **Coverage.** A single earlier look 0.8 m away had a usable surface for only
  21-68% of what the target measured. Some of the rest the source could not
  have seen at all.
- **Agreement.** Where both looks measured a surface, they agreed within
  tolerance on 24-49% of it, falling at 3-4 m on some targets. From one spot,
  turning in place, the median disagreement was 7-28 cm at 2-3 m.
- **The stereo setting.** The depth service runs the stereo with depthai's
  `HIGH_DENSITY` preset, which keeps low-confidence matches to fill more of the
  picture. Left-right checking, subpixel and a 5x5 median are already on, and
  the depth is warped into the colour camera's view
  (`oak_depth/depth_server.py`).

Codex reviewed it read-only. It regenerated the six predictions to their
committed hashes, reproduced every score, and checked the conventions: depth
along the optical axis, the lens scaling, the mount and gimbal rotations, the
pose's origin and depth within 29 ms of the shutter. It found no bug that would
turn the result into a pass. It also found that denser rendering, or a looser
surface rule on the sources, still gives two of six.

## Limits

- Six correlated targets from one four-minute recording, sharing their sources,
  with baselines of 0.73-0.94 m.
- Three of the six targets and three of the five sources were taken while
  moving.
- It says nothing about recall from 25 cm, or about memory fused from many
  looks.

## What next

Codex's order, adopted in the plan:

1. Measure how repeatable and accurate the depth is from a standstill, against
   surveyed flat surfaces.
2. Try a bounded set of stereo settings: today's and at most three others,
   such as confidence threshold, decimation and output size, within the pinned
   depthai version.
3. Try one way of fusing several looks before predicting.
4. Re-run blind recall on fresh, held-out targets with the same 3-of-6 and
   copying gates. For later change detection, add a floor of 80% accuracy where
   predicted.

If nothing passes within that budget, geometry recall with this sensor stops.
The plan would then need another depth source, or prediction built on the
lidar map and appearance instead.
