# Blind depth recall: the frozen protocol

The first experiment of [perceiving by prediction](../../docs/plans/perceiving-by-prediction.md#the-order-of-work):
can the rover predict what its depth camera will measure from a viewpoint by
re-projecting depth it stored earlier from nearby? It was designed so that it
could stop the plan. Everything below was fixed, and committed, before any
target's depth map was opened.

## Evidence

The 27 RGB-D looks of 2026-10-03, 09:13-09:17, held on this desktop under
`captures/2026-10-03-targets/frames-2026-10-03-acceptance/`. Each has a 640x480
fisheye picture and an OAK depth map of 320x180 in millimetres along the
optical axis, with the device's corrected lens in its header. Poses, pan, tilt
and moments come from the read-only copy of the world store
(`tmp/codex-m5-audit/world.db`): every look is at pan 0 and tilt 20, with a
pose checked against the lidar map at the shutter. The OAK rode the gimbal by
then, so `world_state/oak.py`'s `MOUNT` applies.

## Pairs and targets

[pairs.json](pairs.json): every pair in which the later look is within 1 m and
20 degrees of camera yaw of the earlier and at least 60 s after it. That is 10
pairs, 0.73-0.94 m apart, covering six target looks; four targets have two
sources, two have one, and the sources are shared. It is a feasibility screen,
not a measurement of a rate.

## Labels

[labels.json](labels.json), drawn on the photographs before any prediction: a
person sits on the window armchair in source 091400 and is gone 11 s later and
in every target, so the armchair and its surroundings are excluded in the four
targets that use that source. Nothing else was seen to change.

## Prediction ([predict.py](predict.py), [geometry.py](geometry.py))

- Each source's depth between 0.3 and 6 m becomes map points through the lens,
  `MOUNT`, the gimbal's pan and tilt, the camera's taped position (2 cm ahead of
  the lidar, 23.5 cm up) and the stored pose.
- Only 2x2 blocks whose four depths agree within max(5 cm, 5%) are used: one
  surface each, sub-sampled 4x4 so a nearer target does not see between splats.
  Blocks that disagree are depth edges and are dropped.
- Points are drawn into the target camera with a z-buffer; the nearest surface
  wins, across all of the target's sources.
- The target's own depth is never read: its pose, angles and moment come from
  the store and its lens from the sources, which the scorer then checks.
- **Copy baseline**: the nearest source's depth map, unmoved, in the same
  0.3-6 m band.
- No alignment of any kind is fitted to the target.

## Scoring ([score.py](score.py))

- **Measured area** of a target: pixels outside the labels, with depth in
  0.3-6 m, belonging to at least one 2x2 block that is one surface, the same
  rule the sources pass. Raw valid pixels without the surface rule are reported
  beside it, not decided on.
- **Useful**: predicted within max(0.15 m, 5% of the target's depth).
  **Unavailable**: no prediction. **Wrong**: the rest.
- **Stop** if fewer than three of the six targets reach 30% useful area; or if
  warping does not beat copying, meaning it is better on at least four of six
  targets with a positive median difference. The third stop rule in the plan,
  "works only when aligned on the surfaces that changed", cannot trigger,
  because nothing is aligned.

## What development showed, and what it changed

Seven development pairs among the 21 looks that are not targets, chosen by the
same rule with gaps from 5 s, were scored before freezing. Warping was useful
on 5-32% of measured area and copying on 2-15%. Shifting the target's heading
or the common tilt by several degrees, or the camera's position by 10 cm,
changed accuracy by a few points at most, so the geometry chain is not what
limits it. From one spot turning in place, the errors where both looks
measured a surface were 7-28 cm at 2-3 m.

Two choices were made after seeing that, and before any target was scored:
counting a target's measured area by the surface rule, because its raw depth is
full of isolated speckle on plain walls that no prediction could match; and
reporting the raw count beside it rather than deciding on it. The thresholds,
the stop rules, the pairs and the labels are the plan's and were not changed.
