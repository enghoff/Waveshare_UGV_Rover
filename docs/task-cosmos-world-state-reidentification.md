# Task: spatially grounded world-state re-identification

_Status: recommended follow-up to [`task-cosmos-world-state-poc.md`](task-cosmos-world-state-poc.md), 2026-09-01._

This document supersedes the **Expected follow-up** section of the completed
Cosmos world-state POC. Keep the original task as the record of what was built and
measured; use this document for the next implementation step.

## Decision

Do **not** proceed to semantic frontier selection yet, and do **not** spend the
next iteration trying to prompt Cosmos into performing entity identity better.

> **Cosmos 3 was tried on 2026-09-01 and this fork is now closed.** It cannot run
> on this rover — the 4B Edge variant is an architecture llama.cpp cannot load,
> and the only runnable build needs 6.19 GB of weights against 5.85 GB of measured
> headroom and no swap. Tested off the rover on the rover's own frames, its
> perception is clearly better than Reason 2's and its identity behaviour is
> clearly worse: shown a known list containing a grand piano and a fish tank, it
> matched the armchair to the piano and drew the fish tank into the scene. It will
> not say "new". Reason 2 over-creates and stays truthful; Cosmos 3 over-matches
> and does not. The numbers are in
> [`world_state/README.md`](../world_state/README.md).
>
> One change to this document follows from that result, marked below: the known
> entity list should be **withheld from the perception call**, not merely demoted
> to advisory, because it corrupts the detections and not just the identity field.

The POC established a useful boundary:

```text
Cosmos                         application
------                         -----------
what is visible?               which persistent thing is it?
what is it?                    where is it physically?
where is it in this image?     does it match an existing entity?
description / bbox             entity ID / placement / history
```

Cosmos Reason 2 2B is already good enough at the left-hand side. The next task is
to replace its failed `existing_entity` decision with a deterministic
**re-identification layer using measured physical placement plus visual
appearance**.

The target chain is:

```text
camera RGB
   |
   v
Cosmos: label + bbox
   |
   +-----------------------+
   |                       |
   v                       v
appearance crop        aligned OAK depth
   |                       |
   v                       v
visual embedding       calibrated 3-D point
   |                       |
   +-----------+-----------+
               |
               v
        entity resolver
 semantic + spatial + appearance
               |
       MATCH / NEW / AMBIGUOUS
               |
               v
        SQLite world state
```

The most important change from the first POC is that **physical placement is a
first-class part of entity identity**, not merely provenance shown in the popup.
The environment already contains redundant / near-identical furniture, which is
exactly the validation case wanted here: two visually near-identical objects at
different positions must remain two entities.

---

## Existing decisions that are closed

Do not reopen unrelated navigation experiments while doing this work.

- **SLAM Toolbox + the current Nav2 stack remain authoritative** for map geometry,
  robot pose, path planning and control.
- **MPPI has already been validated separately and rejected as unsuitable for
  this rover.** Do not add, benchmark or recommend it as part of this task.
- **RTAB-Map has already been validated separately and rejected as unsuitable for
  this rover.** Do not reintroduce it for semantic mapping, localization or as an
  intermediary merely because RGB/depth is now being used.
- The existing geometric `explore` remains the autonomous exploration baseline.
- Cosmos still gets **no movement authority** in this task.

Nothing below requires a new mapper, local planner or navigation framework. The
world-state layer consumes the existing SLAM pose and map; it does not replace
or compete with either.

---

## OAK-D is available now, not a future dependency

The completed POC deliberately used the gimbal camera and treated OAK-D metric
localization as later work. That scope choice no longer applies to the follow-up.

The OAK-D-Lite is already integrated and running on the Orin, so use it now where
it makes identity more deterministic.

The preferred division is:

```text
gimbal camera                    OAK-D
-------------                    -----
active perception                fixed calibrated geometry
look left/right/up/down           RGB + stereo depth
human-facing visual questions    metric object placement
wide semantic inspection         re-identification evidence
```

Do not remove the gimbal camera. It remains useful for active perception and for
Cosmos' general scene inspection. But when an observation is intended to acquire
or update a persistent **metric object identity**, prefer an OAK RGB frame with
stereo depth aligned to that same RGB frame.

That avoids the unsound combination:

```text
gimbal RGB bbox + OAK depth pixel
```

because those pixels belong to different cameras and different poses.

Instead use:

```text
OAK RGB pixel / bbox
        |
        v
aligned OAK depth at the same ray
        |
        v
OAK intrinsics
        |
        v
3-D point in OAK camera frame
        |
        v
fixed OAK -> base_link extrinsic
        |
        v
current SLAM map pose
        |
        v
map-frame object placement
```

If the fixed OAK-to-rover extrinsic is not already calibrated to the accuracy
needed, make that calibration the first implementation step. Do not substitute a
measured-looking hand estimate and then treat it as ground truth.

No RTAB-Map is needed for this. Projection from calibrated RGB/depth through the
known camera extrinsic and current SLAM pose is sufficient.

---

## Identity must distinguish identical-looking objects by placement

Visual embeddings alone are not sufficient for this environment.

If the room contains two matching chairs, sofas, tables or other redundant
furniture, a correct appearance embedding may quite reasonably say they look
nearly identical. The resolver must therefore preserve the distinction:

```text
chair A
appearance ~= chair B
map position = west wall

chair B
appearance ~= chair A
map position = east wall

=> two entities
```

Conversely, the same sofa viewed from several angles should remain one entity
because its measured placements agree within the expected uncertainty even when
its crop embedding changes with viewpoint.

For mostly static objects/furniture, **spatial compatibility should be a strong
or hard gate**, not just another weak weighted feature.

Conceptually:

```text
new observation
      |
      v
semantic candidates
      |
      v
spatially plausible candidates
      |
      v
appearance ranking
      |
      v
MATCH / NEW / AMBIGUOUS
```

Do not let a very high appearance score merge two objects whose measured
positions are clearly incompatible.

For movable objects such as bottles, bags or cables, location cannot be an
eternal hard identity rule. Keep the first implementation conservative:

- distinguish **static / fixture-like** entities from **movable** entities where
  the semantics make that obvious;
- use placement strongly for static furniture and openings;
- for movable objects, retain placement history and allow a later observation to
  represent relocation only when appearance/semantics give strong evidence;
- when evidence is insufficient, prefer `AMBIGUOUS` over silently merging or
  creating certainty.

A full object-motion model is not required in this slice.

---

## Appearance embeddings

Stop asking Cosmos to choose `existing_entity` as the primary association
mechanism.

**Do not show Cosmos the known entities at all.** This is stronger than the
earlier draft of this section, which allowed the list through "for semantic
context" with the identity answer treated as advisory. Measured on 2026-09-01,
the list does not merely fail to settle identity — it contaminates the
detections. Cosmos 3 handed a list containing a fish tank reported a fish tank in
the picture, with the description copied out of the list; Reason 2 shows a milder
version of the same pull. Ask the model what it can see, and nothing else, so that
its answer is evidence about the room rather than an echo of the world state the
resolver is about to update.

For each Cosmos bbox, crop the corresponding object image with a modest amount of
context around the box and compute a reusable visual embedding.

Start by benchmarking a small general visual feature model such as **DINOv2-S**
(or another comparably small image-retrieval embedding model that fits the Orin).
Do not commit to the model by architecture; expose a tiny interface such as:

```python
class AppearanceEncoder:
    def embed(self, image_crop) -> list[float]: ...
```

The important test is the rover's own data, not a generic benchmark.

Store multiple exemplar embeddings per entity rather than immediately collapsing
all viewpoints into one average vector:

```text
entity:sofa:1
  exemplar A   front
  exemplar B   left side
  exemplar C   oblique
```

Compare a new observation against the best relevant exemplar initially. A single
centroid can be evaluated later if the recorded data shows it is beneficial.

The embedding is **evidence**, not identity on its own.

---

## Metric placement

Add explicit placement to the semantic store once it comes from measured OAK
stereo geometry rather than VLM inference.

The precise schema is an implementation choice, but an observation should be able
to retain something equivalent to:

```text
metric_source             oak_stereo
camera_frame              oak_rgb
camera_point_xyz_m        measured 3-D point
map_point_xyz_m           transformed point
position_uncertainty_m    estimated / measured tolerance
map_session               existing session identifier
```

For an entity, keep a current placement estimate plus the individual observation
placements that produced it. Do not destroy the observation-level measurements
when updating the entity.

A bounding box does not necessarily identify one reliable depth pixel. Use a
robust depth estimate inside the Cosmos box, for example a central mask/region
with invalid stereo pixels rejected and an appropriate median/percentile, rather
than trusting one pixel at the centre.

The exact estimator should be validated against known objects/distances in the
actual room.

Openings such as doorways may need a different geometric representation than a
point object. Do not force every entity into a misleading centroid if a doorway
is better represented by an observed region/plane/bearing. It is fine for the
first follow-up to prioritize furniture and ordinary objects and leave richer
opening geometry for later.

---

## Resolver contract

The resolver should have three normal outcomes:

```text
MATCH       enough evidence for an existing entity
NEW         enough evidence that this is a distinct entity
AMBIGUOUS   evidence is insufficient or candidates conflict
```

`AMBIGUOUS` is important. The first POC showed that always forcing an identity
turns uncertainty into duplicate entities. The opposite error -- merging two
identical chairs because they look the same -- would be worse because it destroys
real structure in the environment.

A reasonable first resolver sequence is:

1. **semantic gate**
   - compare only compatible kinds/labels;
   - permit known synonyms such as sofa/couch without broad fuzzy merging;
2. **map-session / geometry check**
   - compare measured positions only when they are expressed in compatible map
     geometry;
3. **spatial gate**
   - for static objects, reject candidates clearly outside the placement
     uncertainty/tolerance;
4. **appearance comparison**
   - compare the new crop embedding against stored exemplars;
5. **temporal/history check**
   - use recent visibility and placement history as additional evidence;
6. return `MATCH`, `NEW` or `AMBIGUOUS` with concise diagnostic scores/reasons.

Do not turn the score into a fake calibrated probability. Keep the individual
pieces inspectable:

```text
semantic: compatible
spatial_distance_m: 0.18
spatial_gate: pass
appearance_cosine: 0.91
result: MATCH object:7
```

or:

```text
candidate object:7  appearance 0.98  position 4.2 m away -> spatial reject
candidate object:9  appearance 0.97  position 0.16 m away -> match
```

That second case is the redundant-furniture test the system must pass.

---

## World-state schema changes

Preserve the existing observations/entities split and provenance. Extend it rather
than replacing it.

Useful additions are likely to include:

```text
observations
  camera_source
  metric_source
  camera_point_json
  map_point_json
  position_uncertainty_m
  association_result
  association_reason_json

appearance_embeddings
  observation_id
  encoder_id
  vector/blob

entities
  placement_json              current best estimate, nullable
  placement_map_session
  placement_updated_at
```

The exact representation of the vector can be a SQLite blob or a sidecar format;
keep it simple at this scale. Do not introduce a vector database for dozens of
objects.

Keep raw Cosmos output, stored frames and all previous observation geometry so
association mistakes can be reconstructed later.

---

## Camera strategy to benchmark

Because both cameras are already available, compare rather than assume.

### A. OAK-only world inspection

```text
OAK RGB -> Cosmos -> bbox
                  + aligned depth -> placement
```

This is the cleanest metric path and should be the first implementation baseline.

### B. Gimbal semantics + OAK confirmation

Keep the gimbal for broad inspection / active perception, but require a separate
OAK observation before assigning a persistent metric placement to an entity.

Do not invent cross-camera pixel correspondence. Any gimbal-to-OAK association
must happen at the entity/geometry level, not by reusing pixel coordinates.

The result may be that `world_inspect` uses OAK by default while `look` remains the
gimbal-backed conversational/active-perception operation. That separation would
be entirely reasonable.

---

## Validation dataset: use the redundant furniture already in the room

The current environment is better than a synthetic test because it already
contains visually similar / identical furniture at different physical locations.
Use it deliberately.

Build a small recorded benchmark from stored frames/depth/poses before tuning the
resolver. Label enough pairs to answer:

```text
same physical entity, different viewpoint       should MATCH
different entity, different appearance          should not MATCH
different entity, near-identical appearance     must not MATCH
identical frame replay                           must MATCH
```

At minimum include:

1. the same sofa/table/chair from several rover positions or camera views;
2. two redundant/near-identical pieces of furniture at distinct map positions;
3. an identical saved-frame replay;
4. an object leaving and re-entering the field of view;
5. if practical, one movable object deliberately relocated, recorded as a
   separate diagnostic case rather than used to tune static-furniture rules.

Run candidate encoders and resolver thresholds offline against the recorded
fixture first. Do not tune by repeatedly driving the live rover until a number
looks good.

---

## Acceptance criteria before semantic frontier selection

Do not proceed to model-influenced exploration merely because duplicate counts
improve. Require all of the following on the real rover:

- [ ] identical-frame replay reuses the existing IDs rather than creating a
      second copy;
- [ ] the same static object observed from materially different viewpoints
      retains one ID;
- [ ] two visually near-identical pieces of furniture in different physical
      locations retain **different** IDs;
- [ ] metric positions for a repeatedly observed static object agree within a
      documented tolerance appropriate to the OAK/stereo/calibration;
- [ ] a high appearance similarity cannot override clearly incompatible physical
      placement;
- [ ] uncertain cases can remain `AMBIGUOUS` without creating or merging an
      entity incorrectly;
- [ ] raw observations, embeddings, placement evidence and association reasons
      are visible enough in the World State popup/diagnostics to explain a bad
      match;
- [ ] world state persists across restart without losing identity/placement
      evidence;
- [ ] semantic clear still does not touch SLAM/Nav2 state;
- [ ] map-session changes do not silently compare incompatible stale coordinates;
- [ ] the added embedding/depth work does not destabilize lidar, SLAM, Nav2,
      STOP, camera ownership or Qwen voice operation.

Only after these pass should semantic frontier selection be reconsidered.

---

## What not to do in this follow-up

```text
Cosmos-selected frontiers
semantic explore
navigate_to(object:* / place:*)
direct model movement
MPPI evaluation or integration
RTAB-Map evaluation or integration
new SLAM or navigation stack
vector database
knowledge graph / ontology framework
continuous video-rate Cosmos inference
model-generated metric coordinates
aggressive automatic entity merging
```

The task is deliberately narrower:

> **Turn good Cosmos detections into stable, spatially grounded persistent
> entities, including the ability to distinguish two objects that look the same
> because they occupy different places in the world.**

If that works, the current world-state component becomes a credible foundation
for semantic exploration. If it does not, the failure remains isolated to the
identity/placement layer and the rover's navigation stack remains unchanged.
