# Spatially grounded semantic world state

Status: phases for capture, storage, perception, placement, search and console
inspection are deployed. This document records the remaining design constraints;
current operation is in [`world_state/README.md`](../world_state/README.md).

## Goal

Remember visual regions with enough provenance to answer where they were seen,
search them by a person's description, and drive to a placed result through the
existing navigation boundary.

SLAM and Nav2 remain authoritative for geometry and movement. World state may
read pose, map and reachability. It does not command motors. Voice access is
read-only except for the existing navigation tool used after a result is chosen.

## Current design

- YOLOE proposes visual regions.
- DINOv2 supports re-identification; SigLIP2 supports text search.
- Every observation keeps its source frame, camera, capture time, pose, bearing,
  elevation, uncertainty, optional range and model backend.
- Two separated bearings with enough parallax can establish a map position.
- Map visibility, height and OAK range reject inconsistent crossings.
- Appearance can reject or choose candidates after geometry accepts them.
- Ambiguous evidence remains pending.
- A map reset starts a new map session without deleting the observation record.

The design deliberately has no fixed object vocabulary and no local language
model in the inspection path. Both approaches were measured and removed because
names drifted and visual re-identification failed in unsafe directions.

## Completed work

- additive SQLite store and frame provenance;
- isolated perception sidecar with TensorRT and CPU fallback;
- capture-time pose interpolation and per-bearing uncertainty;
- bearing triangulation, robust multi-ray refinement and map visibility bounds;
- elevation and range constraints;
- text search over stored regions;
- console entity, observation, frame and map views;
- read-only voice tools for finding, approaching and measuring placed things;
- replay and focused geometry/perception benches.

## Remaining acceptance work

The next real-room drive must keep the OAK awake and record ranges. It should
show that range constraints prevent false crossings between different objects on
the same sight lines without reducing correct associations.

The OAK-to-gimbal rotation is measured. The translation between the cameras and
the gimbal camera's translation from the SLAM pose are not yet validated. Absolute
height above the floor therefore remains unavailable.

Semantic frontier selection remains out of scope until identity is reliable on a
fresh driven recording. Adding movement authority before that proof would couple
navigation to an unvalidated world model.

Acceptance requires:

1. replay of the previous failure recording through the proposed change;
2. offline suites passing without weakening ambiguity or provenance rules;
3. a driven recording with independently checked object assignments;
4. running-service verification through TCP 8769;
5. no cross-backend vector comparisons and no placement reused across maps.

## Rules that remain closed

- Do not infer persistent identity from one picture.
- Do not convert nearest text into an object name.
- Do not merge evidence merely to empty the pending pool.
- Do not hide uncertainty inside one fused score.
- Do not allow semantic state to bypass Nav2 or the daemon's movement checks.
- Do not store runtime frames, model files or databases in Git.

Earlier measurements and abandoned variants remain available in Git history and
the bench scripts. They are not current operating documentation.
