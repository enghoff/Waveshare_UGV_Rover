# Spatially grounded semantic world state

Status: capture, storage, perception, placement, search and console inspection
are deployed and their rules are settled requirements. What remains is geometric:
[R-WS-10](../requirements/world-state.md#r-ws-10) is failing and blocks the rest.
Current operation is [`world_state/README.md`](../../world_state/README.md); the
measurement that set this back is
[the M0 baseline](../progress/2026-09-07-m0-semantic-world-state.md).

## Goal

Remember visual regions with enough provenance to answer where they were seen,
search them by a person's description, and drive to a placed result through the
existing navigation boundary.

SLAM and Nav2 remain authoritative for geometry and movement. World state may
read pose, map and reachability. It does not command motors. Voice access is
read-only except for the existing navigation tool used after a result is chosen.

## The rules this plan established

Everything the design settled is now a requirement, so that it can be checked
and so that a measurement can move it. The plan no longer restates them:

| Requirement | What it holds to |
|---|---|
| [R-WS-1](../requirements/world-state.md#r-ws-1) | an observation keeps the frame, pose, bearing, uncertainty, range and backend it was made from |
| [R-WS-2](../requirements/world-state.md#r-ws-2) | one picture never places a thing or fixes its identity |
| [R-WS-3](../requirements/world-state.md#r-ws-3) | ambiguous evidence stays pending rather than being merged to empty the pool |
| [R-WS-4](../requirements/world-state.md#r-ws-4) | vectors are never compared across perception backends |
| [R-WS-5](../requirements/world-state.md#r-ws-5) | a placement is never reused across map sessions |
| [R-WS-6](../requirements/world-state.md#r-ws-6) | a map that changed keeps the record and marks it, rather than deleting it |
| [R-WS-7](../requirements/world-state.md#r-ws-7) | no fixed vocabulary, and no name taken from the nearest text |
| [R-WS-8](../requirements/world-state.md#r-ws-8) | uncertainty is not collapsed into one fused score |
| [R-WS-9](../requirements/world-state.md#r-ws-9) | the store grows additively and stays readable backwards |
| [R-NAV-4](../requirements/navigation.md#r-nav-4) | clearing the map clears what was measured against it, in one act |
| [R-SAFE-4](../requirements/safety.md#r-safe-4) | what the rover believes about the room grants it no authority to move |
| [R-CTL-4](../requirements/control.md#r-ctl-4) | the conversational model's access to the store is read-only |
| [R-PLAT-11](../requirements/platform.md#r-plat-11) | frames, databases and model weights stay out of Git |

How the pipeline actually works — YOLOE regions, DINOv2 and SigLIP2 vectors, the
resolver's order of gates — is [`world_state/README.md`](../../world_state/README.md).
Why there is no local vision-language model in the inspection path is
[`decisions/cosmos-reason2.md`](../decisions/cosmos-reason2.md).

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

## What is left

The range work this section used to ask for is done. The driven recording of
2026-09-07 kept the depth camera awake, carried ranges for 583 of 598 in-picture
regions, and showed that range constraints reject false crossings without
costing correct associations — of 100 things bearings alone had placed, 11 lost
their placement once ranges were carried, and the two checked by eye were plainly
false. It refuses more than it invents. The measurement is
[the M0 baseline](../progress/2026-09-07-m0-semantic-world-state.md).

What that same recording found instead is that the geometry underneath all of it
does not hold up, and three requirements carry what is left:

| Requirement | State | What has to happen |
|---|---|---|
| [R-WS-10](../requirements/world-state.md#r-ws-10) | `failing` | measure the pan servo's commanded angle against its actual one, from both directions, then re-measure the OAK mount against it. Nothing else can be settled first, because every OAK number is expressed relative to the gimbal camera. |
| [R-WS-12](../requirements/world-state.md#r-ws-12) | `open` | refuse entities made of bare floor, so that something choosing where to look next cannot spend distance on them |
| [R-WS-13](../requirements/world-state.md#r-ws-13) | `open` | demonstrate correct associations and useful physical-target coverage for identity-dependent actions under M0b; the proposed appearance-band/geometry remedy failed replay |

The [M0 revision](../decisions/m0-hypothesis-inspection.md) permits bounded
verification of uncertain hypotheses only after the separate M0a gate (R-AUT-12).
It does not settle identity or activate a different resolver.

[R-WS-11](../requirements/world-state.md#r-ws-11) — absolute height above the
floor — stays open behind the same measurement, because the translation between
the cameras and the gimbal camera's offset from the SLAM pose are still
unvalidated.

Semantic frontier selection remains out of scope until identity is reliable on a
fresh driven recording. Adding movement authority before that proof would couple
navigation to an unvalidated world model, which is what
[R-SAFE-4](../requirements/safety.md#r-safe-4) exists to prevent.

Acceptance for any change here requires:

1. replay of the previous failure recording through the proposed change;
2. offline suites passing without weakening ambiguity or provenance rules;
3. a driven recording with independently checked object assignments;
4. running-service verification through TCP 8769;
5. no cross-backend vector comparisons and no placement reused across maps.

The six identical dining chairs in the test room are left alone deliberately.
Nothing in this component can tell them apart, and the honest record says so.

## Earlier work

Earlier measurements and abandoned variants remain available in Git history and
the bench scripts. They are not current operating documentation, and the rules
that came out of them are the requirements listed above rather than a second
list here.
