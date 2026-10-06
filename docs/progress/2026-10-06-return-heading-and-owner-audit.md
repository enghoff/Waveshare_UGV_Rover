# An explicit return heading resolves the route check; changed owners remain provisional

The morning route blockage has a concrete preparation remedy. A movement-free
live Nav2 query with the B start facing its return leg produces a 0.833 m path,
instead of the earlier 3.260 m loop. The current global map also admits the
prepared paths and sampled turn footprint. This investigation issued no movement
commands, production fix or entity-policy deployment. R-WS-13 remains open; R-WS-17 and R-WS-18
remain proposed.

This continues the [failed morning precheck](2026-10-06-morning-route-precheck.md).
That failure is preserved: its original start heading was 167.486 degrees, goal
heading -12.514 degrees, and B-to-A length 3.260168 m. The new planning-only
request uses the same B and A coordinates and goal heading, changing only the
hypothetical start heading to -12.514 degrees. It succeeds with error code zero
and length 0.832697 m. This is a causal comparison of planner requests; no
stationary turn or ensuing drive has yet been observed on hardware.

Global and local costmaps were then read through their `GetCostmap` services.
The captured global grid is in `map`. Sampling the configured 0.20 m circular
body at 5-degree intervals around A and B gives maximum covered cost zero.
Sampling the four revised paths at at most 1 cm intervals, with interpolated
headings, also gives maximum covered cost zero. These paths and grids were
captured separately; this does not guarantee future clearance or prove local
clearance at B before the rover gets there. A fresh local check remains required
before the painting-facing and return-facing turns.

The earlier archived-grid Python lattice gives 2.947 m for the original return
heading and 0.668 m when facing the return. Its grid predates the current query,
and its arrival tolerance/search differ from Nav2. It supports the preparation
hypothesis but is not an exact model of the live 3.260/0.833 m answers. No
navigation setting should be changed on this result.

The saved painting placement predicts bearings 93.60 degrees at A and 82.67
degrees at B: only 10.93 degrees difference. Thus the short missing-view run
cannot be advertised as meeting the frozen 12-degree two-view criterion on that
prediction. Actual measured bearings and pure-subject coverage must decide it;
the short run can supply a missing side view while full acceptance remains owed.

Evidence is in `captures/visibility-morning-prepared-1/`: the additional live
path `return-facing-precheck.txt`, `current-costmaps.json`,
`return-heading-offline.json` and `return-preparation-proof.json`.
The current costmap JSON SHA-256 is
`2fb500d4d39d491ce4fe8f0ada0c5ac2dc6cc7e2c49e008f2e7d405202610e4a`.
The capture used an inline read-only helper in `/tmp`; its reusable equivalent
is `experiments/entity_association/capture_route_costmaps.py`. Use an external
timeout as well as ROS waits, since a wedged DDS can defeat spin timeouts.

## Exploratory review of every changed owner in the daylight candidate

The matching-only candidate frozen at `a61f1c4` changes seven final owners in the
October 5 actual-call replay. All seven were reviewed against full photographs,
their selected boxes and stored outlines, plus two chosen historical examples
per destination. Eighteen photographs covering 21 observations were recovered
from the rover's existing frame archive; every image matched its remote SHA-256.
No fresh perception or movement was requested for that recovery.

This is post-score exploratory review: assignments were already inspected, so
these judgments are not independent acceptance labels. They do not overwrite
the original subject labels or historical proxy score, and selected references
do not prove that an entire historical record is pure.

| Changed view | Visual finding |
|---|---|
| 68928 to object:301 | Same dining painting as the selected examples; its outline removes foreground chair tops. It still joins a different historical painting record from object:328. |
| 68967 and 68978 to object:328 | Same painting parts; the central chair is removed by the stored outlines. These remain occluded views, not the missing clear side control. |
| 68926 to object:401 | Doorway region with foreground chair still selected; mixed or unresolved, not clean identity truth. |
| 69013 from object:320 to object:466 | Same occupied green armchair as the candidate examples. Selected old-owner examples include both a green-chair part and the different purple armchair, revealing pre-existing mixed ownership. |
| 69040 to object:487 | Same window-side landscape painting; distinct from the dining-side landscape. |
| 65145 to object:349 | Same bright dining-room window region and wall location. This comparison does not establish range accuracy. |

Six changes are visually consistent with the chosen references; one remains
mixed/unresolved. There is no obvious clean-object mismatch among these seven
reviewed changes. This strengthens the reason to test the narrow candidate but
does not establish its global quality: the old mixed painting/toolbox cross-target
failure remains scored, record purity is only sampled, and independent acceptance
and controlled new-view coverage remain absent.

Review evidence is `.cache/visibility-owner-audit/`: `observations.json`,
`images.zip` with its image hash manifest, comparison sheets, stored-outline
sheet and `review.json`. The candidate result itself was not recomputed or
modified. These are offline experiment/docs changes, requiring no deployment
or service restart. The next drive must explicitly separate travel, painting
view and return headings, preserve the time limits and check current local
clearance. Further movement needs a charger-disconnected handover because the
owner was previously told the rover could be put back on charge.

## Local frame check and control ownership

A second costmap capture includes `base_link` transformed into each costmap's
own frame. The local grid is in `odom`; treating its coordinates as `map` would
be invalid. The local body centre has cost 217 and the sampled 0.20 m physical
footprint has maximum cost 253, with the nearest lethal cell centre 0.242 m
away. An initial preparation check incorrectly applied the 253 inflated-body
band to every body cell, doubling the inflation. It refused this real snapshot.
The corrected check applies the 253 limit to the body centre and the 254 lethal
limit to the physical footprint, while rejecting unknown cells. It admits the
snapshot. This changes only the offline preparation check, not production
navigation or its collision decisions. Frame, age, incomplete-grid and occupied
cell failure checks are verified against mutations of that same real capture.

`capture_route_costmaps.py` now records the body transform and its age.
`check_recorded_turn.py` reports the local footprint check; `--live` additionally
rejects a capture older than five seconds. Archival replay is explicitly not
live turning authority. Missing transforms or failed checks end the trial.

During this read-only work, rover status changed from the saved HOME to
(-16.992, -16.135, -161.4 degrees) and its movement sequence advanced to 21,
last reporting a completed -90-degree turn. This session's RPC log contains no
movement commands. At the later check it was idle with PWM [0, 0], zero speed
and 90% battery. The global grid puts this newer centre at cost 253, so the
earlier HOME-to-A path cannot stand in for a current-start check. The owner was
asked whether another session is using the rover or those movements have ended;
further motion waits for control/charger coordination and a new HOME preflight.

The owner subsequently confirmed moving the rover closer to the wall. A fresh
movement-free current-start query to A then failed with status 6, error code
205 (`START_OCCUPIED`), and no path. This is a different blocker from the
resolved hypothetical B-to-A return heading. Moving 0.10-0.30 m along its
current heading also crosses occupied cells in both saved maps; a simple
straight-forward suggestion would be wrong.

The existing production `goal_fit.fit` selects a hypothetical 0.250 m escape
toward 126.87 degrees. Projecting that segment into the contemporaneous local
grid gives maximum centre cost 253 and footprint cost 254. The global suggestion
therefore is not sufficient clearance proof for this local scene. No escape was
commanded, and these separate snapshots are not an exact execution replay or a
justified software fix. Some other sampled headings have a clear short local
segment, but they do not establish a complete exit/return through production's
goal fitting and have not been driven. The evidence trial waits for the owner
to place the rover on clear floor, away from the wall/chair, and disconnect the
charger. No reset/refit or speculative navigation change was made.

The [measurement](2026-10-06-return-heading-and-owner-audit.json) retains the
input hashes, live planner comparisons, eleven recording-backed turn checks and
all seven exploratory owner judgments. Additional parking evidence is
`new-home-precheck.txt`, `parking-exit-candidates.json`,
`parking-exit-heading-candidates.json` and `existing-backoff-preparation.json`
under the same local capture directory. Documentation checks passed; changes
are limited to preparation helpers and evidence documents.
