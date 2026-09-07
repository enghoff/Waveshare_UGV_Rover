<!-- requirement-area: WS -->

# Visual memory

What the rover records about the things it has seen, and the rules that decide
when a set of observations becomes a thing with a place. The conventions for
these records are in [README.md](README.md); what actually runs is
[world_state/README.md](../../world_state/README.md).

This is the least settled part of the rover. The rules about evidence are in
good shape and hold up under review; the geometry underneath them does not, and
[R-WS-10](#r-ws-10) is currently the most consequential open fault on the
machine.

<a id="r-ws-1"></a>
### R-WS-1 — Every observation keeps the evidence it was made from

- **State:** settled
- **Evidence:** [world_state/README.md](../../world_state/README.md);
  `python world_state/selftest.py`

An observation retains its source frame, region, capture time, camera, pose,
bearing, elevation, uncertainty, any measured range, the appearance vectors and
the perception backend that produced them. The frame is kept as an image, so a
placement can be checked by eye afterwards — which is how every association
error so far has been found.

<a id="r-ws-2"></a>
### R-WS-2 — A single observation never places a thing or fixes its identity

- **State:** settled
- **Evidence:** [world_state/README.md](../../world_state/README.md);
  `python world_state/selftest.py`

One viewpoint yields a bearing and no distance. Placement requires bearings
crossed from viewpoints far enough apart to have real parallax, and identity that
rests on one picture is a guess with a confident number attached to it.

<a id="r-ws-3"></a>
### R-WS-3 — Evidence that does not resolve stays pending

- **State:** settled
- **Evidence:** [world_state/README.md](../../world_state/README.md);
  `python world_state/selftest.py`

The resolver leaves ambiguous observations unresolved rather than choosing the
best available candidate. Emptying the pending pool is not a goal; a wrong merge
is worse than an unanswered question, because the wrong merge is what later
reads back as a fact.

<a id="r-ws-4"></a>
### R-WS-4 — Appearance vectors are never compared across perception backends

- **State:** settled
- **Evidence:** [world_state/README.md](../../world_state/README.md) — each
  observation records its backend; `python world_state/selftest.py`

The sidecar prefers TensorRT engines built for the Orin and falls back to CPU
ONNX Runtime. Vectors from the two are not comparable, so a similarity computed
across them is a number with no meaning, arriving in the same units as one that
has meaning.

<a id="r-ws-5"></a>
### R-WS-5 — A placement is never reused across map sessions

- **State:** settled
- **Evidence:** [world_state/README.md](../../world_state/README.md) — the
  resolver rejects evidence from another map session;
  `python world_state/selftest.py`

Coordinates mean something only in the map frame they were measured in.

<a id="r-ws-6"></a>
### R-WS-6 — A map that changes without being cleared keeps the record and marks it

- **State:** settled
- **Evidence:** [world_state/README.md](../../world_state/README.md) —
  `follow_map` moves the session within seconds of the map identity changing

A failed restore, or a graph built from scratch, gives the navigation stack a
different map identity. The rows stay and are shown as measured against a map
that has gone, rather than being deleted or silently drawn in the current room.
Deleting them would destroy the crops, which are the part that is still true.

A reboot onto the same map changes nothing, because the pose graph persists and
the coordinates still mean what they meant.

<a id="r-ws-7"></a>
### R-WS-7 — Nothing assigns a name from a fixed vocabulary or from the nearest text

- **State:** settled
- **Evidence:** [world_state/README.md](../../world_state/README.md);
  [decisions/cosmos-reason2.md](../decisions/cosmos-reason2.md)

Text search compares a description against stored visual features. It does not
produce a label the store then treats as what something is. Both a fixed class
vocabulary and a local vision-language model were tried and removed: names
drifted on byte-identical frames and re-identification failed in unsafe
directions.

<a id="r-ws-8"></a>
### R-WS-8 — Uncertainty is not collapsed into a single fused score

- **State:** settled
- **Evidence:** [world_state/README.md](../../world_state/README.md) — geometry,
  visibility, elevation, range and appearance are separate gates

Geometry decides where something can be; appearance may then reject or choose
among candidates geometry has already accepted. Multiplying them into one
confidence would let strong appearance rescue impossible geometry, which is the
failure mode that produced the merge errors reviewed on 2026-09-07.

<a id="r-ws-9"></a>
### R-WS-9 — The store grows additively and stays readable backwards

- **State:** settled
- **Evidence:** [world_state/README.md](../../world_state/README.md) —
  migrations in `schema.py`; `python world_state/selftest.py`

Historical columns remain readable even where the current pipeline no longer
writes them, because old observations are the only record of how the rover used
to be wrong.

<a id="r-ws-10"></a>
### R-WS-10 — A recorded bearing is as accurate as the resolver is told to expect

- **State:** failing
- **Broken by:** [2026-09-07 M0 baseline](../progress/2026-09-07-m0-semantic-world-state.md)

The resolver currently crosses bearings using an uncertainty setting of about
a degree and a half. This is an assumption to validate, not a mechanical accuracy
target. Acceptance means measured residual uncertainty is represented honestly
within a useful declared envelope, and unsupported conditions are refused; merely
widening a match tolerance does not satisfy the requirement. See the
[bounded P0 protocol](../plans/autonomous-curiosity.md#bounded-calibration-protocol)
and [current review](../progress/2026-09-07-m0-review.md).

The baseline measured a roughly 1.3-1.6-degree difference between approach
directions at pan -20, 0 and +20, against 0.03-0.05-degree same-direction fit
variation. This is evidence of a direction-dependent error at the tested positions,
not a full-travel guarantee, a standard deviation or the remaining error after
compensation. Approach history was not recorded, so the old observations cannot
be corrected for it after the fact.

Two further faults ride with it: a gain-like walk somewhere between four and
eight per cent, which vanishes straight ahead and reaches one to two degrees at
thirty degrees of pan, and a roll that moves with pan and that neither of the
other faults can produce. The bench cannot pin the gain down more closely because
the room moves while it measures.

Nothing has been re-tuned to hide this. The OAK's mount constant has deliberately
been left at a value known to be about six degrees out rather than moved to one of
several numbers that disagree, because the gimbal it would be measured against is
not yet independently calibrated. The next step is a bounded measurement of
actual pointing and repeatability against an independent reference, separating
correctable bias from remaining physical and measurement variation. The reported
gain-like slope alone does not identify a unique servo correction. Re-measure the
mount only within the supported gimbal envelope, then verify depth attribution to
known objects; stop tuning when the agreed budget or measurable benefit is exhausted.

<a id="r-ws-11"></a>
### R-WS-11 — A thing's height above the floor is known

- **State:** open
- **Blocked by:** the translation between the OAK and the gimbal camera, and the
  gimbal camera's offset from the SLAM pose, are unmeasured — see
  [../plans/semantic-world-state.md](../plans/semantic-world-state.md)

The OAK-to-gimbal *rotation* is measured. Without the translations, elevation is
usable as a relative constraint between observations but absolute height above
the floor is unavailable, which is why elevation can reject a crossing but cannot
say a thing is on a table.

<a id="r-ws-12"></a>
### R-WS-12 — Bare floor and background are not eligible as things to go and look at

- **State:** open
- **Blocked by:** [2026-09-07 M0 baseline](../progress/2026-09-07-m0-semantic-world-state.md)
  found two of fourteen reviewed entities to be floor, one of them placed
  confidently to within 0.18 m

Region proposals include patches of blown-out floor, and the resolver will place
them like anything else. Something choosing where to look next would spend real
distance and battery on them. Deliberate geometric coverage of a floor area
remains a legitimate goal — it is simply a different kind of goal from
inspecting an object.

<a id="r-ws-13"></a>
### R-WS-13 — No association eligible to influence movement is knowingly wrong

- **State:** open
- **Blocked by:** [../plans/autonomous-curiosity.md](../plans/autonomous-curiosity.md)
  (M0, criteria 3 and 8)

The bar is a reviewed acceptance sample of at least fifty association decisions
with zero known incorrect merges among those eligible to influence movement,
counting unresolved cases separately rather than as failures.

Identity came out of the 2026-09-07 review better than the plan assumed: around
250 decisions were reviewed with no error at or above 0.70 appearance similarity.
The four errors found all sit in the 0.55-to-0.70 band and every one is a thing
standing behind another thing, which is where an elevation or range gate earns
its place. Until that band is treated as geometry-only, the requirement is not
met. Six identical dining chairs in the test room are beyond what this component
can distinguish at all, and the honest record says so rather than guessing.

<a id="r-ws-14"></a>
### R-WS-14 — A thing whose map was replaced is recognised when it is seen again

- **State:** settled
- **Evidence:** [world_state/README.md](../../world_state/README.md) —
  `resolve._adopt`, gated at `RECOGNISED`; `python world_state/selftest.py`

The coordinates expired; the crops did not. The first crossing in the new map
that plainly looks like something the rover already owns takes that thing's
identity back with its history intact, and the adoption is refused where two
known things look equally like it. Without this nothing could ever re-place an
orphaned thing, because the resolver only considers what is placed in the map it
is working in.

Things stranded in bulk can also be carried across by hand with `reanchor.py`,
which lines two maps of one room up by the things that appear in both. It is run
deliberately, not on a schedule, and writes nothing without `--apply`.

<a id="r-ws-15"></a>
### R-WS-15 — The visual memory can be emptied without disturbing the map

- **State:** settled
- **Evidence:** [world_state/README.md](../../world_state/README.md) —
  `world_state_clear`

A repeatable experiment needs to start from an empty store on a map the rover
already has. This is the one direction the two are separable in; the other
direction is [R-NAV-4](navigation.md#r-nav-4).

A clear is refused while a look is in flight, and rows that survive a refusal are
marked as belonging to the map that has gone rather than left looking current.

<a id="r-ws-16"></a>
### R-WS-16 — An observation is only given a direction when the rover's place on the map has been confirmed

- **State:** open
- **Blocked by:** [2026-09-07 refit window](../progress/2026-09-07-refit-window.md)
  — nothing in `world_state` or `rover_world.py` consults navigation's
  `map_settled`, so an unconfirmed restore still produces bearings

This is [R-WS-5](#r-ws-5)'s neighbour and the gap between them is easy to miss:
R-WS-5 keeps evidence from being read against the wrong *map*, while this one is
about the wrong *pose within the right map*.

Capture already refuses a direction when there is no map identity or no fresh
transform — `rover_world._world_pose` gates on `position_trusted`, which is the
question of whether a pose exists and is recent. That is not the question of
whether it is correct. The navigation stack answers the second one separately in
`map_settled` ([R-NAV-2](navigation.md#r-nav-2)): after a restore, the rover's
place on its map is the mapper's anchor until something confirms it, and an
anchor that landed somewhere else is neither confirmed nor disproved.

So a rover that came up on a restored map it could not place has a fresh
transform, a real map identity and a confidently wrong heading, and every look it
takes is recorded with a bearing measured from that heading. Measured on
2026-09-07: 34 observations were stamped this way between a restart and the fit
that corrected it, from a heading then shown to be 152.5 degrees out. None was
placed, because a parked rover cannot cross bearings for want of parallax — so
the placed record survived by [R-WS-2](#r-ws-2) holding for a reason unrelated to
the pose being wrong, and the rows stay crossable once the rover drives.

The fix is not to discard the look. The picture is worth keeping — the same
reasoning as R-WS-5, where the coordinates expire but the crops do not. What
should be withheld is the direction, exactly as it already is when the map
identity is missing.
