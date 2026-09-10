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

**The mechanism this asked for now exists; the measurement that condemned it has
not been retaken.** That is the whole of why this is still `failing`. What
follows is where each piece stands.

*The direction-dependent error is measured and represented.* Approaching one
commanded angle from opposite sides puts the camera 1.19 to 2.23 degrees apart,
and across seven campaign sessions on 2026-09-07 — two tilts, three board
distances — that figure held between 1.59 and 1.72 degrees, a spread of 0.13.
It is the most stable thing the bench measures. Since 2026-09-07 the rover
records which way the pan servo last travelled, so a look reached from the
descending side is charged 2.3 degrees of bearing error instead of being
silently believed to 1.5. Observations recorded before that date carry no
approach and cannot be corrected after the fact.

*There is a declared envelope and unsupported conditions are refused.*
Commanded pan -20 to +20, validated at tilt zero and at tilt +20 — the two
values that account for 98.8% of the looks this rover has taken. A look outside
that pan range keeps its picture and records no direction at all. See
[the tilt-20 result](../progress/2026-09-07-gimbal-tilt20-passes.md).

*The gain is bounded inside the envelope and depends on tilt.* Ascending-only
gain error came out +0.48% at tilt zero and -0.92% at tilt +20, both inside the
1.0% the protocol allows, which at pan 20 is under 0.2 degrees of pointing. The
earlier "four to eight per cent" walk, and the claim that the bench could not
pin it down because the room moves while it measures, were an artefact of
measuring with the board too far away: every repeatability figure improved three
to four fold when it came in from 0.70 m to 0.47 m. The 1.5-point difference
between the two tilts is real and means the gain must be measured at each tilt
rather than interpolated.

*A roll that moves with pan is still unexplained*, and neither of the other two
faults can produce it.

*Nothing has been re-tuned to hide any of this*, and no gain correction has been
applied — the candidate rule is to leave the gain alone. The OAK's mount
constant was measured against the printed board and adopted on 2026-09-07, so
the note that used to stand here about deliberately leaving it six degrees out
is history; see
[the mount entry](../progress/2026-09-07-p0-oak-mount.md).

**What is left is the acceptance measurement.** The baseline that set this to
`failing` was taken on a driven recording, where half the bearings fell outside
the 1.5 degrees the resolver expects. Only another driven recording, taken
through the current envelope and the current gates, can lift it.

<a id="r-ws-11"></a>
### R-WS-11 — A thing's height above the floor is known

- **State:** open
- **Blocked by:** the gimbal camera's offset from the SLAM pose is unmeasured —
  see [../plans/semantic-world-state.md](../plans/semantic-world-state.md)

Both the rotation and the translation between the OAK and the gimbal camera are
measured, as of the [2026-09-07 mount
measurement](../progress/2026-09-07-p0-oak-mount.md): the OAK sits 87 mm forward,
3 mm to the right and 94 mm below the gimbal camera's optical centre, with the
forward figure good to about a centimetre. That closes half of what blocked this.

What is left is where the gimbal camera itself sits relative to the pose SLAM
reports. Until that is known, elevation is usable as a relative constraint between
observations but absolute height above the floor is unavailable, which is why
elevation can reject a crossing but cannot say a thing is on a table.

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

**"Bare" is the load-bearing word, and it is easy to over-read.** This is about
a patch with no object in it: blown-out floor, a featureless stretch of wall, a
window the camera has turned white. **A rug, a mat or a ceiling fan is an
object** and belongs in the store like any other — floor-level and ceiling-level
things are things, and the owner may well want the rover to know about them. The
[acceptance drive](../progress/2026-09-07-m0-acceptance-drive.md) first counted a
patterned rug and a ceiling fan against this requirement and that was wrong. A
rug appearing twice is a different fault altogether — a split, not an eligibility
question.

**It then named two more that were also wrong, and correcting them leaves the
requirement with no instance on that recording.** `object:2` and `object:24`,
recorded here as blown-out wall and window patches, are a ceiling light panel and
a doorway through to the next room; the bare crops inside them are wrong
attachments, which is a merge and not an eligibility question either. Re-reviewed
at a size where the labels can be trusted, one entity of 43 is arguably a patch
of nothing and it was seen twice, which is below what the resolver will place.
See [the re-review](../progress/2026-09-07-no-bare-patches.md).

**That is not grounds to settle this.** One recording that fails to reproduce a
fault is not a demonstration that it is gone, and the earlier baseline that
opened this requirement found two of fourteen entities to be floor. What changed
between them — the corrected mount, the capture gates, or `perceive._blank`
refusing crops with no picture in them — is not established. The held-out drive
is what would settle it, and until then nothing should be built to filter a fault
nobody can currently reproduce.

<a id="r-ws-13"></a>
### R-WS-13 — Identity-dependent actions use independently validated associations

- **State:** open
- **Blocked by:** [../plans/autonomous-curiosity.md](../plans/autonomous-curiosity.md)
  (M0b)

The bar is an independently reviewed acceptance sample of at least fifty
association decisions with zero known incorrect merges among those eligible for
actions that rely on persistent identity. Review every eligible confidence band;
appearance similarity alone is not calibrated identity confidence. Count unresolved
cases separately and demonstrate useful coverage of independently named physical
targets. Repeated frames of the same physical association are not independent cases.

The scope was revised on 2026-09-10: an uncertain association may motivate a bounded
inspection to test it under R-AUT-12, but cannot be treated as established identity
by that inspection or any following action. The distinction must be enforced at dispatch,
not inferred from the goal's name. See the
[M0 decision](../decisions/m0-hypothesis-inspection.md). Neither gate has passed;
the earlier measurements below retain their original acceptance interpretation.

Identity came out of the 2026-09-07 review better than the plan assumed: around
250 decisions were reviewed with no error at or above 0.70 appearance similarity.
The four errors found all sit in the 0.55-to-0.70 band and every one is a thing
standing behind another thing. Six identical dining chairs in the test room are
beyond what this component can distinguish at all, and the honest record says so
rather than guessing.

**The remedy this record used to propose does not work, and the
[acceptance drive](../progress/2026-09-07-m0-acceptance-drive.md) is why.** It
said that a thing standing behind another thing is where an elevation or range
gate earns its place, and that treating the 0.55-to-0.70 band as geometry-only
would settle it. On a fresh driven recording, 16 of 394 reviewed attachments were
plainly wrong and every one is the same case — a dining chair merged with the
framed picture on the wall behind it — and neither gate separates them. The wrong
look places the thing 0.21 m from the centre of the fourteen correct ones, inside
their own 0.40 m spread, so any tolerance that keeps the correct looks admits it.
Raw range does no better: the picture reads 1.52 m where the chairs read 0.67 to
2.90. Elevation does not either, since the affected entities span 12 to 37
degrees of it and so do entities nobody faulted.

The reason sits upstream of any gate. The box drawn round the picture had the
chair in front of it inside the same box, so the depth patch may have sampled the
chair — which means **a range measured through a box holding two objects at
different depths makes the wrong merge look geometrically consistent.**

**What this needs is the region's mask, and the rover already computes one.** The
region model is a segmentation model: its engine returns 32 mask coefficients per
anchor alongside the box, and a 32-channel prototype tensor, and both are copied
back from the GPU on every look before `perceive.py` keeps the boxes and drops
the rest. Decoded on the looks that caused this room's merges, the mask covers
about half of its box and the part it excludes is the chair — verified by eye,
including one case of two chair backs in front of one picture where the mask
comes out as a clean "T". See
[the masks entry](../progress/2026-09-07-masks-are-already-there.md).

**The remedy is to ask the appearance question twice, and it prevents all four
measured faults.** A look that only resembles its entity because of what is
standing in front of it is the look whose score collapses when the intruder is
removed. Scored against the middle of the entity's other looks, the four wrong
attachments read 0.561 to 0.698 as the rover computed them and 0.203 to 0.469
from the masked crop; the gate is 0.55, so all four cross it. Ordinary looks fall
by a median of 0.061 and a 95th percentile of 0.222. **Refusing an attachment
whose score drops by 0.20 or more catches four of four and costs 25 of 360
correct attachments**, and a refusal is a split rather than a merge, which is the
direction acceptance asks for. Replacing the vector outright instead is the wrong
way to use the same mask: it would lose 73 correct attachments to catch the same
four. Costed at 84 ms on a look whose median is 550. See
[the collapse test](../progress/2026-09-07-masking-the-crop.md).

**0.20 is a development candidate and not a settled threshold.** It was chosen
after seeing four positives, which is the thing acceptance forbids counting as
independent evidence, so it is owed a held-out recording with the threshold frozen
first. Two of the six faulted entities — `object:5` and `object:29`, which mix in
a cabinet, a window, a television and a person — have wrong looks that were never
individually identified, so the rule is untested on them.

**Sampling the range on the masked pixels cannot be tested on the recording**,
because the recording keeps each box's computed distance and never the depth map
it came from. Keeping the depth evidence is a prerequisite for testing any range
remedy offline. Live, the mask does separate the two objects — a picture's box
read 1.760 m where the largest rectangle inside its mask read 3.801 m — but both
figures came from a sliver at the top of the depth camera's picture, because the
picture in this room sits above its 43-degree vertical field.

**The collapse test is deployed and the merges are still there, at a rate that
is now measured rather than sampled.** Every thing built from the drive of
2026-09-08 was given a written verdict by eye — 76 of them, in
[world_state/labels/m0-2026-09-08.json](../../world_state/labels/m0-2026-09-08.json)
— and 17 hold looks at two different physical objects, against a criterion that
allows none. Ten more real objects are split across several things, which
nothing in the component reports at all. That labelled set is now the instrument
every candidate remedy is scored against, because the three faults spotted by
eye that preceded it were too few to tell a remedy from noise: one attempt
separated a chair from a painting and merged a person into a sofa in the same
change. See [the labelling](../progress/2026-09-08-every-thing-labelled.md).

**Placement uncertainty was the most promising lead and it cannot gate.** A
thing holding two objects is placed between them and says so — 0.60 m of stated
uncertainty against 0.36 m for a real object — but the tightest threshold
admitting no merge sits at 0.135 m and leaves one of fifty-two real objects
eligible, and an arbitrary control (how many looks a thing holds) produces a
nominally better gate. The reason is not the threshold: a person on a sofa or a
chair in front of a painting is a few tens of centimetres from what it was
merged with, so the rays cross where they would have crossed anyway and the
geometry has nothing to notice. The number remains useful for *ranking* what to
re-look at, which is what `autonomy/goals.py` already does with it. See
[the measurement](../progress/2026-09-08-uncertainty-cannot-gate.md).

**Bounded revision is implemented as an offline experiment and is not an
accepted remedy.** The [September 10 comparison](../progress/2026-09-10-bounded-entity-fitting.md)
reproduces all 76 original entity memberships before testing changes. The full
prototype separates four of eight reviewed mix-ups, leaves three mixed and one
unresolved, but retains only 50.7% of same-object pairs within the 52 originally
labelled clean entities. Its conservative depth contradiction check makes no
correction on the three local drives. R-WS-13 remains open. Observation-level
constraints are now available to score corrections without carrying an old
entity's mixed verdict onto a successfully separated fragment.

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

- **State:** settled
- **Evidence:** [2026-09-08 the restart demonstration](../progress/2026-09-08-restart-withholds-directions.md);
  `python rover_daemon/selftest.py`
- **How it is met:** `rover_world._world_pose` refuses a direction unless
  navigation reports both a trusted position and `map_settled`, the offline
  half is covered by `python rover_daemon/selftest.py` — the withholding, and
  that a later confirmation does not give an earlier bearing back — and the
  hardware half was demonstrated on 2026-09-08 across a navigation restart
  during a drive: eighteen regions recorded with no direction and none placed,
  every picture kept, three of them keeping a measured distance, and no bearing
  put back afterwards

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
identity is missing. That is what the capture path now does.

The half worth stating separately is that the withholding is permanent. An
observation's row is written once and nothing ever puts a pose back onto it, so
confirming the rover an hour later cannot retroactively validate a bearing
recorded before the confirmation — which is the trap this requirement names and
the reason the fix is a gate at capture rather than a filter at read time.
