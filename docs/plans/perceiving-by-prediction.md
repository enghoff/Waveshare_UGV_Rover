# Perceiving by prediction

Status: proposed 2026-10-12; nothing built, and a proposal, not a promise:
each later stage starts only on fresh evidence from the one before. This is
the specific architecture behind
[the first-principles design](perception-from-first-principles.md), narrowed at
the owner's request to one approach that would let the rover rely on
prediction. It was worked through four drafts with Codex, whose reviews are
kept outside the repository in `tmp/codex-assistance-replies/`
(`ugv-rover-25-predictive-review-*.md`). Where it differs from that design's
"How it would be built", this plan holds. On 2026-10-13 the owner asked for
prediction and error at several levels, and [Levels of prediction](#levels-of-prediction)
was added after two literature surveys, a verification pass and Codex's
critique (`ugv-rover-25-hier-*.md` beside the others). The first experiment ran
on 2026-10-13 and triggered its stop rule for pixel-level recall, which the owner does not take as a gate on the levels above: see step 1 of
[the order of work](#the-order-of-work).

## In short

The rover does not need one large learned world model. It needs predictions it
can check: before a look, what the depth camera and the picture should show,
and how sure that is, from what it has already measured. It learns a surface, a
place or an object in one shot by keeping the look that measured it. Wherever
nothing was measured it says "unknown" rather than inventing a view.

The world model is two models. **A body model** predicts what a command will do
to the rover and the camera. **A house model** predicts what will be seen from
a camera pose by reading stored RGB-D looks back through their own geometry.
Geometry comes first, because it is what the rover measures well. Appearance
comes after, once geometry has shown it can say where the same surface is.
Changes and identities are hypotheses. They stay pending until a later look,
not used to form them, tells them apart. Where no look can, they stay
unresolved.

Errors are explained at the highest level that can account for them. Pixels
and surfaces sit at the bottom, then what an object looks like, then where it
stands, then the room and the place. A moved chair is one change to where the
chair stands, not hundreds of patch errors. Once the chair is predicted at its
new place from its own stored views, a damaged part still shows, as what the
move does not explain.

The rover relies on prediction first for something small: **choosing where to
point the camera during an inspection it is already allowed to make**. It
predicts which pan and tilt will give a usable view of the target with valid
depth, and the predicted choice must beat today's centring rule in paired
trials. Change reports, a second opinion on its position, and filing what it
sees come later. Each is switched on only when it passes a gate frozen in
advance, against the method it replaces and against a floor of its own. The
first experiment uses pictures already on this desktop, and it can stop the
whole line of work if depth recall does not predict enough of a view.

## The model: a body and a house

    p(observation | memory, state, command)
      ~ p(next state | state, command)            body model
        x p(observation | next state, memory)     house model

The state includes:

- the rover's pose;
- the camera's orientation through the gimbal;
- the moment of the shutter;
- the exposure;
- the uncertain state of whatever is in view.

The prediction carries the pose's uncertainty, not just its mean.

The body model predicts *before* a command, from what is known then: the
command, the battery and the floor. That prediction is what tests the model.
The estimate *after* a move, from the gyros recorded during it, is better, but
it tests nothing about anticipation.

Compared with an action-conditioned latent world model such as V-JEPA 2-AC
(which also conditions on its arm's own state) or Dreamer 4, this gives up
three things:

- learned dynamics;
- temporal context;
- filling in surfaces it has never seen.

For a house rover that notices change conservatively, those losses are
acceptable. They would matter for predicting people, contact and long
sequences of actions, which this plan does not attempt.

In return, geometry is read from measurement rather than learned into weights,
so one look is enough to teach it. And the data it needs exists: the body model
needs commands and measured motion, and the house model needs looks with poses.

| Part | Representation | Model | Learns |
|---|---|---|---|
| Body, wheels | whole-move rotation and translation | direction-specific gain and a stopping-tail term, then recursive least squares | one step per completed move |
| Body, gimbal | relative pan and tilt travel | OAK gyro minus chassis gyro, each bias- and scale-corrected, in one frame, over one interval | per pan; a constant zero error is not observable this way |
| House, geometry | immutable RGB-D looks, each with pose, anchor and lens | depth warped into the target view with a z-buffer | one shot, by storing the look |
| House, appearance (later) | dense frozen patch features on single-surface regions | DINOv2-S/14 run densely or DINOv3-S/16, chosen by benchmark; a registered image difference is the floor | one shot; the encoder never trains |
| Correspondence and pose | keypoints on the picture, made 3D from reference depth | XFeat with LighterGlue, 512-1,024 points, then PnP with RANSAC; ORB is the baseline, SuperPoint with LightGlue the challenger | nothing is learned here |
| Place | the look's whole-view descriptor | pooled features from the DINOv2 already running; MegaLoc only if those fail | one shot |
| Surprise | residuals against an empirical null | a small binned table with a minimum count per bin; quantile regression later | as unchanged evidence accumulates |
| Objects (later) | verified bundles of a few RGB-D views each, with the rigid transforms between them, a placement and a path as intervals | two hypothesis tracks per object, placement and content, each tested by re-rendering the bundle ([below](#levels-of-prediction)); a persistence filter per object, with rates shared by kind | a tentative bundle in one shot; rates and nuisance spreads slowly |

## What is kept per look

The rover already keeps the picture, the OAK depth map, the pose, pan, tilt and
shutter time. It would also keep:

- **an anchor that says which map solution the pose belongs to.** This is the
  main unsolved problem in the design. The world store keeps each look's pose,
  but not the odometry or the `map`-to-`odom` transform at the shutter. The
  drive recordings have those transforms only for the windows they cover. The
  installed slam_toolbox exposes no graph revision at all. A correction can also
  bend old parts of the graph gradually, with no jump in the current transform.
  So jumps in `map`-to-`odom` can *warn* of a discontinuity, but they cannot
  certify that two looks share one geometry. Until a look can be attached to a
  graph node and to the graph solution it was taken in, a prediction uses one of
  two kinds of reference. The first is a frozen reference bank under a single
  anchor. The second is a reference whose registration to the target was
  checked independently, with the remaining uncertainty carried. From now on the
  rover should log the shutter's odometry and transform beside each look, and
  record a solution identifier with node attachment if navigation's holder can
  provide one.
- **the gimbal's measured travel.** The OAK carries its own gyro, a BMI270, on
  the gimbal, and the production pipeline does not use it today. Take its rate
  and the chassis gyro's, bias- and scale-correct both, express them in one
  frame and integrate both over the same pan: the difference is the gimbal's
  relative travel. The [2026-10-09 sweep](../progress/2026-10-09-gimbal-whole-pan-travel.md)
  measured the OAK gyro over-reading by about 1.8%, and found the reference and
  the gyro disagreeing by 0.25-0.34 degrees. Those are drift and disagreement
  figures, and that sweep failed its own reference gate. The chassis samples are
  spread evenly across each UART batch, so their timing needs reconstructing,
  and an OAK restart or a gap in the data breaks the interval.
- **later, dense features**, versioned together with their compression, crop and
  lens, and recomputed for every stored look if any of those change.

## How a look is predicted

1. **References.** Choose stored looks under the same anchor whose view
   overlaps the target's, that have depth, and that come from the current
   surface epoch at that place. Prefer the closest viewing direction.
2. **Warp.** Carry each reference's depth into the target camera with a
   z-buffer. The depth comes from the OAK and the picture from the fisheye, so
   geometry goes through the measured mount and both lenses, not through a crop
   of the middle of the picture. Drop depth edges and any region spanning
   several surfaces.
3. **Three kinds of geometry, never mixed.**
   - *Measured*: an OAK or lidar return saw it.
   - *Inferred*: the lidar's 20 cm slice extruded upward, or a door frame
     assumed to continue.
   - *Unknown*: nothing measured it.

   Inferred geometry may guide where to look. It is never evidence that
   something was there and has gone.
4. **Visibility needs both sides.** A reference's depth agreeing with its own
   warp is only self-consistency, not proof that the surface is visible from the
   target. A claim that something is gone or new also needs valid *target*
   depth along the ray. Missing target depth means unknown, and an occluder is
   not a change.
5. **Compare.** Each pixel, then each region, is classed as one of:
   - consistent;
   - farther through measured-occupied space (gone);
   - nearer in measured-free space (arrived, or an occluder);
   - unknown.

   Darkness, saturation and blur make a region unscorable, never surprising.
6. **Keep two kinds of evidence apart.** Background evidence anchors the pose;
   object-change evidence says what moved. The same moved furniture must never
   both redefine where the rover is and confirm its own change. That is the
   shakedown's failure in another form.

The counts in this plan were made by Codex on 2026-10-12. They come from a
read-only copy of the world store taken at 11:44 and from the benchmark's
[frame index](../../experiments/perception_bench/frame_index.json); the
reviews that report them are named in the status above. On the 27 depth maps
already copied to this desktop, 65% of pixels fall between 0.2 and 6 m,
ranging from 39% to 85% per frame. That is availability, not
accuracy, and it covers only the OAK's 65 by 40 degrees. How much of a view is
usefully predicted is for the first experiment to measure.

## Telling errors apart, and when not to

**A whole-view rotation has three possible causes**: the rover's heading, the
gimbal's pointing, or the shutter's timing. Nothing is recalibrated from such a
rotation unless one cause is pinned down independently, and even then it is
supporting evidence, not proof:

- the two gyros measure the gimbal's relative travel, but not a constant zero
  error;
- when several pans from one standstill all need one common correction, a
  heading error and a gimbal zero error explain it equally well;
- the lidar's own fit is vulnerable to the same moved furniture.

So the default answer is "unresolved".

**The rover's position is cross-checked by visual relocalisation.** Keypoints
in the look are matched to 3D points from reference depth, and the camera pose
is solved by PnP.

- **Why PnP rather than parallax between looks.** Between the closest control
  pairs, within 10 cm and 3 degrees, the median baseline is 11 cm, which may be
  weak for triangulating a translation from the two looks alone. PnP does not
  need that: it matches 2D points in the look to 3D points that the reference's
  own depth already measured. The first experiment's pairs are 0.7-0.9 m apart.
- **Guarding against the shakedown's failure.** A confident match on furniture
  would repeat what the lidar did there, so the pose must hold up three ways:
  - on background structure outside any candidate change;
  - against competing places retrieved from elsewhere;
  - on correspondences withheld from the fit.
- **No false agreement.** A pose it cannot constrain is reported as
  "unobservable", not as "agreeing".
- **The gimbal's pivots.** Comparing a camera pose with the chassis pose needs
  them, and they are not modelled.

## Surprise, and the units it is counted in

- **The null** is the residual on unchanged surfaces, kept in a small table: a
  few bins, with a minimum count in each. It is calibrated through the whole
  frozen pipeline (retrieval, warping, alignment and grouping), with the
  memory's size and update rule frozen too.
- **Per look**: the largest region score in the look, compared with a threshold
  set on one block of looks and checked on a later block. These rates are
  empirical and the blocks are named, because days, lighting, pose quality and
  memory size are not interchangeable. A look outside what calibration saw is
  refused, not scored.
- **Per event, for the owner**: reports are merged into events, and each event
  is counted once within a deadline frozen in advance. Repeated pictures of one
  event are not extra trials. Every claim is stated with its interval:
  - with zero false alarms in 20 independent room-controls, the one-sided 95%
    upper bound is about 14%;
  - a claim below 5% needs about 60 clean controls;
  - a claim below 1% needs about 300.

  Room-controls, owner episodes and repeated looks are different units, and are
  never pooled.

## Changes and identity, kept pending

- **A change is a transaction**, in five steps:
  1. freeze the old memory;
  2. propose a local change;
  3. predict a withheld look under both the old and the new arrangement;
  4. check that look's pose and visibility independently;
  5. only then commit a new **surface epoch**, recording its spatial extent,
     effective time and provenance.

  Until step 5, both arrangements are pending. Each may generate test
  predictions, but neither is adopted as what the rover expects. Missing depth or an occlusion never advances an epoch. A change of
  appearance on unchanged geometry gets an appearance version of its own.
- **Physical identity is kept apart from what changed.** "The chair moved" and
  "a chair left and an identical one arrived" can predict exactly the same
  view. A look settles them only if it tells the surviving alternatives apart
  by a margin frozen in advance. Otherwise the identity stays unresolved
  ([R-WS-2](../requirements/world-state.md#r-ws-2)). Two pans from one spot are
  not the placement baseline that R-WS-2 asks for.
- **There are two routes into a tentative slot.**
  - *Novelty*: a thing where space was unknown.
  - *Arrival*: a persistent thing where space was measured free.

  An arrival's predecessor is not limited to departures the rover noticed. A
  departure it missed, one outside the window, or one across a map reset are
  possibilities too.
- **Hard constraints are never traded for a simpler story.** Two things seen at
  once are two things, and the geometry gates of
  [R-WS-8](../requirements/world-state.md#r-ws-8) stand.

## Levels of prediction

The owner asked whether prediction and error detection could work at several
levels. A chair that has moved would be caught higher up than a chair with an
unexpected feature, such as a broken leg, and pixels and patches would sit at
the bottom. The owner also asked whether existing VLM, VLA or world models
could supply a saliency signal from their internal variables.

**The levels are levels of explanation, not separate memories.** Explaining
away does not mean silencing an alarm. It means three steps:

1. change a variable at a higher level, such as where the chair stands;
2. predict the lower levels again under that change;
3. check that the error disappeared on a look that was not used to make the
   change.

Among the systems surveyed, none stacks such levels end to end on a robot. The
change detectors among them (Living Scenes, 3DGS-CD, OASIS-Map and Khronos,
with RIO for relocalising objects) work in roughly this order:

1. the robot's own motion;
2. moves, found by re-registering objects;
3. whatever remains, which becomes an arrival or a departure.

None of those surveyed looks inside a re-registered object for a changed part,
so that step is this plan's own.

| Level | What it predicts | An error here means | First year? |
|---|---|---|---|
| Surfaces | depth and dense features, per pixel, pooled over regions: the residual engine described above | a connected part of the world differs: surface evidence not yet attributed, reported flat until the object levels exist | yes |
| Object content | an object's appearance in its own frame, from its stored views | the object stands where predicted, or at a verified new placement, but part of it differs: a cushion, a throw, a drawer, a leg; this proves nothing about its physical identity | yes, from close views |
| Object placement | where each object stands | the object moved, left, arrived, or cannot be resolved | yes |
| Room | the set of placements | the room has been rearranged, given a sound background pose | as a summary of placements |
| Place and pose | which stored place this is, and where the rover is in it | wrong place, wrong pose, or an anchor fault | yes, as the gate before the levels above |
| Routine | what is usual here at this time | an event unusual for its time or place | later, from many natural days |

**The order, for every look:**

1. **The place and the pose, first.** Retrieve the place and check the
   rover's pose against the stable background before any object is scored. If
   that pose fails, nothing below is committed: the old memory stays frozen and
   the reports stay pending. Many changed objects with a sound background are
   evidence that the room was rearranged, not that the rover is lost, so they
   are never suppressed on that ground.
2. **Visibility and occlusion** are checked before any explanation is scored.
   Occlusion means the view was blocked; it is not a state of the object.
3. **Each object has two hypothesis tracks.** *Placement* is one of: the same
   place, moved, gone, new, or unresolved. *Content* is one of: unchanged,
   altered, or unresolved. A thing can be moved *and* altered.
   - One route to a move is a departure and an arrival that match: first by
     DINO features pooled over the two masks, then by XFeat with LighterGlue
     and PnP on the OAK's depth. It is not the only route, since a departure
     can be missed or fall outside the window.
   - Registration counts only if its matches are spread across the object and
     the transform they give is observable.
   - The object's views are then rendered at the proposed new pose. Within
     each track the hypotheses compete until the change transaction above
     settles them on a withheld look.
   - Fitting regions and checking regions inside the object are fixed in
     advance. The "intact" part is never chosen as whichever subset makes the
     damage disappear.
   - A poor fit means unresolved, not replaced. A moved chair and an identical
     replacement stay unresolved as to identity (R-WS-2).
4. **What is left over** is an arrival where the space had been measured free,
   novelty where it was unknown, and a departure where a surface was predicted
   but seen through.

**A change is accepted by one frozen test**, not by a judgement of "more
explained than it costs":

- The error is scored on regions fixed in advance, by a scoring rule defined
  and frozen before any acceptance session. Depth and appearance pass
  separate gates ([R-WS-8](../requirements/world-state.md#r-ws-8)).
- How references are chosen, how the transform is searched, how many
  candidates there are, and when the search stops are all frozen in advance.
- The decisive look is withheld from both fitting and candidate selection.
- The improvement must exceed the 95th percentile of the largest improvement
  the same search finds on unchanged development runs.
- No credit is given for dropping measured support or for enlarging what is
  unknown.

A moved chair with a damaged cushion therefore comes out as two events, never
hundreds. Each measurement is stored once and referenced by every level that
uses it, so it is never counted again at each level.

**Objects start as verified bundles, not as reconstructions.** A bundle is a
few RGB-D views of one object, held in one reference camera's coordinates,
with the rigid transforms between the views and with unseen surfaces marked
unknown. Building bundles is where this fails first on the rover, so building
them automatically is a step to be tested, not something reorganising memory
provides:

- record 838 holds both armchairs, and using its members as views would turn
  that fault into the prediction;
- the region finder's "objectness" is its highest score over its vocabulary,
  and the outlines it stores are clipped half-resolution masks, not whole
  objects;
- depth covers only the OAK's central field.

The first bundles are verified by hand. Masks drawn by hand mark the upper
bound of what is possible; acceptance uses the rover's own region pipeline.
Each object's normal error starts from a population figure shared across
objects, shrinking toward the object's own figure as its looks build up. Only
looks independently established as unchanged count toward that figure.
Correlated looks are discounted, the figure's uncertainty is carried, and it
is frozen while a change is pending.
Today, 318 of the store's 684 records have at most two looks with a range,
too few for a figure of their own. FOCUS (2026), which keeps such a figure per
object, does the same for sparse objects, and its filter has three states,
persisted, replaced and removed, with no moves. The two tracks here are this
plan's own extension.

**The content level reads appearance, not depth, from close views.** A 3 cm
chair leg 3 m away spans 0.57 degrees. Across the fisheye's 134 degrees that is
under 3 of the 640 pixels, a fifth of one 14-pixel DINOv2 patch. On passive
stereo it would be almost all depth edge, which the plan drops, so a missing
leg would read as unknown, not as gone.

In the 2592x1944 still that leg spans about 11 pixels. Resizing a whole
metre-wide chair to the encoder's 224 pixels shrinks the leg back to about 7,
so content is compared on local tiles of the still or on a closer look that the
object level asks for.

Those stills are separate grabs, 0.33-0.68 s after the look, and they are kept
if the rover moved less than 2 cm and turned less than half a degree. Half a
degree is about ten pixels, so each tile is re-registered locally, and checked
against the look's depth, before it is compared. None of the labelled
benchmark's pictures has a full-resolution still, so the content level needs
fresh paired evidence before it can be scored at all.

Comparing patches with their nearest neighbours in the object's own patch bank
catches new texture, such as a throw. It can miss a missing part when the
patches the gap exposes, floor and wood, match the bank: AnomalyDINO is at
chance (50.2 AUROC) on MVTec's swapped cables. A missing part needs the
comparison aligned to the object's pose.

Staging starts with cushions, throws and drawers, not legs, and the smallest
change scored, with its projected size in pixels, is frozen beforehand. The
synthetic BrokenChairs-180K set (Looking 3D, 2024) can serve as an offline
bench for the content check.

**What pretrained models supply is representations, not explanations.** The
evidence was checked against the primary sources.

- **Frozen DINO patch features compared against a memory aligned by pose** are
  the demonstrated signal.
  - SceneDiff (2025) re-projects depth, which it estimates from video, and
    pools DINOv3 features over regions.
  - It found changed objects at 43.8 average precision on one of its subsets
    and 20.9 on the other. A Qwen2.5-VL model asked what changed scored 7.1
    and 3.6.
  - The exported DINOv2 graph outputs 256 patch tokens for every region crop
    besides the summary token, and the code keeps only the summary token. If
    the engine installed on the rover outputs the same, keeping the rest
    should be cheap, but that has to be checked on the rover. Running the
    encoder densely over a whole view, over tiles, or for registration and
    search still has to be timed.
- **Latent video predictors give surprise only for continuous video**, over
  the next second or so.
  - V-JEPA's error in latent prediction told possible from impossible events
    98% of the time on IntPhys, but 62-66% on harder sets.
  - That was V-JEPA 1. On IntPhys 2, most models are at chance.
  - Movement of the camera reads as surprise.
  - None of this says anything about a still look against yesterday's. It is a
    desktop comparison at most, for things falling or a person entering.
- **Navigation World Models and Cosmos Predict** need a remote GPU.
- **VLA latents** monitor manipulation policies the rover does not run.
- **VLM attention maps** show where a question sends the model, not what is
  unexpected. The survey found no study supporting caption surprisal as a
  change detector.
- **A VLM is useful one step later**, to name or check a discrepancy the
  levels have already isolated: the hosted Qwen given the old and new crop
  and a targeted question.
  - VLMs miss most differences on their own. In VDiff-Bench (2026), 7-8B open
    models scored 9-33% on low-level changes.
  - A local Qwen3-VL-2B at INT4 takes 4.4 GB for one small image, which the
    rover's shared 7.4 GB would have to make room for beside the perception
    models.

## What it relies on first: where to point the camera

**What it replaces.** Today an aimed look centres the target using the scan's
heading check, skips pans smaller than 25 degrees, and picks a tilt of 0 or 20
degrees from the target's height ([autonomy/goals.py](../../autonomy/goals.py),
[world_state/aimed.py](../../world_state/aimed.py)).

**What changes.** For each pan and tilt that the inspection already allows, the
predictor estimates three things:

- the share of the target that will fall in the OAK's field with valid depth;
- how much the view overlaps what is already known;
- whether the view will be dark or saturated.

It begins as plain geometry, compared first against a simple centring-and-tilt
rule, and gains a learned score only if that score beats the plain one.

**What stays fixed.** It changes only where the camera points, within
inspections already authorised, and only when the rover is stationary, which is
enforced rather than assumed. The camera lock, the demonstrated pan limits, the
calibrated tilts, settling time, fresh matched depth, return to rest, STOP and
the inspection limits all stay as they are.

**After acceptance**, one inspection in four keeps today's choice, chosen at
random, so the comparison continues. That audit keeps the baseline's coverage;
it does not by itself guarantee that the rover looks where it expects nothing.

## The order of work

1. **Blind depth recall on the desktop: run 2026-10-13, and it stopped the
   plan as configured** ([the entry](../progress/2026-10-13-blind-depth-recall.md)).
   Re-projecting stored depth beat copying it unmoved, but it usefully
   predicted only 5-32% of each target's measured surface, and only 2 of 6
   targets reached the 30% needed, against 3. Passive stereo, as the rover
   configures it, is the leading suspect. Pooled over 4-degree blocks, the
   predicted and measured medians agree within 15 cm or 5% on half the blocks,
   with typical errors of 0.11-0.21 m at 2-4 m.

   **This does not gate the levels above** (the owner, 2026-10-13). The aim is
   to notice what is salient or unexpected, not to redraw the view. Each level
   needs evidence only at its own grain: a moved armchair or a new bean bag
   changes depth along its outline by far more than 0.2 m. So the next test is
   whether staged changes stand out from the noise on unchanged runs (step 1c).
   Pixel-level recall matters only for small things and close views, and step
   1b improves it.

   **Step 1c, saliency at the grain of events.** Offline, on the M5 shakedown
   and the seven controls, with their looks and depth copied read-only from the
   rover:
   - each look is compared with earlier looks from within about 1.5 m and 30
     degrees;
   - three residuals are pooled over regions, with no pixel alignment:
     - the depth median per region;
     - the novelty of each region's DINOv2 patch tokens against the
       references' patches, by nearest neighbour;
     - the lidar change watch, as it runs today;
   - scoring is per event: a staged change counts as found if a surprising
     region lies within 0.6 m of it, and false alarms are counted per
     room-control.

   It is compared with the lidar watch alone and with "seen empty", on frozen
   eligibility. If it does not beat them, appearance and depth add nothing at
   this grain, and the levels above stand on the lidar.

   **Step 1b, the depth itself, within a fixed budget.** In order:
   - how repeatable and accurate the OAK's depth is from a standstill,
     measured against surveyed flat surfaces;
   - today's stereo setting and at most three others (confidence threshold,
     decimation, output size), within the pinned depthai version;
   - one way of fusing several looks before predicting;
   - blind recall again on fresh, held-out targets, with step 1's gates: at
     least three of six targets reaching 30% useful area, and warping beating
     copying.

   For pixel-level change detection there is also a floor of 80% accuracy
   where predicted. If nothing passes within that budget, pixel-level recall
   with this sensor stops. The levels above then rest on pooled depth, the
   lidar and appearance, as step 1c tests. The standstill measurement needs the
   rover; everything else can run on stored or newly recorded looks.
2. **A larger frozen recall test**: frames and depth copied read-only from the
   rover for a sample of the 4,186 candidate pairs within 25 cm and 5 degrees
   (3,304 of them across days). Unchanged surfaces are labelled from the
   photographs, and each pair's anchor is checked as above.
3. **The first consumer**: in shadow, then the paired trials.
4. **The pose second opinion**: XFeat and LighterGlue with PnP, timed on the
   Orin.
5. **Change reports from geometry**: departures and arrivals, scored
   separately on the M5 sessions' staged changes and room-controls.
6. **Appearance**: dense features on single-surface regions, benchmarked.
   Changes of appearance are reported only once this benchmark passes.
7. **Object bundles and moves.** Verified provisional bundles, then the
   placement track. Its development step is to reproduce both regression
   failures faithfully. An offline screen of moves can run right after step 1
   without delaying the first consumer.
8. **Object content**: the content track, on close views and tiles of the
   full-resolution stills.
9. **Identity and filing**, through the transaction above.
10. **Later**:
   - routines;
   - relations within a room ("at the table");
   - how much each kind of object varies, and deformable things;
   - a derived 3D occupancy cache;
   - a lidar stability per cell for the mapper's filtered scan;
   - the body model, tested on a second day's turn trials.

   The stability layer needs a history that the change watch does not keep
   today; it holds only the current and the previous visit. A dropped return
   would be written as NaN. The slam_toolbox 2.8.5 source skips NaN readings
   when it draws the grid and when it scores a match. It still counts them in
   the matcher's normalisation, though, so masking returns lowers the match
   response and could make the matcher refuse a scan. That has to be tested on
   the installed build.

## Gates

Each gate compares the new consumer with the method it replaces, on the same
frozen evidence. It also has a floor of its own, so that a weak baseline cannot
carry it, and a coverage requirement, so that abstaining cannot pass. The
numbers are pilot gates, not statistical promises, and abstentions count as
misses.

| Consumer | Replaces | Switched on when |
|---|---|---|
| Where to point | today's centring and tilt rule | **Paired trials**: 40 inspection opportunities with both choices captured in random order, covering at least 10 target-and-station cases over five days, if both captures fit existing authority and budgets. Otherwise 120 inspections in blocked 1:1 alternation. **Win**: the predicted choice gives a usable view more often, with the 90% interval of the difference above zero. A usable view is an independently identified target surface with a usable range on it; a range on the wrong region or an occluder is a failure; 30% valid depth is a precondition, not the outcome. **Accuracy**: the predicted valid-depth share is within 0.15 of the observed, by mean absolute error. |
| Change reports | the "seen empty" test, a departure signal only, which fired on 70 of 140 aimed in-view looks and on 30 of 69 looks at labelled real objects ([2026-10-10](../progress/2026-10-10-m5-what-reports-a-change.md)) | **Recall**: departures and arrivals scored separately (changes of appearance join only after the appearance benchmark), each event once within a frozen deadline, eligibility frozen from the owner's manifest and photographs rather than from the predictor, unscorable counted as missed; at least 80% recall, and no lower than the old test's on departures. **False alarms**: none held up in at least 20 eligible room-controls (a 14% upper bound under independence), with immediate alarms reported too. The camera baseline is replayed on the same looks; the M5 control counts are the lidar change watch's and do not substitute. |
| Pose second opinion | none; a flag beside the position check | **Setup**: real looks whose capture pose and reference anchor are independently known correct, with the belief shifted on purpose by 0.3-1.5 m and by ±5, ±10 and ±20 degrees, run through the whole pipeline without access to the true answer. Which opportunities are observable is frozen independently before scoring, and abstentions count as missed. **Detection**: at least 90% over at least 40 distinct anchored opportunities, with each error band reported separately. **False flags**: none over 60 anchored correct-pose opportunities spread across days, rooms and arrangements (a bound below 4.9% if they are independent; a claim below 1% needs about 300). Intervals and the dependence between opportunities are reported. Visual-only and combined are scored separately. The shakedown is development; a shifted belief does not test errors shared by look and memory, graph deformation, wrong retrieval, gimbal zero or timing faults, or looks taken on the move. |
| Moves ([R-WS-17](../requirements/world-state.md#r-ws-17), [R-WS-18](../requirements/world-state.md#r-ws-18)) | departures and arrivals reported apart, and placement histories that do not follow a move | **Development**: both regression failures reproduced faithfully. **Acceptance**: at least 16 of 20 moved trials give one correctly paired move within both four looks and five minutes, with both placements within 0.6 m of measured truth; identity kept on at least 90% of independently distinguishable moves, with no confident wrong join; identical twins left unresolved; fewer duplicate events than flat reporting, with no lower recall of the two ends. M5's own totals keep counting two end events per move, and the paired event is counted beside them. |
| Object content | flat reporting of appearance change | At least 8 of 10 altered-only trials and 8 of 10 moved-and-altered trials report the photographed changed region within the same deadline, and the latter keep their move; no confirmed alteration on unchanged, moved-only or occlusion trials, and none held up in 20 room-controls; better alteration recall than the flat comparator with no more false reports. |
| Filing | the resolver founding from every region | On exhaustively labelled sessions: at least 80% of looks at labelled things accounted for, at least 50% correctly assigned; at least 80% of distinguishable new things correctly founded; no observed wrong join; at least half of the distinguishable moves resolved; the [two regression cases](../../experiments/perception_bench/regression_cases.json) pass. |
| Looking to separate hypotheses | today's viewpoint score | At least 10 matched trials, a ceiling of four looks, unresolved counted at the ceiling; at least 8 of 10 settled correctly, and a median saving of at least one look. Any extra looks or movement need their own authority. |
| Mapper's filtered scan | the deployed setting alone (the scan matcher charging for distance from odometry), kept on while masking is tested | Improves the reproduced furniture failure. On at least 20 independent anchors it stays within 0.3 m and 3 degrees, with a median degradation of at most 0.05 m and 1 degree elsewhere. Stationary anchors do not prove safety while moving. |
| Body, turns | a defined causal baseline | At least 40 trials on a second day, stratified by direction and angle, with heading truth independent of the gyro and predictions frozen before each command; at least 36 of 40 within 2 degrees, and better than the baseline ([R-NAV-9](../requirements/navigation.md#r-nav-9)). |

## Making the truth

- Unchanged-surface labels on the recall pairs, from the photographs.
- The scene-1 armchairs labelled from the photographs, with "unresolved"
  allowed.
- The M5 sessions' staging manifests
  ([how they are counted](../decisions/m5-sessions-are-counted-before-they-start.md)),
  with actual times, before-and-after looks, and measured positions wherever
  position is scored.
- A room visit in a survey loop counts as a room-control only when its
  unchanged state is established independently: nothing staged, and the
  photographs show nothing moved. A loop is several room visits, not one
  control. Survey loops do not supply staged changes, new identities or turn
  truth.
- For moves and content, fresh M5-style staging:
  - 10 trials in each of four conditions: unchanged, moved only, altered only,
    and moved and altered;
  - spread across at least four objects and three days;
  - 10 occlusion trials and 20 independently established room-controls.

  The alterations are harmless and reversible, and the smallest scored
  change is frozen beforehand. Acceptance runs on the rover's own region
  pipeline. Unscorable trials count as misses, and repeated looks are not
  extra trials.
- Wrong beliefs induced on independently anchored looks, so the pose second
  opinion can be tested without waiting for the rover to get lost.
- `/cmd_vel`, the gimbal's commands and the OAK gyro, logged at their own rates,
  with the camera's real exposure timestamps.
- The shutter's odometry and `map`-to-`odom` logged beside each look.
- Taped stationary positions. Fiducial tags only if the owner agrees; they
  would be surveyed, and masked out of appearance and place scoring.

## What does not change

- Nothing here grants authority over movement
  ([R-SAFE-4](../requirements/safety.md#r-safe-4)).
- Remembered or predicted geometry is never relied on to find obstacles
  ([R-NAV-8](../requirements/navigation.md#r-nav-8)); avoidance stays on the
  raw lidar.
- A doubt about the rover's position only raises a flag and proposes a refit,
  which a person confirms ([R-NAV-3](../requirements/navigation.md#r-nav-3)).
- No single look fixes an identity or a placement (R-WS-2).
- Geometry gates come before appearance (R-WS-8).

## How the first design's second reading was answered

| Codex's point on [the first-principles design](perception-from-first-principles.md#second-reading) | Answer here |
|---|---|
| 1. A filter for the mapper only, and dropped returns | NaN: skipped in the grid and in match scoring, but counted in normalisation, so it is tested on the installed build; a later stage |
| 2. The encoder drifting | one frozen encoder, versioned with its compression, crop and lens, and re-run over the store on any change; appearance comes later |
| 3. A causal predictor trained on commands | factored: the body model learns from commands before the move, and the house model is measured memory conditioned on pose |
| 4. Missing truth | made: photo labels, M5 manifests, room-controls, induced wrong beliefs, logged commands and transforms, taped positions |
| 5. The observation model | measured, inferred and unknown kept apart; visibility checked on both sides; background kept apart from change; errors left unattributed unless pinned down |
| 7. Feasibility | the first experiment can stop the work; running times are to be measured |
| 8. Smaller alternatives | the first stage is keyframe recall alone |

Point 6 corrected facts, and was taken into the first-principles design.

## Sources

In addition to those of [the first-principles design](perception-from-first-principles.md#sources):

- Potje et al., 2024, XFeat, with LighterGlue, https://github.com/verlab/accelerated_features
- Lindenberger, Sarlin & Pollefeys, 2023, LightGlue, https://github.com/cvg/LightGlue
- Siméoni et al., 2025, DINOv3, https://arxiv.org/abs/2508.10104
- Kim et al., 2025, Towards Generalizable Scene Change Detection (GeSCF), CVPR, https://arxiv.org/abs/2409.06214
- OASIS-Map, 2026, multi-session object-level change detection, https://arxiv.org/abs/2607.14899
- Berton & Masone, 2025, MegaLoc, https://arxiv.org/abs/2502.17237
- Angelopoulos & Bates, A Gentle Introduction to Conformal Prediction, https://arxiv.org/abs/2107.07511
- Levels of prediction, checked against the primary sources on 2026-10-13
  (2026 entries are preprints):
  - SceneDiff, 2025, https://arxiv.org/abs/2512.16908
  - FOCUS, 2026, https://arxiv.org/abs/2610.05639
  - OASIS-Map, 2026: moved-object association F1 0.667, and detecting that an object moved F1 0.353
  - Khronos, https://arxiv.org/abs/2402.13817
  - 3DGS-CD, https://arxiv.org/abs/2411.03706
  - RIO, https://arxiv.org/abs/1908.06109
  - Living Scenes, https://arxiv.org/abs/2312.09138
  - AnomalyDINO, https://arxiv.org/abs/2405.14529 (Cable Swap 50.2 AUROC, appendix B.1)
  - Looking 3D and BrokenChairs-180K, https://arxiv.org/abs/2406.19393
  - SplatPose, https://arxiv.org/abs/2404.06832
  - V-JEPA intuitive physics, https://arxiv.org/abs/2502.11831
  - IntPhys 2, https://arxiv.org/abs/2506.09849
  - Navigation World Models, https://arxiv.org/abs/2412.03572
  - VDiff-Bench, 2026, https://arxiv.org/abs/2609.06245
  - TensorRT-Edge-LLM Orin Nano benchmarks, https://nvidia.github.io/TensorRT-Edge-LLM/0.9.1/user_guide/performance/performance-benchmarks.html
  - SAFE (VLA failure detection), https://arxiv.org/abs/2506.09937
- slam_toolbox 2.8.5 Karto source, grid and lookup, https://raw.githubusercontent.com/SteveMacenski/slam_toolbox/2.8.5/lib/karto_sdk/include/karto_sdk/Karto.h
- slam_toolbox 2.8.5 Karto source, response normalisation, https://raw.githubusercontent.com/SteveMacenski/slam_toolbox/2.8.5/lib/karto_sdk/src/Mapper.cpp
