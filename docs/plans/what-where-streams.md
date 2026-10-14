# Prediction in what and where streams

Status: proposed 2026-10-14 at the owner's direction; nothing on the rover.
Steps 1 and 2 ran offline on the M5 shakedown the same day. Step 1 used frozen
encoders and nearest-memory prediction, and met its frozen bar narrowly
([entry](../progress/2026-10-14-what-where-streams-on-the-shakedown.md)).
Step 2 added navigation's pose, and found all four arrivals, but gone found
nothing real
([entry](../progress/2026-10-14-what-where-step-2-with-navigations-pose.md)).
Step 3, chosen by the owner the same day, takes where and gone from the
lidar's change watch and joins them to the camera's what. On recordings
already seen it names what changed but finds nothing the watch misses on the
shakedown; its test, the next staged M5 session, is scripted
([entry](../progress/2026-10-14-what-where-step-3-events.md)).
This plan replaces the house model of
[perceiving by prediction](perceiving-by-prediction.md) as the place where
prediction and observation are compared. That plan's rules for pending changes,
not filling in, settling and gates still hold, and so do its levels.

## In short

The rover will not compare what it expects with what it sees as pictures or
depth maps. Rendering a stored view into a new viewpoint and differencing it is
brittle: lens, stereo noise, lighting and a few centimetres of pose all show up
as error. The [depth recall test](../progress/2026-10-13-blind-depth-recall.md)
predicted too little of a nearby view to go on. The owner's direction on
2026-10-14 was to move the comparison up, into progressively more abstract
representations, with what and where carried separately.

So a look is parsed into a small set of things. Each thing has a **what**, a
vector from a frozen general-purpose encoder, and a **where**, its place on the
map. Memory predicts both from what the rover has seen before. The errors live
in those two spaces, and they are not meant to be read by a person. They are
calibrated against revisits where nothing changed, and they serve three
purposes: flagging a change, choosing where to look next, and deciding what to
learn.

## The two streams

**Parsing.** The region finder the rover already runs (YOLOE) gives each region
an outline. It is good at finding things and poor at telling them apart, because
detection training teaches a network to ignore the differences between one
chair and another. So it supplies the parse and nothing else.

**What.** A DINOv2 vector of the region with its background blanked, which the
store already keeps for every region (and SigLIP2 beside it for words). These
encoders keep far more of how a particular object looks than detection features
do, and they change gradually with viewpoint. They stay frozen: training an
encoder and its predictor together lets the errors collapse, and hierarchies
trained end to end tend to leave all the work to the bottom layer (the critique
of PredNet below).

**Where.** A thing's place in map coordinates, never in the picture. The
picture moves every time the gimbal does; the room does not. Since 2026-10-14
where comes from geometry, not from a latent: the lidar change watch the rover
already runs keeps each place's latest state, solid or seen through, and says
where something has arrived and where something has gone. A latent where,
matching a region's appearance against remembered things near its place, was
tried on the 11 October recordings and could not tell moved armchairs from
unmoved ones ([step 3's development](../progress/2026-10-14-what-where-step-3-events.md)).
The camera's regions are tied to those places by the depth camera's range
under the outline or, for things on the floor, the lidar's return across the
region's width. A region gets a bearing only once the rover's place is
confirmed ([R-WS-16](../requirements/world-state.md#r-ws-16)), which in a
rearranged room is often refused (below).

**What is reported:**
- **New thing**: the regions at an arrival, or at a place the lidar calls small
  or cannot see, are unlike anything in the house's memory, in two looks.
- **Moved**: a departure and an arrival whose regions look alike. Which of two
  identical things moved cannot be told
  ([R-WS-2](../requirements/world-state.md#r-ws-2)).
- **Arrived** and **gone**: the watch's other furniture-sized changes, named by
  the regions seen or remembered there. A missing detection is never evidence
  of absence; gone needs the space seen empty.

The two kinds of evidence are kept apart, never fused into one score
([R-WS-8](../requirements/world-state.md#r-ws-8)). The what stream's error is a
vector, the difference between the predicted and the observed embedding, and
its direction is what the learned stages are for: a lighting change, a
different viewpoint and a cushion moved should come to point different ways.

## How memory predicts, in three stages

1. **Nearest memory (step 1, done).** The prediction for a look is the regions
   of the latest views of that place. Nothing is learned.
2. **The room's latest state (step 3, built).** Geometry is the change watch's
   latest state of every place; appearance is the newest regions remembered at
   a place. Memory says where things are now rather than everywhere they have
   been, which is what step 1's stale references lacked. Changes stay pending
   until a later look confirms them, as in
   [the transaction](perceiving-by-prediction.md#changes-and-identity-kept-pending).
   Remembering each object as one thing
   ([R-WS-17](../requirements/world-state.md#r-ws-17),
   [R-WS-18](../requirements/world-state.md#r-ws-18)) is not needed for this,
   and is not attempted.
3. **Learned predictors.** Small networks trained on the rover's own unchanged
   revisits, with the frozen encoder's vectors as targets, as DINO-WM plans in
   DINOv2's space. The first predicts how a thing's vector should shift seen
   from a new angle. The second predicts the spread of the error for a thing
   and a context, which is calibration. Later, a third predicts where things
   tend to be (routines). The rover has thousands of revisits; the store holds
   over 40,000 regions.

Above things, the same pattern repeats: a place embedding for which room and
viewpoint this is, and a room level for the set of placements. These follow
the levels of the earlier plan.

## Calibration, and what keeps it honest

An error in an embedding space has no units. It means something only against
the spread the same score shows where nothing changed, for that kind of thing,
range and viewpoint. Thresholds are percentiles of unchanged controls, frozen
before a test. A flag counts only when it persists across looks, which guards
against the "noisy TV": an agent drawn to whatever is unpredictable rather than
whatever changed. Spreads freeze while changes are pending, so the rover cannot
learn its own mistakes as normal variation. Expectation never fills in what was
not seen.

## What steps 1 to 3 found

The entries have the numbers. What, the frozen encoder's novelty, found both
bean bags early and cleanly, and scene 1's tipped stool, which the lidar called
small. A latent where found the moved armchairs only with navigation's pose and
only weakly, held back by stale memory and rough places. Gone from region
detections found nothing. Step 3 joins the watch's geometry to the camera's
what and reports changes as moved or new: on the shakedown the same five of
six as the watch, three of its four named right, on evidence already seen.

## The order of work

1. **Done:** nearest-memory streams on the shakedown, frozen before scoring.
2. **Done: with navigation's pose from the recording**, the same code on the
   looks the store kept no pose for, and rooms judged by where a flagged thing
   stands. All four arrivals found by where alone, gone nothing real. It shows
   the where stream needs a pose in the rearranged room, and so the far-hit
   heading check or a pose from settling, before it can run on the rover.
3. **Built: the watch's geometry joined to the camera's what**, developed on
   the controls and scene 1 of 2026-10-11 and checked on the shakedown. Its
   test is fixed in the branch's `step3.md` for the next staged M5 session
   ([sessions-that-serve-several-milestones](sessions-that-serve-several-milestones.md)),
   which is fresh evidence nobody has tuned on, against the watch alone on the
   same scans. That session's recording and world store are all it needs.
4. **The learned viewpoint predictor and spread**, trained on revisits before a
   cutoff, tested after it.
5. **On the rover, in shadow**: step 3's events beside the watch's own log,
   reports only, during M5 sessions. The vectors are already computed per look
   and the watch already runs, so the cost is joining them. It needs a pose in
   the rearranged room first: the far-hit heading check or a pose from
   settling.

**Gate for each step:** frozen before scoring. Against the watch alone it must
find every staged change the watch finds, add one the watch misses or name
four in five of its events right, and raise at most one more false alarm in
the room-controls. What it adds is what the watch cannot give: things the
lidar plane misses, and what changed rather than only that something did.

## Risks

- **The region finder crops the same thing differently every look.** One region
  in seven found no look-alike in the latest three views of an unchanged place,
  which is why every score is graded and every flag needs persistence.
- **Memory deep enough to absorb that noise is stale.** For geometry the
  watch's latest state answers it; for appearance only the newest regions at a
  place are used.
- **A rearranged room refuses the pose.** The heading check needs 90% of the
  scan on walls. The far-hit check proposed on
  [2026-10-12](../progress/2026-10-12-heading-check-in-a-changed-room.md)
  would confirm about half of those looks.
- **Two identical armchairs.** What cannot tell them apart, so a move between
  them is reported without saying which went where, and a swap is invisible
  ([R-WS-2](../requirements/world-state.md#r-ws-2)).
- **Camera-only new things bring their own false alarms**: one in seven
  controls on development, a bright patch of floor.
- **People.** A person is new to what every time. Persistence and the region
  floor keep a passer-by from being a change; a person who stays is reported
  as one.

## Sources

- Object-centric scenes with separate what, where and presence codes per slot:
  AIR, Eslami et al. 2016 ([arXiv 1603.08575](https://arxiv.org/abs/1603.08575));
  SPAIR, Crawford and Pineau, AAAI 2019; SPACE, Lin et al., ICLR 2020
  ([arXiv 2001.02407](https://arxiv.org/abs/2001.02407)).
- Frozen encoders as targets for prediction: DINOv2, Oquab et al. 2023
  ([arXiv 2304.07193](https://arxiv.org/abs/2304.07193)); DINO-WM, Zhou et al.
  2024 ([arXiv 2411.04983](https://arxiv.org/abs/2411.04983)); I-JEPA, Assran
  et al. 2023 ([arXiv 2301.08243](https://arxiv.org/abs/2301.08243)).
- Hierarchies trained end to end: PredNet, Lotter et al. 2016
  ([arXiv 1605.08104](https://arxiv.org/abs/1605.08104)), and Rane et al.,
  "PredNet and predictive coding: a critical review", ICMR 2020.
- Prediction error driving action: curiosity in a learned feature space, Pathak
  et al. 2017 ([arXiv 1705.05363](https://arxiv.org/abs/1705.05363)); the noisy
  TV, Burda et al. 2018 ([arXiv 1808.04355](https://arxiv.org/abs/1808.04355)).
- What and where in detectors: YOLOX's decoupled head, Ge et al. 2021
  ([arXiv 2107.08430](https://arxiv.org/abs/2107.08430)).
