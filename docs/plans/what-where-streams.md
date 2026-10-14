# Prediction in what and where streams

Status: proposed 2026-10-14 at the owner's direction; nothing on the rover.
Steps 1 and 2 ran offline on the M5 shakedown the same day. Step 1 used frozen
encoders and nearest-memory prediction, and met its frozen bar narrowly
([entry](../progress/2026-10-14-what-where-streams-on-the-shakedown.md)).
Step 2 added navigation's pose, and found all four arrivals, but gone found
nothing real
([entry](../progress/2026-10-14-what-where-step-2-with-navigations-pose.md)).
Step 3 is in development.
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
picture moves every time the gimbal does; the room does not. The bearing comes
from the pose and the gimbal, as it does now, and the range from the depth
camera under the region's outline. The depth camera covers only the middle of
the fisheye's view, so most regions have no range. For things standing on the
floor, the lidar's hit at the region's bearing can supply one, which is
proposed below. A look gets a where only once the rover's place is confirmed
([R-WS-16](../requirements/world-state.md#r-ws-16)), which in a rearranged room
is often refused (below).

**The errors, per thing:**
- **New**: nothing in the house's memory looks like it.
- **Out of place**: it looks like something remembered, but not like anything
  memory holds where it now stands.
- **Gone**: memory expected it and the look does not show it. A missing
  detection is not evidence of absence. The shakedown showed that region
  detections alone cannot carry this (below). It needs the space to be seen
  empty: the lidar passing through where the thing stood, or the depth camera
  measuring beyond it.

The scores are kept apart, never fused into one
([R-WS-8](../requirements/world-state.md#r-ws-8)). Each comes with the error
vector itself, the difference between the predicted and the observed embedding.
Its direction is what the later stages learn from: a lighting change, a
different viewpoint and a cushion moved should come to point different ways.

## How memory predicts, in three stages

1. **Nearest memory (step 1, done).** The prediction for a look is the regions
   of the latest views of that place. Nothing is learned.
2. **A memory of things in their latest state.** Each remembered thing keeps
   its latest place and a few appearance exemplars. A later look updates them,
   and a look that sees the space empty retires a place, so memory says where
   things are now rather than everywhere they have been. The shakedown's misses
   came from exactly this: its references reached back days into earlier
   arrangements. Changes stay pending until a later look confirms them, as in
   [the transaction](perceiving-by-prediction.md#changes-and-identity-kept-pending).
   This is close to what one thing per object
   ([R-WS-17](../requirements/world-state.md#r-ws-17),
   [R-WS-18](../requirements/world-state.md#r-ws-18)) asks of the store, and is
   built to the same rules.
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

## What step 1 found

[The entry](../progress/2026-10-14-what-where-streams-on-the-shakedown.md) has
the numbers. In short: what found both bean bags early and cleanly; where found
one armchair's new place and separated moved from unmoved well, but from stale
references and too few confirmed poses; gone found nothing, because a missing
detection says little; and the untouched rooms held no persistent false alarm.
Three of six staged changes, where the lidar watch found five. Steps 2 to 4
answer those three weaknesses in turn.

## The order of work

1. **Done:** nearest-memory streams on the shakedown, frozen before scoring.
2. **Done: with navigation's pose from the recording**, the same code on the
   looks the store kept no pose for, and rooms judged by where a flagged thing
   stands. All four arrivals found by where alone, gone nothing real. It shows
   the where stream needs a pose in the rearranged room, and so the far-hit
   heading check or a pose from settling, before it can run on the rover.
3. **The latest-state memory and lidar ranging** for regions on the floor,
   developed on scene 1 of 2026-10-11 and the controls. With a place for most
   things, the where tolerances can shrink from what absorbs the region
   finder's noise (15 degrees, 1 m) to what a ranged place needs; on the
   shakedown the wide ones absorbed a 0.9 m move. Tested on the next
   staged M5 session
   ([sessions-that-serve-several-milestones](sessions-that-serve-several-milestones.md)),
   which is fresh evidence nobody has tuned on.
4. **Gone from free space**: a remembered thing's place passed through by the
   lidar, or measured beyond by the depth camera, in looks that expected it.
5. **The learned viewpoint predictor and spread**, trained on revisits before a
   cutoff, tested after it.
6. **On the rover, in shadow**: the streams scored at every look beside the
   lidar watch. The vectors are already computed per look; a nearest-neighbour
   search over the store's vectors costs milliseconds. Reports only, through
   the existing change log, during M5 sessions.

**Gate for each step:** frozen before scoring. It must find more staged changes
than the step before, with no more persistent false alarms in the
room-controls. A step that only matches the lidar watch is kept for what it
adds that the watch cannot see: things the lidar plane misses, and what changed
rather than only that something did.

## Risks

- **The region finder crops the same thing differently every look.** One region
  in seven found no look-alike in the latest three views of an unchanged place,
  which is why every score is graded and every flag needs persistence.
- **Memory deep enough to absorb that noise is stale.** The latest-state memory
  is the answer proposed, not yet shown.
- **A rearranged room refuses the pose.** The heading check needs 90% of the
  scan on walls. The far-hit check proposed on
  [2026-10-12](../progress/2026-10-12-heading-check-in-a-changed-room.md)
  would confirm about half of those looks.
- **Two identical armchairs.** What cannot tell them apart; only where can, so
  a swap is invisible ([R-WS-2](../requirements/world-state.md#r-ws-2)).
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
