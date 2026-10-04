# Spatially grounded semantic world state

Status: capture, storage, perception, placement, search and console inspection
are deployed and their rules are settled requirements. Bearing accuracy is measured
rather than required since [R-WS-10](../requirements/world-state.md#r-ws-10) retired on
2026-10-03. The live work is [one thing per object](#one-thing-per-object-and-only-that-objects-looks):
duplicates and wrong looks, in investigation, with a reviewed merge step on the rover
and nothing applied automatically.
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
| [R-WS-10](../requirements/world-state.md#r-ws-10) | `retired` | nothing: [the decision](../decisions/bearings-are-measured-not-required.md) replaced it with measured performance and a per-look claim, `stated_bearing_sigma_deg`, set from the drive of [2026-10-03](../progress/2026-10-03-moving-looks-against-the-tape.md). |
| [R-WS-12](../requirements/world-state.md#r-ws-12) | `open` | refuse entities made of bare floor, so that something choosing where to look next cannot spend distance on them |
| [R-WS-13](../requirements/world-state.md#r-ws-13) | `open` | show, for each identity-dependent action when it is proposed, that the identity evidence it accepts is good enough for what a mistake would cost ([2026-10-01](../decisions/identity-is-judged-action-by-action.md)); the proposed appearance-band/geometry remedy failed replay |
| [R-WS-17](../requirements/world-state.md#r-ws-17) | `proposed` | each object is one thing: see [one thing per object](#one-thing-per-object-and-only-that-objects-looks) |
| [R-WS-18](../requirements/world-state.md#r-ws-18) | `proposed` | each thing holds one object's looks only: the same section |

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

## One thing per object, and only that object's looks

### What we aim for

Every object the rover can tell apart is one thing
([R-WS-17](../requirements/world-state.md#r-ws-17)), and every look filed under a thing
shows that thing ([R-WS-18](../requirements/world-state.md#r-ws-18)), reached by the
rover on its own, with no person reviewing each change. Both are proposed, with
tolerances of one split object in ten and one wrong look in twenty for the owner to
agree. The autonomy plan's [milestone M5](autonomous-curiosity.md#milestone-m5-the-rover-notices-useful-changes-without-filling-the-world-with-duplicates)
depends on both: it fails on a world filling with duplicates and on any wrong confident
merge.

Today's store misses both, by the coding agent's labels of map session 67: three of 19
labelled objects are two things each, and 46 of 412 labelled looks are filed under the
wrong thing. The resolver causes both, by design: it decides each look once, when it
arrives, and never compares two things it has placed with each other again.

### What has been established

- **A single score for "is this the same object"** is a log-likelihood ratio with
  weights fitted by logistic regression, the established method. Among the things the
  resolver's tolerance admits, it puts nearly all the weight on appearance
  ([2026-10-04](../progress/2026-10-04-one-score-for-appearance-and-position.md)).
- **Changing how the resolver files looks as they arrive does not help.** Ranking by
  the score, or refusing below a threshold, trades wrong looks for split objects (same
  entry).
- **Removing single wrong looks after the fact** gains nothing over the masked-crop
  rule: about 15 of 46 wrong looks for 8 of 366 right ones
  ([2026-10-03](../progress/2026-10-03-cleaning-things-tested-on-recordings.md)), and
  the resolver would attach them straight back, because the store cannot record that a
  look is not a given thing.
- **Re-solving a whole session at once by EM** loses too many things
  ([2026-10-03](../progress/2026-10-03-whole-session-em.md)).
- **Joining duplicates by the score after the session works.** 216 more same-object
  pairs end up together, with none of different objects. Joining them while the rover is
  still looking makes things worse on every schedule tried
  ([2026-10-04](../progress/2026-10-04-merging-while-the-rover-looks.md)). A person can
  review and apply the merges now ([the runbook](../runbooks/world-state-merge.md)).
- **Rebuilding a few neighbouring things from their pooled looks** handles wrong looks
  best. With the dining chairs aside and weights fitted without the things scored, it
  keeps 7 of 37 wrong looks with their object, against 10 for single-look removal at
  the same cost of 34 of 311 right looks, and rejoins 4 of 6 duplicate halves. It has
  been run only on groups around the 25 labelled things.

### What is still to find out

Each question is answered by replay of map session 67 unless it says otherwise, with
the success predicate fixed here before the run.

1. **Does a grouping that the resolver never sees keep the gain throughout a session?**
   "Same object as" groups would sit over the resolver's things, recomputed by the merge
   rule every few minutes, while the resolver keeps working on its own things. Replay
   with the groups recomputed every five minutes and score at five evenly spaced points
   in the session. It passes if, at every point, same-object pairs together are at least
   the resolver's own count at that point, different-object pairs together are no more
   than the resolver's own, and at the end the counts match the pass after the last
   look (3,937 and 710).
2. **Is the two-boxes mechanism why merging during a session hurts?** In the replays
   that merged every five minutes, count the things founded from a region whose picture
   gave its other region to a merged thing. This is diagnosis only, with no predicate.
   It decides whether merging the store itself is worth revisiting.
3. **Does the neighbourhood rebuild hold across the whole store?** Tile the store into
   groups of neighbouring things so that each look is decided once, rebuild every
   group, and score all 25 labelled things with weights fitted without them. It passes
   if wrong looks kept are no more than single-look removal keeps at the same cost of
   right looks, no more than 10% of right looks are let go outright, and no two labelled
   objects end up in one thing.
4. **Does a look the rebuild lets go stay away?** Replay with a "not this thing"
   record written by the rebuild and consulted by the resolver. It passes if no look the
   rebuild took off its thing is later filed under that same thing again.
5. **Do the answers hold on an independently labelled drive?** On a new driven
   recording, the owner labels a random sample of at least 20 things, look by look, from
   contact sheets, before any of the above is scored against it. This is item 3 of the
   acceptance list above, and nothing here is called settled without it.

### Implementation and integration

Each step needs the answer above it to have passed. Each is replayed, passes the
offline suites, is deployed and verified over TCP 8769, and gets a progress entry.

1. **A hard geometric limit on the merge rule.** No pair is proposed whose placements
   are further apart than their error ellipses allow at 99.9%, so that appearance only
   chooses among pairs geometry accepts ([R-WS-8](../requirements/world-state.md#r-ws-8)).
   This is owed before any further merging code is deployed.
2. **Groups over things** (after question 1), with these parts:
   - a table of which things are one object, recomputed by a pass in the daemon's world
     loop on its own clock;
   - the console, the voice tools and the executive read groups where they read things
     today;
   - the old-name forwarding and the autonomy recorder's aliases, built on 2026-10-04 and
     not yet deployed, carried over to groups;
   - the automatic merge pass built the same day is not deployed.
3. **The "not this thing" record** (after question 4), with these parts:
   - a table of looks and the things they are not;
   - both of the resolver's attach paths consult it through the existing
     `association_allowed` hook;
   - the rebuild writes it, and a person can, through a control call.
4. **The neighbourhood rebuild** (after question 3), with these parts:
   - first as a reviewed step like the merge step: proposals, contact sheets, apply,
     rollback;
   - then on its own clock over the groups that changed, journalled so that each run can
     be put back;
   - unattended only once question 5 has passed.
5. **Acceptance.** The owner's labelled drive is scored against
   [R-WS-17](../requirements/world-state.md#r-ws-17) and
   [R-WS-18](../requirements/world-state.md#r-ws-18) with the tolerances agreed. Those
   requirements move to `settled` only when both hold there, and to `open` before that,
   once the tolerances are agreed.

## Earlier work

Earlier measurements and abandoned variants remain available in Git history and
the bench scripts. They are not current operating documentation, and the rules
that came out of them are the requirements listed above rather than a second
list here.
