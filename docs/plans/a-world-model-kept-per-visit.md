# A world model kept per visit

Status: proposed 2026-10-15 at the owner's direction, after
[M5 session 4](../progress/2026-10-15-m5-session-4.md). Steps 1 to 4 ran
offline the same day on branch `visit-model`. The test did not pass: one record
mixed two paintings, and the blue painting sat in two records against one.
Unmoved things now stay together, and nearly every picture is filed, but no
moved thing's record followed it
([entry](../progress/2026-10-15-a-world-model-kept-per-visit-tested.md)).
Version 2 adds a likeness learned from the rover's own pictures (0.97 against
0.90 at telling one object from two, no labels), a colour veto and records
joined into paths
([entry](../progress/2026-10-15-a-likeness-learned-from-the-rovers-own-pictures.md)).
Over the twelve staged moves it cannot be told from version 1, because six of
them are the two armchairs, which nothing tells apart. Its test is a staged
session of several distinguishable things (step 6), which needs the rover and
the owner. Since 2026-10-16 the model is pass/fail on honesty alone, and the
rest is measured against the deployed store without a bar
([agreed](../decisions/the-world-model-is-pass-fail-on-honesty-alone.md)).
Nothing is on the rover. It aims at
[R-WS-17](../requirements/world-state.md#r-ws-17) and
[R-WS-18](../requirements/world-state.md#r-ws-18), and keeps
[R-WS-2](../requirements/world-state.md#r-ws-2) and
[R-WS-8](../requirements/world-state.md#r-ws-8).

## Why

Moving an armchair, bringing a bean bag in from another room and pushing a
cabinet along a wall are the ordinary life of a home. The world state cannot
keep up with them, and the M5 sessions show where it breaks:

- **Identity is decided one picture at a time**, from a bearing that misses its
  object by 7 degrees at the 90th percentile. One object becomes many records:
  on 2026-10-15 about 480 placed records stood in and around the living room
  for a few dozen objects, and a dozen records looked as much like the armchair
  with the throw as its own.
- **No record ever stops being where it was.** The lidar's change watch sees a
  place go empty, and the records never hear of it. A record of where something
  stood on 2026-10-03 competes with the one where it stands now; in session 4
  such a record, holding pictures of both armchairs, is what stopped the
  record-moving step.
- **The fixes were patches on that.** Lidar ranges, the move step and the
  what/where events each answered one failure and left the representation as
  it was.

The designs of 2026-10-12 already say what to do instead
([perception that keeps what it learns](perception-that-keeps-what-it-learns.md),
[first principles](perception-from-first-principles.md),
[perceiving by prediction](perceiving-by-prediction.md)): absence measured, not
inferred from a missed detection; an object as a path through places; identity
held pending where it cannot be told. This plan builds the smallest version of
that which can be scored against the deployed store on looks already recorded,
and keeps the owner's direction of 2026-10-14 that change is judged in what and
where, not in rendered pictures.

## What changes

The region finder, the encoders, the looks and the lidar change watch stay as
they are. What changes is what is built from them:

1. **One track per object per visit.** Looks are taken about a second apart, so
   the same armchair in consecutive pictures is a short step in place and
   barely a step in appearance. Regions are joined into tracks while they are
   seen, by place (a range from depth or the lidar, or bearings from several
   viewpoints met at one point) and appearance together, one region of any
   picture to a track. A track, not a picture, is what meets memory.
2. **A visit is matched against memory all at once.** When a track closes, its
   place, size and appearance are compared with every remembered object in one
   assignment, where "here as before", "moved from where it was seen gone",
   "new" and "one of several that look alike" are all answers.
3. **Absence is measured.** A remembered object whose place the lidar or the
   depth camera has since seen through, from close enough and with nothing in
   the way, is gone from there. It keeps its history and stops competing for
   the pictures of the place. A move is then a gone object matched by a new
   arrival.
4. **An object has a current place and a history**, not one placement fitted
   to every look it ever had.

Localising on what does not move is the second half
([first principles, the first test on recordings](perception-from-first-principles.md#what-could-be-tried-on-the-recordings-now)): the
mapper's scans weighted by how stable each cell has been, replayed through
[`experiments/nav_replay`](../../experiments/nav_replay/README.md) on the
shakedown and session 3, against the frozen criteria that test already
declares. It shares nothing with this plan but the recordings. It ran on
2026-10-15 and did not pass: with moving cells taken out, the original
matcher setting no longer walks off, but it ties the deployed strict setting
on the shakedown and does not fix session 3's 08:07 miss
([entry](../progress/2026-10-15-localising-on-stable-cells.md)).

## The benchmark

Everything since 2026-10-03 is in one map frame (map session 67): about 45,000
regions in 6,300 pictures, with drive recordings (lidar, odometry, the
mapper's corrections) for the shakedown, the controls and scene 1 of
2026-10-10 and 11, and M5 sessions 1 to 4.

- **Moves.** The staged moves of labelled things across the shakedown and
  sessions 1 to 4: the two black armchairs (told apart by the throw), the
  cabinet and the red bean bag, counted across sessions as
  `experiments/identity_moves/count_all.py --across` counts them. A thing
  carried in from a room the rover never enters, as the red bean bag was in
  session 4, is a trial: it is the same object the store remembered.
- **Objects that stay put.** Before the new model is first scored, at least
  eight living-room objects that were never staged are labelled across the
  history, so that splitting is measured where nothing moved.
- **Labels** for development are made by the coding agent from photographs.
  Acceptance needs labels by someone else (R-WS-17's own rule).
- **The baseline** is the deployed store's own filing of the same looks, scored
  by the same code.

## What counts

Agreed with the owner on 2026-10-16
([the decision](../decisions/the-world-model-is-pass-fail-on-honesty-alone.md)).

- **Pass or fail: honesty.** No record holds labelled pictures of two things
  that can be told apart, every filing counted, as criterion 4 counts. The
  dining chairs and the two black armchairs are left out; for them the answer
  is "one of the two".
- **Measured against the deployed store on the same pictures, without a bar:**
  moves followed by the strict reading (the record holding most of the thing
  afterwards held most of it before), the records holding each thing that
  stayed put, and the share of labelled pictures filed.
- **It takes the resolver's place** once, in shadow on the rover, it has made
  no wrong merge, follows clearly more moves than the deployed store, and is
  no worse on the other two. Moves need about three to five staged sessions;
  honesty is shown slowly, by the shadow run's ordinary drives.

It also reports: the records placed in the living room, gone objects and when
the evidence came, and every move it made or held pending.

Version 1 was tested once, on sessions 3 and 4, against five conditions frozen before
it ran, and did not pass
([entry](../progress/2026-10-15-a-world-model-kept-per-visit-tested.md)).
Those conditions are retired.

## The order of work

1. **Done: the benchmark and the baseline.**
2. **Done: clusters per visit.**
3. **Done: matching visits to memory, and absence.**
4. **Done, not passed: the test** on sessions 3 and 4.
5. **Built: version 2, an object is a path, and colour and a learned
   likeness tell look-alikes apart.**
   - *Paths.* Records that were never present at the same time, look alike,
     and of which one was seen gone before the other appeared are one object
     at two places. The arrival in the test found an older record of itself
     every time; this joins them instead of leaving two.
   - *Colour.* A colour description of each region inside its outline, made
     on the rover from the stored pictures, and calibrated on the labels
     beside the image vectors. The red and the black bean bag scored 0.74 and
     0.73 against one arrival.
   - *A learned likeness* (`metric.py`): KISSME on pairs the pictures give
     for free, two regions of one picture and one visit's clusters.
   - Development used all five labelled sessions, since the test had been
     seen.
6. **Version 2's test**, frozen before it runs, on a staged session built to
   decide it: several distinguishable things, each seen at its place before
   the staging. Proposed for session 5:
   - Before the session the owner sets out in the living room five movable
     things that can be told apart: the red and the black bean bag, the box,
     the stool and the step ladder from session 2. Loop A sees each where it
     stands.
   - In the wait the owner moves three of them, at least 1 m, and leaves two
     where they are. The armchairs and the cabinet are not touched.
   - Labels are made from the session's photographs before the model is run
     on it, and the learned likeness is the one trained before the session.
   - Scored as *What counts* says, version 1 beside it.
   - Before it, version 2 is made to decline its one mixed record in
     development (two pictures of the blue painting in the snowy-tree
     painting's record, on the shakedown) and frozen again.
   - The two armchairs are left out and counted separately
     ([agreed 2026-10-16](../decisions/the-two-armchairs-cannot-be-told-apart.md)).
7. **On the rover in shadow**: the model kept beside the deployed store from
   the live looks, reports only. The next staged sessions and every ordinary
   drive are its acceptance.
8. **In place of the resolver**, once accepted, with the world state's
   consumers (search, the executive's looks, the console) moved over one at a
   time.

**Paused meanwhile**: tuning the resolver and the record-moving step, and staged
sessions for them. The shadow runner keeps acting on moved events as deployed;
it has acted on none in sessions 3 and 4.

Later steps are proposals: each starts only on the evidence of the one before.

## Risks

- **Pose.** Every place depends on it, and it went 0.5 m wrong in session 3
  among moved furniture. Looks in a known bad-pose window are left out of
  scoring, as before, and localising on stable structure is the remedy.
- **Things above the lidar's plane** (on tables, on walls) have only the depth
  camera's word for absence, and it sees 65 degrees of the room and sleeps
  when the wheels stop. They may stay "not seen lately" rather than gone.
- **Two armchairs alike.** A move between them may stay unresolved; that is
  allowed (R-WS-2) and counted as not kept, never as kept.
- **Few trials.** Twelve staged moves so far, six of them the two armchairs; a
  session gives about three more. Moves are measured, not barred, and the
  benchmark measures splitting on unmoved objects too so that one session's
  luck does not decide it.
