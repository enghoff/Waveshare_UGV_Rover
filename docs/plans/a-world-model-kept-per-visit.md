# A world model kept per visit

Status: proposed 2026-10-15 at the owner's direction, after
[M5 session 4](../progress/2026-10-15-m5-session-4.md); nothing built. Step 1,
the benchmark and the deployed store's numbers on it, is live, offline, on
branch `visit-model`. It aims at
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
declares. It runs after this plan's step 3 and shares nothing with it but the
recordings.

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

Frozen here, before the new model is run on any of it.

Development uses the history up to the end of M5 session 2 (2026-10-14 13:00).
**The test is sessions 3 and 4**, run once, after development is frozen. It
passes when, on the test sessions and counted across sessions:

- every record holding labelled pictures of two different objects as ordinary
  matches fails it, as it does criterion 4;
- it keeps more staged-move trials than the deployed store, and at least half;
- the labelled objects that stayed put are each held by no more records than in
  the deployed store, counting at least one (amended 2026-10-15 before the
  test was run: the deployed store holds the front door's test pictures in no
  record, so as first written any model filing them would fail), and by two
  or fewer for at least three in four of them;
- it files at least as large a share of all the labelled pictures in the test
  sessions as the deployed store does, so that leaving pictures unfiled cannot
  pass the two conditions above (added 2026-10-15, after the deployed store
  was scored and before the new model was run).

It also reports, without a bar: the records placed in the living room, gone
objects and when the evidence came, and every move it made or held pending.

The acceptance after that is on the rover in shadow, on the next staged
sessions, with labels by someone else.

## The order of work

1. **The benchmark and the baseline.** Loaders over the store copy and the
   recordings, the labels for objects that stayed put, the deployed store's
   numbers.
2. **Tracks per visit**, scored on development by how many tracks each
   labelled object makes in a session and whether any track holds two.
3. **Matching visits to memory, and absence**, over the whole development
   history in time order.
4. **The test** on sessions 3 and 4.
5. **On the rover in shadow**: the model kept beside the deployed store from
   the live looks, reports only, the next staged sessions as its acceptance.
6. **In place of the resolver**, once accepted, with the world state's
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
- **Few trials.** About a dozen staged moves; the test sessions hold four. The
  benchmark measures splitting on unmoved objects too so that one session's
  luck does not decide it.
