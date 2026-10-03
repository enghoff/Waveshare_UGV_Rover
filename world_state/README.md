# Semantic world state

This component records what the rover has seen, where it saw it, and the images
behind those observations. ROS owns the map, pose and routes. World state has no
authority over driving.

The rover does not assign names from a fixed vocabulary. Earlier vision-language
models produced unstable names and unreliable re-identification. Current
perception finds regions with YOLOE and stores DINOv2 and SigLIP2 vectors.
Identity comes primarily from measured geometry; text is used only to search
stored images.

## Runtime

`perception_server.py` listens on loopback port 8776. It prefers TensorRT engines
built for the Orin and falls back to CPU ONNX Runtime when those engines are not
available. Each observation records the backend because vectors from the two
backends are not comparable.

`rover_daemon/rover_world.py` owns capture and background scheduling. It records
through the gimbal camera by default. The OAK rides the gimbal's rail beside it,
looking the same way, so any region in the middle of the picture -- the OAK's 65
by 40 degrees inside the fisheye's 130 by 96 -- can carry a range, wherever the
gimbal points. Where the OAK sits relative to the gimbal camera (`oak.MOUNT`) is
one fixed transform that turns with the platform; it was measured on 2026-09-30
from matched features at fifteen gimbal positions, which agreed to a tenth of a
degree ([the measurement](../docs/progress/2026-09-30-oak-on-the-gimbal.md)).
The gimbal camera's own position relative to the SLAM pose remains unmeasured.

**A region's range is read from the depth under its own outline**
([outline.py](outline.py)). The region finder's mask, cut to the region's box, is
carried into one depth map fetched for the whole look, and the depth service's own
nearest-surface statistic is taken over those pixels. The box is read the same way only
where the outline leaves too few depth pixels; the depth service reads the box itself
only when no depth map can be fetched. Each observation keeps the outline
(`outline_blob`) and says which reading its range is (`range_from`), and the depth map
kept beside the frame is the one the ranges were read from. Before 2026-10-02 every range
was the service's box, which is a chair whenever a chair stands in front of the painting:
[the replay](../docs/progress/2026-10-02-ranging-from-the-outline.md) put the worst
placement among the taped objects at 0.38 m instead of 0.87. A store built the old way is
read again and rebuilt by `world_state_rebuild` ([rebuild.py](rebuild.py),
[the runbook](../docs/runbooks/world-state-rebuild.md)).

**The depth map read is the one taken nearest the picture**, not the newest when the
look gets round to asking: the depth service keeps three seconds of frames and answers
`/depth.raw?at=<shutter>`. If the rover was turning, each region is first turned by
how far it turned between the two, from the turn rate across the shutter bracket
(`outline.turned`). Such a range is dropped for turning only when the turn is too
fast for the shutter's own unmeasured moment (`inspection_ranges.SHUTTER_UNKNOWN_S`,
33 degrees a second against the 1-degree limit). A range read off the newest frame,
from a service without the history, keeps the old rule, which on 2026-10-02 dropped
347 regions' ranges in a six-minute drive. The depth map kept says when it was taken
and how far that was from the picture (`taken_at`, `off_s`).

Until 2026-09-30 the OAK was bolted to the chassis, and its lens as published by
`oak_depth` was 9.6% short in focal length. Depth maps saved before then are read
through the bracket they were taken on (`oak.CHASSIS_MOUNT`, chosen by
`oak.mount_at`); ranges stored on observations before then were attached through
both of those errors.

Runtime data lives outside the deploy tree:

```text
~/.ugv/world/world.db
~/.ugv/world/frames/*.jpg
```

The database grows through additive migrations in `schema.py`. Historical
columns remain readable even when the current pipeline no longer writes them.

It survives a reboot, because the navigation stack keeps its pose graph and the
coordinates every row is measured in therefore still mean what they meant. Three
things move it:

- **Clearing the map clears the world state with it.** One button does both, and
  the rover does both: `clear_map` empties the store and re-points it at the new
  map before it answers, so the reply says what went. Everything the store holds
  is a position in the map's frame or a bearing from a pose in it, so what
  survived a map clear was a list of things with nowhere to be. The console used
  to make the world's half of the call itself, guarded by a flag its world panel
  sets and nothing else does; on 2026-09-06 a map cleared with that panel shut
  left 423 things behind, measured against a map that had gone, and said nothing.
- **A map that changed without being cleared starts a new map session and keeps
  the record.** A restore that failed, or a graph built from scratch, gives the
  navigation stack a different map identity; `follow_map` notices within seconds
  and moves the session. The rows stay, shown as measured against a map that has
  gone rather than drawn in this room.
- **The things an older map stranded can be carried onto this one in a block**,
  which is what clears a backlog: two maps of one room differ by a turn and a
  shift, and the things that appear in both are enough to find it. See
  `reanchor.py`, which is run by hand rather than on a schedule.
- **A thing whose map was replaced is recognised when it is seen again, rather
  than met as a stranger.** The coordinates expired; the crops did not. So the
  first crossing in the new map that plainly looks like something the rover
  already owns takes that thing's identity back, with its history intact --
  `resolve._adopt`, gated at `RECOGNISED` and refused where two known things look
  equally like it. Until this existed nothing could ever re-place an orphaned
  thing, because the resolver considers only what is placed in the map it is
  working in: replayed across the map change of 2026-09-06, 272 things came out
  of which 99 could be driven to and none were still what the rover had spent the
  previous day learning.
- **`world_state_clear` on its own empties the store and leaves the map alone**,
  which is what a repeatable experiment needs.
- **The things of the current map session can be re-solved all at once, and put
  back.** `world_state_consolidate` hands every look of the session to the
  expectation-maximisation in [consolidate.py](consolidate.py), starting from the
  things the resolver holds: it joins halves of one thing the resolver keeps apart,
  such as the door founded on a picture with a chair in front of it, and it also
  loses real things and re-splits neighbours that share pictures, which is why it
  is a person's act and not a schedule. Applying it journals every thing and look it
  changes, so `{"rollback": true}` restores those exactly and leaves looks recorded
  since where the resolver put them. Things keep their numbers; each result carries
  the name most of its looks had. See
  [the runbook](../docs/runbooks/world-state-consolidate.md).

The clear is refused while a look is in flight, after waiting `CLEAR_WAIT_S` for
it. The console reports that as `not cleared` with the reason beside it, and the
session moves on regardless, so rows that survived a refusal are marked as
belonging to the map that has gone.

## How an observation becomes an entity

An observation keeps its frame, region, capture time, camera, pose, bearing,
elevation, uncertainty, optional OAK range, and appearance vectors. It stays
unplaced until the resolver has enough independent evidence.

**Five conditions take the direction off a look while keeping everything else it
measured.** They are all the same shape -- the picture, the regions and the
vectors are written down, and what is withheld is the one thing that was not
measured well enough to keep:

- no pose, no map identity, or a pose the navigator does not trust;
- a pose in a map the rover has not been confirmed to be placed in, which the
  navigator answers separately from whether a pose exists. A restore whose
  anchor landed somewhere else looks perfectly healthy from here, and on
  2026-09-07 it produced 34 looks from a heading 152.5 degrees out. Confirming
  the rover afterwards does not make those bearings true and nothing back-fills
  them. See `rover_daemon/rover_world.py` and
  [R-WS-16](../docs/requirements/world-state.md#r-ws-16);
- more travel during the shutter bracket than `MOVED_WHILE_LOOKING_M`, or a turn
  that widens the bearing past `MAX_BEARING_SIGMA_DEG`;
- a commanded pan outside `inspector.DEMONSTRATED_PAN_DEG`, which is the range
  the gimbal's pan campaign actually validated. Past it the servo's gain error
  is unmeasured rather than merely larger, and the store holds looks taken at
  pan 145.
- a scan that fits the map nowhere near where the rover believes it is
  ([headingcheck.py](headingcheck.py)), which is what a carried rover looks like
  ([R-WS-16](../docs/requirements/world-state.md#r-ws-16)). Turning on the spot
  leaves the rover's heading about 7% of each turn out, so a still look asks the
  navigator where one scan says the rover is. That is read-only, about a tenth of
  a second, and never holds a move (`ros_nav` `measure_pose`). The look keeps its
  pose if the scan agrees, takes the scan's pose if the scan confidently
  disagrees, and gets no direction if the search fits nowhere; nor does any look
  after it until a search fits again. **A look taken while moving is checked
  too**: the navigator matches the scan from where the rover was half way through
  its sweep, and refuses a sweep turned faster than 30 degrees a second; a moving
  search that fits nowhere is blamed on the motion and withholds nothing. A
  moving look whose check was refused takes the last check's correction while
  that check is fresh: under 15 degrees of turning since, and under half a metre
  of travel if the check had to correct. **Any other look takes the heading the
  navigator believes**, as every look did before 2026-10-01. Withholding those instead left the driven run of 2026-10-02 with
  17 of 906 regions given a direction.
  The search starts from the last correction found, because the drift builds
  steadily. The check and any correction are written beside the pose as
  `checked`, so a pose without it is the navigator's own. Against a tape, a
  heading believed 43 degrees out was stored within 3.5
  ([the check on the rover](../docs/progress/2026-10-01-photo-heading-check.md)). See
  [the measurement](../docs/progress/2026-10-01-heading-after-turning.md) and
  [R-WS-10](../docs/requirements/world-state.md#r-ws-10).

**One condition widens the bearing instead of withholding it**: an angle reached
from the descending side of the servo's backlash, or by a gimbal that has not
moved since the daemon started. That error is measured -- 1.19 to 2.23 degrees
between opposite approaches -- so it is carried as `bearing_sigma_deg` and spent
by `locate`, the same treatment a look taken while turning gets. Refusing these
would cost every bearing taken while tracking a face, which moves the gimbal both
ways by its nature. `Rover.centre_gimbal` undershoots and comes back up so that
rest, where nearly every look is taken from, is the approach the calibration
measured.

The resolver:

1. rejects evidence from another map session, while still offering a new
   crossing to the things that map left behind;
2. crosses bearings taken from separated viewpoints with enough parallax;
3. checks map visibility, elevation and any measured ranges;
4. uses appearance to reject or choose among geometrically valid candidates;
5. leaves ambiguous observations pending.

A single viewpoint cannot locate a thing. Several objects on the same lines of
sight can still form a false crossing when ranges are absent. Range-assisted
association was validated on the driven run of 2026-09-07 and it does help: of
100 things bearings alone placed, 11 had no counterpart once the ranges were
carried, and the two that were checked by eye were plainly false -- a pool of
blown-out floor placed below the floor, and a blue case pooled with a red wooden
surface near the ceiling. It refuses more than it invents. See
[`docs/progress/2026-09-07-m0-semantic-world-state.md`](../docs/progress/2026-09-07-m0-semantic-world-state.md),
which also says why the ranges on that run are not trustworthy as ranges *to*
anything: the mount constant the boxes were placed through was 6.2 degrees out at
the time. It was re-measured and replaced on 2026-09-07 -- see
[`docs/progress/2026-09-07-p0-oak-mount.md`](../docs/progress/2026-09-07-p0-oak-mount.md)
-- so the depth attribution wants checking again before it is believed. The OAK's
lens was also 9.6% short in focal length until 2026-09-30, which put a box three
degrees off at the edge of its picture; the ranges on those runs carry that too.

## What to expect from it

Measured against the owner's tape on the drives of 2026-10-01, 10-02 and 10-03. These
are descriptions, not pass marks ([the decision](../docs/decisions/p0-measures-the-hardware.md)),
and since 2026-10-03 there is no accuracy requirement on a bearing either
([the decision](../docs/decisions/bearings-are-measured-not-required.md)): what is
required is that the rover claims what it has been measured to deliver.
Where the rover claims less error than this, the claim is the fault; see
[the measurement](../docs/progress/2026-10-02-what-to-expect-from-the-hardware.md).

| What | Typical error | Worst seen | What the rover claims |
|---|---|---|---|
| direction of a still look | 1.0-1.5 deg | 95% within 5.7 deg | `stated_bearing_sigma_deg` 3.0 since 2026-10-03; 1.5 before, too little |
| direction of a moving look | 3.2 deg | 95% within 10.3 deg, worst 23 | `stated_bearing_sigma_deg` 5.0 since 2026-10-03; 1.5 before, too little |
| elevation, with the 4.9 deg bias at the rest tilt taken out (`locate.elevation_of`) | 1.4-1.8 deg | under 4 deg | 2.2 deg |
| range of a still look, in plain view | 0.07-0.17 m, about a tenth of the distance | 0.36 m | 0.07-0.13 m: too little |
| range through something in front | the thing in front | 1.3 m short | the same as in plain view |
| a placement from two or more viewpoints, in plain view | 0.18 m | 0.55 m | about right: on 2026-10-03, 2 of 6 inside the claim and all 6 inside twice it |
| a placement from one viewpoint | 0.55 m | 1.77 m | `stated_uncertainty_m` 1.0 m since 2026-10-02 (it used to claim 0.05-0.24 m) |
| a thing behind other things | 0.88 m | 1.12 m | too little in 4 of 5 |
| a height, from replay | 0.08-0.29 m | 0.29 m | inside the claim 7 of 7 |
| a small object on the floor | often never becomes a thing of its own | | |

**What a placement claims is `stated_uncertainty_m`, not `uncertainty_m`.** The
second is also the tolerance the resolver joins looks by, and widening it to the
measured error loosens that joining: on the labelled drive of 2026-09-08, redrawn
through today's lens, a 0.5 m floor on one-look placements, or a 2.2 degree
bearing sigma, took merges of different objects from 8 to 13. So `store.place`
writes the claim beside it (`locate.stated_uncertainty`). One-look placements
claim 1.0 m, and placements from two or more viewpoints, which measured honest,
claim their own figure. Replayed on the three taped drives, every target
placement then lies within twice its claim, 14 of 14. The hypothesis generator
reads the claim.

**What a look claims for its bearing is `stated_bearing_sigma_deg`, not the width it
is matched with.** The resolver crosses and joins bearings at `locate.sigma_of`,
floored at the 1.5 degree calibration, and widening that to the measured error
merges different objects for the same reason. So a look read back from the store
carries its claim beside it (`locate.stated_bearing_sigma`): 3.0 degrees from a
standstill and 5.0 when the capture bracket says it travelled or turned, or its own
width where that is wider. On the drive of 2026-10-03, which set those numbers, 93%
of looks at the targets lie within twice their claim, against 70% at the matching
width; on the taped drives of 2026-10-01 and 10-02, 80-90% of still looks and 85% of
moving ones. Where a thing sits in the picture matters as well: 1.6 degrees within
10 of the middle, 3 to 4 further out, which the claim does not yet use. See
[the drive](../docs/progress/2026-10-03-moving-looks-against-the-tape.md).

The range rows were measured with the box. Read under the outline, the range through
something in front came to the painting rather than the chair in front of it on the one
taped case (2.45 m against a taped 2.54, where the box read 1.22), and placements from
one viewpoint improved with it; that is replayed, not yet measured on a drive.

The per-look range figures marked too little are left as recorded. On 2026-10-03
every range that came back was within 0.5 m, but the cabinet read 0.2 to 0.4 m long
each time, probably into its open shelves, against claims under 0.1 m. Range errors
depend on the target as much as on the look. They are
small on the compact toolbox; larger on the cabinet and the painting above it,
where the depth sample can land anywhere on a large object; and partly in the
taped reference points. They are not yet a sound basis for a stricter check.

## Offline experiment: bounded association and revision

`bench_incremental.py` compares the production resolver with an experimental
`IncrementalResolver`. The daemon does not enable this resolver. On the local
September recordings it reduces computation but fragments previously correct
identities; see [the measured comparison](../docs/progress/2026-09-10-bounded-entity-fitting.md)
for the results and the remaining work under R-WS-13.

The experiment keeps the original observations in SQLite, retrieves older
observations through four deterministic appearance hash tables, and offers each
camera viewpoint its own spatial/appearance shortlist. The active pool is at
most 48 pending observations and 24 existing candidate entities per view, with
the production limit of two new entities per pass. Two queued entities can be
reviewed per frame, using eight founding and sixteen recent observations.
Revision can detach an observation, clear contaminated exemplars and refit the
remaining support. Detached observations retain their IDs and measurements and
can be grouped again. The runtime adapter's optional `association_allowed`
hook prevents a rejected observation immediately rejoining its former owner.

`negative_evidence.py` tests whether archived depth measures clear space through
a proposed position. Missing depth, stale captures, uncertainty, occlusion and
frame edges abstain. Three separated viewpoints must contradict the same
unchanged position before its placement is withdrawn. This is an experimental
depth contradiction rule; a calibrated detector-miss likelihood is still absent.
No placement was withdrawn by this rule on the tested recordings.

The indexes grow with the archive and have bounded query buckets. Working-set
limits are approximate retrieval limits, not a guarantee of a globally optimal
association or constant end-to-end runtime. Queue backlog, bucket overflow and
candidate deferrals are reported. The prototype's revision vetoes and queue are
in memory; deployment as an active resolver would require persistence and
recovery tests, as well as acceptable identity results.

Run from the repository root, with NumPy and SciPy available:

```text
python world_state/bench_incremental.py captures/m0-2026-09-08/world.db --mode baseline --output captures/comparison/baseline-08.json
python world_state/bench_incremental.py captures/m0-2026-09-08/world.db --mode shortlist --output captures/comparison/shortlist-08.json
python world_state/bench_incremental.py captures/m0-2026-09-08/world.db --mode revision --output captures/comparison/revision-08.json
python world_state/bench_incremental.py captures/m0-2026-09-08/world.db --mode negative --output captures/comparison/negative-08.json
python world_state/report_incremental.py captures/comparison
python -m unittest world_state.test_incremental
```

Reporting additionally needs Matplotlib. `--repeat 2` feeds a drive through twice
with distinct observation IDs and timestamps for a workload test. It is not a
second independent accuracy sample. `--max-seconds` saves a clearly marked
partial result when a run reaches its time budget. The `cached` mode is a
memoization control which was slower in the measured trial.

The benchmark reads the source database in read-only mode and resolves into a
temporary database. It preserves observation IDs, records input and code hashes,
times each update, and counts actual geometry calls. It does not re-perceive the
images, remeasure bearings, or alter the source recording. All modes use the same
stored detections and omit wall checks because contemporaneous map snapshots
are unavailable. Empty/skipped frames are not replayed.

`score_incremental.py` checks observation-level cannot-link cases in
`labels/incremental-2026-09-10.json` and retention of same-object pairs within
the 52 previously labelled clean entities. It distinguishes separated from
unresolved cases. The older `bench_identity.py` remains useful for scoring a
signal on the original entities; its transfer of entity verdicts by overlap is
not a way to judge corrected splits.

## Install and run

The deployer installs source. Models and TensorRT engines are host-built runtime
assets under `vendor/`:

```bash
ssh orin 'sh ~/ugv/world_state/install_perception.sh'
ssh orin 'sh ~/ugv/world_state/install_gpu_recovery.sh'
ssh orin '~/ugv/world_state/restart_perception.sh'
```

The supervisor starts `run_perception.sh` from the `jetson` user's crontab. It
loads models on the first request and attempts GPU recovery before each start.
Use `restart_perception.sh`; do not kill or launch the child directly.

Health:

```bash
ssh orin "curl -s http://127.0.0.1:8776/health"
```

The response identifies the selected backend, fallback reason, load time and
whether inference is busy.

## Checking a hypothesis with one look

`world_state_check` answers whether one stored look shows something where a claim
says a thing stands, for the executive's M0a inspections
([R-AUT-12](../docs/requirements/autonomy.md#r-aut-12)); the rule is
[`hypothesis_check.py`](hypothesis_check.py). **It is a question about a place,
not about identity.** On the labelled drive of 2026-09-08 a held-out look of a
real object matched the rest of its looks at 0.70 or better 43% of the time, and
a different object put in its place matched that well 2.3% of the time, so one
look cannot say which thing it sees and nothing here claims to.

A look answers `supported` when a region's ranged point lands within the claim's
uncertainty plus 0.15 m, `contradicted` when depth was measured past the place
across its whole uncertainty, and `unresolved` otherwise, with the reason: a look
whose direction was withheld, a place outside the depth camera's view, too little
depth, something nearer in the way, or a surface with no region on it. A look
whose own error, at twice its pointing and range errors, is larger than 0.15 m
cannot confirm anything, because in a furnished room something stands within a
noisy allowance of almost anywhere. A claim looser than half a metre is not
tested. Not seeing a region is never a contradiction.

A look taken for a check is recorded even if the picture matches the last one,
keeps its depth map whether or not anything was ranged, and saves the depth
camera's lens beside it, so the check can be replayed at a desk. The daemon
takes it holding the camera, wakes the depth camera for it, and aims it: one
scan is measured against the map before the shutter and the gimbal pans,
within its calibrated 20 degrees, to put the place in the middle of the picture
([on the rover](../docs/progress/2026-10-01-hypothesis-check-on-the-rover.md)).
`bench_inspection.py` replays it over a recording's real looks; see
[the replay](../docs/progress/2026-10-01-hypothesis-inspection-replay.md).

## Control calls

The daemon exposes control calls on TCP 8769 for the console and diagnostics:

- `world_state_summary`
- `world_state_search`
- `world_state_entities`
- `world_state_entity`
- `world_state_observations`
- `world_state_frame`
- `world_state_viewpoint`
- `world_state_clear`
- `world_state_rebuild`: the kept looks read again under their outlines and every
  thing built again from them; without `apply` it only says what would change
  ([the runbook](../docs/runbooks/world-state-rebuild.md))
- `world_inspect`
- `world_state_check`

There was a `world_map_session` here and it has been removed. It read as a
question and was an instruction: every call minted a new session, which is to say
it told the rover that everything it had located belonged to a map that no longer
existed. The session moves for one reason now -- the map identity underneath it
changing -- and the number is readable in `world_state_summary`.

Voice tools are read-only: `find_thing`, `go_to_thing`, and
`distance_between_things`. Clearing and direct inspection are not shown to the
voice model.

## Verification and diagnostics

Run the offline suite from the repository:

```bash
python world_state/selftest.py
```

It covers storage, migration, geometry, association, perception contracts,
search and OAK range handling with fakes. It does not prove camera calibration,
encoder pose, GPU execution or real-room identity.

Useful replay and measurement tools remain beside the component:

- `replay.py` reruns stored observations through current resolution logic,
  following the map changes the recording itself went through (`--session`
  pins it to one), and reports how much survived them;
- `reanchor.py` lines an old map up with the one the rover is on and carries
  the things it stranded across, folding away the copies of them that were
  found again in the meantime; it writes nothing without `--apply`;
- `bench_oak.py` measures the relationship between the two cameras;
- `bench_bearing.py`, `bench_height.py` and `bench_cluster.py` compare geometry;
- `bench_whole.py` replays one map session and then re-solves all of it at once
  by `cluster.py`'s expectation-maximisation, seeded with what the resolver
  ended up holding. It finds the duplicates the resolver leaves and loses too
  many things to adopt
  ([2026-10-03](../docs/progress/2026-10-03-whole-session-em.md));
- `bench_perceive.py` and `bench_still.py` inspect model and capture behavior;
- `bench_identity.py` scores a candidate identity remedy against
  [labels/m0-2026-09-08.json](labels/m0-2026-09-08.json), which is 76 things
  from one drive with a written verdict each: whether every look in the thing is
  the same object, or two, or bare floor. **Identity is the criterion this
  component fails**, and it is failed by 17 of those 76, so a change to
  association has somewhere to be measured rather than argued about. Verdicts
  join to a rebuilt thing by which looks it holds, because every entity
  identifier in the room changes when the resolver does.

That proof was taken on 2026-09-07 and measured range does prevent false
crossings. The next hardware proof is a different one, and it is not about this
camera at all. Sweeping the gimbal through a position from either side -- which
needs an overshoot, or the approach is only ever opposed at pan 0 -- shows the
gimbal camera **carries about a degree and a half of backlash at every angle in
its travel**, against a same-direction floor of five hundredths. The OAK was
bolted to the chassis then, so nothing but the gimbal's own pointing could account
for it. That
alone is the whole of the 1.5 degrees a bearing here is believed to. **Which way
the gimbal last moved is recorded now**, which it was not when this paragraph was
first written: `Rover.pan_approach` is set as each servo command goes out and
travels on the capture, because the moment the command is sent is the only moment
it is knowable. What consults it is below.
Two more faults ride with it: a gain-like walk somewhere between four and eight
per cent -- the bench cannot pin it closer, because the room moves while it
measures -- which vanishes straight ahead and reaches one to two degrees at pan 30,
and a roll that moves with
pan and that neither of the other two can produce. This component never aims the
camera -- it captures wherever the gimbal is -- so its looks have happened to be
straight ahead so far, but a face being tracked or a `look_at` puts them out at
wide pan where the gain is worst. Measure the pan servo's commanded angle against
its actual one before anything else; it is the largest correctable term.

That measurement has since been made, within a bounded envelope. The faults
themselves are untouched and belong to every bearing either camera records: the
OAK rides the same servos now, so a look through it is pointed exactly as badly
as a look through the fisheye. What they no longer touch is the step between the
two cameras -- the OAK turns with the fisheye, so where a fisheye box lands in
the OAK's picture is the same at every pan and tilt.

**The fisheye's own lens model was the next largest term, and it was not a
servo fault.** Fitting one rigid mount across fifteen gimbal positions on
2026-09-30 only closed once the fisheye's angles off its axis were stretched by
7.2%. The lens was refitted the same day without trusting a servo, and every
stored look was redrawn through it by `relens.py`, which rewrites only what it
can first reproduce through the old lens -- see
[the lens entry](../docs/progress/2026-09-30-the-fisheye-lens-refitted.md).
