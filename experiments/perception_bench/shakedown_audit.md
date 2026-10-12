# M5 shakedown identity audit, 2026-10-12

**Criterion 4 fails on this evidence.** The red bean bag acquired a second
identity after moving; one armchair record holds both physical armchairs, including
confident attachments after the refit. This measures R-WS-17 and R-WS-18; it does
not propose accepting bad poses or changing the rover's store.

The source is a SQLite backup of the live world store and episode store taken at
11:44 IST on 2026-10-12, opened read-only. The current map is `7da19bef3888`, session
67, generation `07dec15b41240130`. The consistent backups remain on the rover at
`/tmp/codex-m5-audit-world.db` and `/tmp/codex-m5-audit-episodes.db`; metadata and
selected photographs were copied to `tmp/codex-m5-audit/`. The snapshot hashes,
individual reviewed regions, placements and resolver notes are in
[shakedown_audit.json](shakedown_audit.json). The staging source is
[shakedown-1012.json](../m5_changes/staged/shakedown-1012.json).

## Four things, and the records holding their looks

Counts below are **minimum visually confirmed regions**, not an exhaustive census
of tiny or occluded detections. The review covered 159 whole pictures, 420 candidate
crops and historical strips of 12 records. Small, mixed and ambiguous crops were
excluded; the owner has not reviewed these new visual labels. Record memberships
are the snapshot's current assignments, including retrospective resolution.

| Physical thing | Before staging / earlier history | After staging | Outcome |
|---|---|---|---|
| Charger armchair, carrying the patterned throw and boxes | Taped N4 explicitly labels 317, 327, 382, 427, 478, 599, 784; historical sampled 317/327/838 show the throw-bearing chair | 28 confirmed regions: 20 unassigned; the other 8 go to 651 (3), 909 (2), 838 (1), 918 (1), 527 (1) | Its history is scattered across existing records. The current 838 also holds the other armchair. No single correct move/history is demonstrated. |
| Balcony armchair, the other plain chair | Two pre-staging looks, 99860 and 99871, both go to 371; earlier sampled 371/601/466/527 show the owner's seated armchair | 34 confirmed regions: 20 unassigned; the other 14 go to 379 (2), 601 (4), 466 (1), 320 (3), 896 (1), 838 (3) | Several existing records are still used; 371 received neither of the later confirmed looks. 838 combines both chairs. |
| Red bean bag | Added at (-19.8, -14.15), then moved about 08:08 to (-16.25, -13.2); no claimed pre-addition identity | 26 confirmed regions: 15 unassigned, 5 at its first place go to 917, 6 at its second place go to 920 | **Split across a move.** 917 still exists, with 12 total memberships, at (-20.544, -14.628). 920 has 6 total memberships at (-15.565, -13.618). No history link between them is present. |
| Black bean bag | Added at (-16.4, -11.75); it did not move again during the test | 28 confirmed regions: 25 unassigned, 3 go to 914 | One new record, 914, at (-16.204, -11.421), about 0.38 m from the staged scan estimate. This is an addition/control, **not a successful movement test**. |

The charger-chair distinction is supported by its throw/boxes and scene context:
look 100672 at 08:14:41 has a ranged surface at approximately (-17.39, -15.51),
agreeing with its staged new place (-17.5, -15.55). Accessory changes and occlusion
make some edge-only looks ambiguous, so those are not counted. The M4 record list
is an earlier label set, not proof that every historical member of those records
is pure. Several duplicates and mixtures predate this shakedown.

## An incorrect confident association after refitting

Record **838** holds the charger chair's throw/boxes in look **100674**, frame
`20261012-081441-685220`, and the other plain chair in looks **100838** and
**100847**, frames `20261012-081736-4564a8` and `20261012-081738-086bab`.
The actual crops were inspected, not inferred from their entity positions.
The resolver attached these as ordinary matches with appearance scores 0.63,
0.72 and 0.69, using only 8%, 7% and 14% of its allowed bearing disagreement.
All three are after the refit. This is a confident wrong **membership**, rather
than a logged merge operation: `merge_runs` is empty in the snapshot.
Historical look 91803 (2026-10-10 13:39:54) also shows the throw-bearing charger
chair in 838. This audit cannot date the first historical contamination.

## Why so many looks never acquire an identity

The episode store holds **25 successful unique `world_inspect` pictures** in the
run, plus three failed calls/retries. **20 of those 25 have `pose: null`**; their
answers say the heading check could not match at least 90% of the scan to a wall.
Those 20 are living-room looks. The five with a pose are corridor/bedroom looks.
The camera still kept the pictures and some ranges, but correctly supplied no
bearing and settled no identity when it could not certify a pose.

This happens well **before** the navigation error at 08:00:58: for example, the
07:52:49 inspection reported a best fit of 73%, and 07:52:57 reported 70%, because
the room had changed. Do not describe every null pose as the later localization
fault. Across both passive capture and explicit inspections in the run there are
513 regions in 82 frames, 324 without a recorded pose and 354 currently unassigned.
Those are stream totals, not 82 deliberate autonomy inspections.

The red bag's first record was minted at 08:02:21, using both earlier looks and
bad-pose-window looks; its first placement is consequently contaminated. Its
second record was minted at 08:17:51. Geometry errors cannot prove movement on
their own, but the owner's staged move and the inspected red crops prove that
917 and 920 are the same physical bag at different times.

## What the rover answers now

A read-only `find_thing` call over TCP 8769 at approximately 11:48 IST answered
"the black armchair" as found, 41 sightings, last seen four hours ago, placed at
**(-18.47, -16.33), known to 0.1 m**. The corresponding `world_state_search`
selected **601**, look **100480**, frame `20261012-080118-8ff051`, score 0.1428.
That picture shows the plain balcony chair, but it is in the **bad-pose window**.
The selected entity placement is (-18.466, -16.326), stated uncertainty 0.116 m;
its rounded voice answer conceals that caveat. It is about 0.71 m from the staged
new balcony-chair estimate (-18.95, -15.8), which is itself scan-read rather than
tape truth. The phrase does not distinguish the two black chairs.

This is not a newest-sighting query: `_recall` chooses the highest text-matching
assigned look among the top ten and returns that look's time, while reporting the
entity's accumulated placement. The current answer therefore combines a selected
historical look with a separately updated placement. The saved full TCP answer is
`tmp/codex-m5-audit/recall-now.json`.

## What to change next

First freeze this red-bag split and the 838 cross-chair memberships as regression
cases. Any proposed identity remedy must retain the bag's history across movement
and refuse the cross-chair attachments on these actual crops, including the
after-refit examples. Model and replay must reproduce the existing memberships
before a fix is attempted; this audit does **not** claim that reproduction yet.

Retain a time-stamped position history separately from lasting identity rather
than fitting pre- and post-move bearings to one static point. Keep pose validation
and identity association separate: preserve rejected-pose provenance for replay,
but never invent a trusted bearing to fill these nulls. The scan-fit refusal in a
changed room must be measured as a coverage limit, not silently relaxed. Finally,
make recall expose identity ambiguity and pose provenance before claiming a
decimetre location from a bad-pose look. These are proposals to test, not deployed
fixes. No store edits, service restarts, deployment or movement were performed.
