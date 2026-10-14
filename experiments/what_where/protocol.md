# What/where streams on the shakedown: the frozen protocol

The first test of [prediction in what and where streams](../../docs/plans/what-where-streams.md):
do prediction errors carried in a frozen appearance space ("what") and in the
map ("where"), measured against the rover's own memory, single out the things
the owner staged in the M5 shakedown of 2026-10-12, without flagging the rooms
nobody touched? Nothing is learned, rendered or tuned on the shakedown.
Everything below, the code and the development thresholds were committed before
the shakedown was scored.

## Evidence

- The read-only SQLite backup of the world store taken at 11:44 IST on
  2026-10-12 (`tmp/codex-m5-audit/world.db`, map session 67). Every region the
  region finder (YOLOE) kept carries a DINOv2 vector of its crop with the
  background blanked (`dino_alone_blob`), its map bearing where the look had a
  pose, and its range where the depth camera measured it.
- Codex's visual labels of the shakedown, made on 2026-10-12 for the identity
  audit, before this method existed
  ([shakedown_labels.json](../perception_bench/shakedown_labels.json)): 118
  regions of the two armchairs and two bean bags.
- The staging, read off the recording's scans
  ([shakedown-1012.json](../m5_changes/staged/shakedown-1012.json)).

The drive recording, which holds navigation's pose for the looks the store kept
none for, is on the rover, which was off the network on 2026-10-14. So the where
stream sees only looks the rover's heading check certified; most living-room
looks after the staging have none, and that coverage is reported, not hidden.

## The streams ([streams.py](streams.py))

Each region below is furniture-sized by Codex's labelling floor: its box
reaches below 57% of the picture's height, covers 1.2% of it and is a tenth of
it tall. Memory is every region the store held before a cutoff, frozen.

- **what**: 1 - the region's best cosine against every remembered region in the
  house. High means nothing like it has been seen.
- **where**, for looks with a pose: the references are the 30 most recent
  remembered looks taken within 1 m and facing within 45 degrees. A consistent
  place is a bearing within 15 degrees of where a remembered region should
  appear from this camera (through its map point when it was ranged) and, when
  both were ranged, a range within 1 m. The score is (1 - what) minus the best
  cosine among reference regions in a consistent place, never below 0: how much
  better the region matches something elsewhere than anything remembered here.
- **gone**, for looks with a pose: each furniture-sized reference region within
  45 degrees of the camera's axis and at most 4 m away (when ranged) is
  expected. Its score is how well the other references agreed on it minus how
  well any region of the look matches it in a consistent place. Over a visit, a
  remembered region's gone score is its smallest over the looks that expected
  it, and it counts only when two or more looks expected it.

## Development ([development.json](development.json))

The seven unchanged controls of 2026-10-11 (08:04 to 13:07), each a visit with
memory frozen at its start. All choices above were made on these alone,
looking at score distributions: the reference depth (3 recent looks within
0.5 m left one unchanged region in ten scoring high, so 30 within 1 m), the
bearing tolerance (95% of alike matches agree within 14 degrees) and the range
tolerance (90% within 0.6 m, 95% within 1.3). Thresholds are the 99th
percentile of each score there:

| Stream | Threshold | Regions |
|---|---|---|
| what | 0.3216 | 1,028 |
| where | 0.4587 | 1,011 |
| gone (per visit) | 0.9316 | 1,094 expected twice |

## Test ([run.py](run.py) `test`)

Memory frozen at 07:48:05, the end of loop A, when the staging began. Two
visits: the run until navigation's pose went wrong (07:52:15-08:00:58), and
everything after the refit (08:11:00-08:21:00). The bad-pose window between is
left out.

- A region is **flagged** if furniture-sized and over the what threshold
  ("new") or over the where threshold ("out of place").
- A staged **arrival** (each bean bag; each armchair at its new place) is
  **found** when labelled regions of it are flagged in two or more distinct
  looks across the two visits. The what stream needs no pose, so labelled
  regions from looks without one count for it.
- A staged armchair's **old place** is found when a remembered region gone
  over a visit lies within 1 m of it (or, unranged, its sight line passes
  within 0.5 m of it).
- **Room-controls** are the corridor and the bedroom in each visit (four),
  judged by the camera's position. A **persistent false alarm** is a group of
  flagged regions in two or more looks of one visit that look alike (cosine
  0.5) and sit in one place (map points within 1 m, or cameras within 1 m and
  bearings within 15 degrees), or a gone region over a visit whose place is in
  the corridor or bedroom. A flag whose crop shows a person is reported as a
  person, not counted either way.

**Shows promise** if at least three of the four arrivals are found and the
four room-controls hold at most one persistent false alarm, which is the
lidar watch's showing on the same session (five of six found, four with the
pose right; one furniture-sized false alarm in six room-controls, which did
not hold up). The comparison is not matched: the watch had the whole session
and every scan; the streams have the certified poses only.

Reported beside it, not decided on: which stream flagged each staged thing; the
coverage of labelled regions by each stream; flags in the living room more than
1 m from every staged place, which may be real changes the labels do not cover.

## What it cannot show

This is one session, staged once, labelled by Codex and not by the owner. Four
room-controls cannot establish a false-alarm rate. Thresholds chosen at the
99th percentile flag about one unchanged region in a hundred by design, so
persistence across looks is what keeps them from being alarms. A failure here
says the frozen encoder and nearest-memory prediction are not enough on this
evidence. It does not say that the streams with learned predictors would fail.
