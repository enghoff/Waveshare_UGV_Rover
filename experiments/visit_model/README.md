# A world model kept per visit: the benchmark

Offline work for [the plan](../../docs/plans/a-world-model-kept-per-visit.md):
everything the rover saw from 2026-10-03 to M5 session 4, in one map frame,
with labels saying which regions show which object, and the deployed store's
own filing scored on it as the baseline.

| File | What it does |
|---|---|
| `data.py STORE.db OUT.npz REC.npz...` | every gimbal region of map session 67 as arrays: when, where the camera stood and pointed, bearing, range, masked DINOv2 vector, and what the deployed store filed it under; a region the store left unposed takes navigation's pose from a recording, an unranged one the lidar's return across its width |
| `truth.py` | the five staged sessions (shakedown and M5 sessions 1 and 2 for development, 3 and 4 for the test), what was staged, and which physical object each label names |
| `score.py LOOKS.npz [FILING.json]` | the plan's counts for one filing: trials kept across sessions, wrong merges, records per object that stayed put, and the share of labelled regions filed |
| `sheets.py` | contact sheets for labelling: candidates by sight line and a loose likeness to known pictures, never by the store's filing; drawn on the rover |
| `labels/` | the shakedown's labels (Codex's, from branch what-where), session 4's, and `stayed-put.json`, 11 objects that were never staged |
| `recorded.py`, `lidar.py`, `poses.py` | the what/where streams' recording readers, from branch what-where |

    python data.py STORE.db LOOKS.npz $(recordings)    # once; 44,466 regions
    python score.py LOOKS.npz                          # the deployed store

The store copy is the rover's after session 4 (`tmp/session4-1015/world-after.db`);
the recordings are the shakedown's, sessions 1-4's, and the controls and scene 1
of 2026-10-10 and 11. Every recording agrees with the store's map within a few
centimetres at the looks they share.

## The deployed store

Scored 2026-10-15, before any new model was run:

| | Trials kept | Wrong merges | Labelled regions filed | Records per unmoved object (2+ regions) |
|---|---|---|---|---|
| development | 1 of 7 | 3 (both armchairs in one record) | 70% | 2-10, median 3 |
| test (sessions 3, 4) | 3 of 5 | 0 | 63% | 0-7, median 3 |

"Kept" is count_all's lenient rule: the record holding most of the thing's
pictures after the move held any one of them before. By the strict rule (the
record that held most of them before) the deployed store keeps none.

## Version 1 and its test

`model.py` (frozen at `f1eb342`): clusters per visit, one assignment against
memory, lidar absence, pending identity, consolidation of objects never seen
together, raised things placed on the stable walls (`walls.py`,
`wallplace.py`), lidar ranges as floors, spreads and likeness bars from
`calibrate.py`. `run_dev.sh` runs development (nothing after 10-14 13:00);
`run_test.sh` ran the test once, `gate.py` judging it by the plan.

| | Moves kept (strict) | Mixed records | Unmoved in two records or fewer | Filed |
|---|---|---|---|---|
| development | 4 of 7 (3) | 2 | 9 of 11 | 88% |
| test | 5 of 5 (0) | 1 | 10 of 11 | 95% |

The test does not pass (docs/progress/2026-10-15-a-world-model-kept-per-visit-tested.md).
`why.py`, `splits.py` and `merges.py` trace a filing's decisions for one object,
every split and every mixed record.

## Version 2

Development on all five sessions, since the test has been seen.

- `colour.py`, run on the rover: each region's colour inside its own outline,
  merged into the looks as `colour`.
- `metric.py`: a likeness learned with no labels from pairs the pictures give
  for free (KISSME); `metric-1015.npz` is the one trained on every look to
  session 4, frozen for version 2's test. With `--save` it writes each
  region's learned vector into the looks (`lvec`).
- `model.py` now uses the learned likeness (mapped onto the old scale on
  same-picture pairs), vetoes matches whose colours differ, and joins a
  record found standing after half an hour unseen to a look-alike seen gone
  in the same hour (`link_path`).
- `run_v2.sh` runs it over every look and scores all five sessions.

Over all five sessions, version 2 against version 1: moves kept 7 (2 strict)
against 9 (3) of 12, mixed records 2 against 3 -- not separable on a benchmark
whose moves are half the two armchairs. Its test is the staged session the
plan describes.

Since 2026-10-16 the model is pass/fail on honesty alone
(docs/decisions/the-world-model-is-pass-fail-on-honesty-alone.md), so
version 2 was made to decline its one mixed record of two distinguishable
things (pictures of the blue painting in the snowy-tree painting's record): a
cluster placed only where its sight lines cross now joins a record only if
every one of those lines passes the record's place, and only as alike as sight
lines alone need (0.75, not 0.55). Over all five sessions, with records
holding both armchairs not counted: no mixed record (was 1), distinguishable
moves followed by the strict reading 3 of 6 (was 2), pictures filed 87% and
95%, unmoved things unchanged. Frozen here for session 5.

## Session 5 and wrong merges across sessions

Version 2 frozen at `3c2498e` failed M5 session 5 (2026-10-16): the black bean
bag, put down 0.6 m from where the red one stood in session 4, went to record
v251, which held the red bean bag's pictures and, since 10-07, unlabelled
pictures of other things at that spot; two of its colour samples were dark
(`probe_colour.py`). It kept the red bean bag in one record through its move.

`score.py` counted a wrong merge only within a session until then. It now
counts a record that, during a session, takes a thing other than those it
held in that session or any earlier one, the two armchairs counting as one.
Over every session (deployed store by its ordinary matches):

| | shakedown | 1 | 2 | 3 | 4 | 5 |
|---|---|---|---|---|---|---|
| deployed store | 1 (paintings) | 3 (bean bags twice, paintings) | 0 | 0 | 0 | 1 (bean bags) |
| version 1 | 0 | 0 | 1 (armchair, painting) | 1 (paintings) | 0 | 1 (bean bags) |
| version 2 | 0 | 0 | 0 | 0 | 0 | 1 (bean bags) |

Tried for version 3 (2026-10-16), not adopted: the colour veto against a
record's median sample rather than its best (`COLOUR_MOSTLY=1`). It removes
session 5's bean bag merge but makes three elsewhere (a red bean bag record
taking an armchair in session 1 and the cabinet in session 3; the front door's
taking the rug in session 4): a veto changes which record wins, and the next
best is not always right. Version 3 needs records that stop drifting from
thing to thing by place, not a stricter veto alone.

## Version 3

Developed on all six sessions (2026-10-16), so none of it is a test:

- **A record unseen for three hours is "here as before" only on a move's
  likeness (0.75, not 0.55).** Hours later, standing where something stood
  says little about which thing it is.
- **An arrival pairs first with a departure seen in the last half hour**, and
  with older ones only after the visit's absences are in, so its own
  departure seen in the same visit can win.
- **Two pictures whose colours match under 0.2 are not linked into one
  cluster** (3% of same-object pairs, 18% of different ones): a cluster of
  two rug pictures and one of the red bean bag went to the rug's record.
- **Merges run in a fixed order.** They ran in the order of a set of names,
  which changes with Python's hash seed, and with two seeds in three an
  earlier build made that rug merge and with one it did not.

`run_v3.sh` runs it; `compare.py` sets filings side by side; `run_versions.sh`
runs the frozen versions with a fixed seed. Versions 1 and 2 vary with the seed
only in records per unmoved object (by one). With session 5's unmoved objects
labelled too (`labels/stayed-put.json`, after versions 1 and 2 had been scored
on the session):

| | wrong merges, all sessions | distinguishable moves strict | unmoved in 2 records or fewer, sessions 3-5 | filed, sessions 3-5 |
|---|---|---|---|---|
| deployed store | 8 | 0 of 10 | 4 of 22 | 69-74% |
| version 1 | 4 | 2 of 11 | 20 of 22 | 89-95% |
| version 2 (`3c2498e`) | 1 | 1 of 11 | 18-19 of 22 | 90-95% |
| version 3 (`e5b39fd`) | 0 | 6 of 11 | 21 of 22 | 90-93% |

Frozen at `e5b39fd` for M5 session 6. After a session: `session_collect.sh`
on the rover (experiments/m5_changes), `prepare_session.sh NAME` here, labels,
then `run_versions.sh`.

## Session 6, and a scale fitted over every picture

Version 3's test, M5 session 6 (2026-10-16,
docs/progress/2026-10-16-m5-session-6.md, on main), with labels for the four
things and the eleven unmoved objects made before any model ran on it
(`labels/session6-1016.json`, the `session 6` lists in `labels/stayed-put.json`).
On the session version 3 made no wrong merge and followed all three moves of
the wait (`compare.py` row `session 6`: deployed 8 wrong merges, version 1 3,
version 2 0, version 3 0). It does not pass: in the same run its record of the
blue painting took two of session 5's pictures of the snowy-tree painting.

The cause is `learn_scale`, which maps the learned likeness onto the image
vectors' scale by matching quantiles of same-picture pairs over the whole
looks file, later pictures included. The 44 pictures the rover took on its
charger after session 5 had been copied off (11:29-11:41) shift a score on
2026-10-03 from 0.850 to 0.849, and the history diverges from there
(`looks-s5end.npz`, session 5's pictures only, reproduces the merge; session
6's recording changes nothing; identical input gives byte-identical filings).
Version 2 shares it and gained an armchair-and-cabinet merge in session 3 the
same way. A model whose past filing moves with every new picture cannot be
checked once: the scale is to be fitted on development pictures and frozen
beside `metric-1015.npz`.

## Honesty over draws, and version 4

A wrong merge that comes and goes with unrelated pictures says nothing about
the next session, so honesty is judged over draws: `draws.py` writes looks
files each without a random 1% of the pictures that hold no labelled region,
and `run_draws.sh LOOKS TAG COMMIT` runs a frozen version on all pictures and
five draws. Version 3 made 1 wrong merge on all pictures of the seven sessions
and 0, 1, 0, 2, 1 on five draws, always from the same three places: the black
bean bag in the red one's record (session 5), far-off pictures of the
snowy-tree painting in the blue painting's, one red bean bag picture in the
rug's.

First the likeness scale was fixed: `learn_scale` is fitted once on the
pictures before 2026-10-16 and kept as `scale-1015.npz` (`--fit-scale`), so a
picture added later leaves every earlier decision as it was (identical
decisions before the first added picture). That alone did not change the
spread (1, and 0, 1, 0, 0, 2).

**Version 4** (`cabe003`) raises every bar that joins two things by 0.06
(`JOIN_MARGIN`), so a likeness just over a bar starts a record of its own:

| on all pictures, seven sessions | wrong merges, all + 10 draws | moves strict | unmoved in 2 records or fewer | filed |
|---|---|---|---|---|
| deployed store | 16 | 2 of 16 | 11 of 44 | 69-80% |
| version 3 (`e5b39fd`) | 1; 0-2 a draw | 10 of 18 | 41 of 44 | 86-95% |
| version 4 (`cabe003`) | 0; 0 in every draw | 6 of 16 | 43 of 44 | 78-90% |

Tried on the way: 0.03 (three wrong merges in six runs) and 0.05 or 0.06 with
the move bars spared, which followed more moves (10 of 16) but let a
painting's record "move" onto another painting in one run of eleven and one of
three. `scale-1015.npz` is part of version 4 and is not to be refitted.

## Session 7: version 4's test, passed

M5 session 7 (2026-10-17, docs/progress/2026-10-17-m5-session-7.md on main):
session 6's swap the other way round, the black bean bag set down 0.25 m from
where the red one had just stood. A manual drive went through the staged rooms
before the session's record; its pictures belong to the session (`"began"` in
the staged file, `"moved_at_step": "record"` for the red bean bag put down after
it), and its three minutes without a sound pose are a bad-pose window. Labels
were committed before any model ran (`d385954`).

`run_draws.sh looks-session7-1017.npz s7v4 cabe003`: no wrong merge in any
session on all pictures or in five draws. On session 7, version 4 kept 4 of 7
trials strictly and 11 of 11 unmoved objects in two records or fewer, filing
79% of labelled pictures; over the seven sessions, wrong merges deployed 20,
versions 1-4 10, 2, 2, 0 (`run_versions.sh`, tag `s7`). Next is the plan's step
7: version 4 in shadow on the rover.

## Absence from the camera (a candidate, not adopted)

Version 4 marks a record gone only when the lidar sees through its place, so a
stray -- a record started from a region or two that was never anything -- or a
thing above the lidar's plane stays present for ever: on the rover on
2026-10-18, 284 of 502 standing records had been seen on one or two visits, and
the first autonomous run's 14 looks at such records fed none of them.
`model.py` can take absence from the camera as well (`CAM_EMPTY_VISITS`, off
unless set): a visit with a standing record's depth place within 2.5 m and 45
degrees of the camera's axis in three pictures or more, that files nothing to
it, is an empty visit; so many in a row and it is gone. With it off the filing
is version 4's byte for byte.

- `absence.py`: on version 4's filing, how often a visit with a never-staged
  object's place in view filed nothing to its records -- 11% with three
  pictures within 2.5 m -- and the longest run of such visits in a row, 3 (the
  snowy-tree painting).
- `camera_gone.py`: what a run called gone by the camera, judged on the labels.

Over every picture to the drives of 2026-10-17 (`looks-drives1-1017.npz`):

| Empty visit means | In a row | Gone by the camera | Records of never-staged objects called gone | Standing at the end |
|---|---|---|---|---|
| (version 4) | | 0 | 0 | 518 |
| nothing filed to it | 4 | 546 | 10, among them the blue painting's, the rug's and the forest painting's main records | 284 |
| nothing filed to it, and no region of the visit placed within 0.5 m (`CAM_EMPTY_MODE` "nothing there") | 4 | 254 | 2: a second record of the rug (8 labelled regions) and one holding one picture of the animals painting | 363 |
| the same | 8 | 131 | 1: one holding one picture of the animals painting | 426 |

"Nothing filed to it" confuses a look filed to a second record of the same
thing with the place found empty. "Nothing there" does not touch a main record,
but still calls records of things that were there gone, so as "gone" it is not
honest: the rover would say the rug is no longer where it was. Not adopted;
the question it leaves is whether such records should be set aside as not
found again rather than called gone.

### Set aside as not found again (adopted, on the rover)

The owner agreed on 2026-10-18 that a record looked for and not found should be
set aside as not found again rather than called gone. In `model.py` the camera
rule now only flags a record (`unfound`), reading nothing the filing reads, so
the filing stays version 4's whatever the switch. Scored with `unfound.py`:

| An empty visit also needs | In a row | Set aside | Of never-staged objects |
|---|---|---|---|
| nothing placed within 0.5 m | 3, 4, 6 | 267, 226, 155 | 9, 6, 4 -- the snowy-tree painting's main record (54 pictures) aside at the end |
| no sight line past it that stopped there or short, or was not ranged | 2, 3, 4 | 6, 3, 0 | 0 |
| the same, regions filed to a record standing elsewhere not counted | 2, 3, 4 | 62, 49, 30 | 0, but 31, 19, 10 found again later |

Paintings hung above what the depth camera ranges up close give no distance,
so nothing seems placed there; unranged sight lines from a 160-degree camera
pass near nearly every place. What was adopted is simpler (`set_aside.py`):
seen on two visits or fewer, not for 7 days, and nothing filed to it on three
close visits since -- 81 of 518 standing records at the end, none holding a
labelled picture, every labelled object keeping its main record. The rover's
model counts the close visits (`world_state/visit_model/model.py`,
`looked_for`) and the daemon applies the rule
(docs/progress/2026-10-18-records-not-found-again-are-set-aside.md).

## The multi-day benchmark's two days (2026-10-20)

`truth.py` takes the benchmark's days 1 and 2 (role `md days`): the three
things set out before each day's record began (`staged/`), labelled from
contact sheets drawn from the full-size photographs (`labels/md-day*.json`;
day 2's stool is in no region). The looks file is every picture to 2026-10-20
(`tmp/visit-model/looks-md-1020.npz`), with the two days' drive recordings
and the M10 trial's of 2026-10-18 added to the usual ones. Frozen version 4
over it gives the record the rover gave for 98.6% of the two days' regions.

Version 4 kept 1 of the 5 judgeable trials: every thing already had three to
five records before the benchmark began. `probe_alike.py` shows, for a visit,
how alike each labelled thing's regions are to every record then.
`pair_likeness.py` measures how alike records of one thing and of two things
are after a whole run. `false_gone.py` counts gone calls on records of things
that never moved. `score_all.py` sets several filings side by side, every role
at once. `draws_seq.sh` is `run_draws.sh` one run at a time.

Two switches in `model.py`, off unless set, the filing version 4's with both off:

- `ONE_THING_RIVALS` (with `DOUBLE_COS`, 0.70): in a move, a gone record as
  alike to the leader as records of one thing are and never in one picture with
  it is not a rival. Over every labelled pair of records, two things matched at
  0.63 at most, one thing's records 27 times in 98 at 0.70 or more. It follows
  the red bean bag on day 1 (0.809 against its own 0.772 and 0.770) and changes
  no trial on any earlier session.
- `SOLID_BY_VISIT`: hits against passes over the visit's scans rather than one
  scan, which passed between the stool's legs. It calls the stool's day-1
  record gone on day 2, but adds 41 gone calls over the history, one of them on
  the rug's record (what the lidar hit stood on the rug), and loses session 2's
  cabinet. Not adopted.

docs/progress/2026-10-20-where-the-benchmark-days-pictures-went.md has the
numbers.
