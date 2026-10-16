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
