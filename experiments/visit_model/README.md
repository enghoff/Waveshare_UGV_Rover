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
