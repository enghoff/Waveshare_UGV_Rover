# The world model kept per visit, on the rover

Version 4 of [the world model kept per visit](../../docs/plans/a-world-model-kept-per-visit.md),
chosen by the owner over the store's own filing on 2026-10-17
([why](../../docs/progress/2026-10-17-version-4-against-the-deployed-store-on-unseen-pictures.md)),
kept up to date on the rover beside the store. The store keeps filing looks as
it always has; nothing here writes to it.

## What runs

`job.py`, started at boot by `run_job.sh` from the `jetson` crontab
(`install_job.sh` puts the line down) and reloaded by `restart_job.sh` after a
deploy. Niced, inside navigation's ROS environment because it listens to the
lidar. Every minute it:

1. reads the regions the store has added, once they are a minute old
   (`engine.prepare`: the loader, wall places, colour and the learned likeness,
   exactly as the desk pipeline builds them);
2. files every visit that has closed (`engine.Engine.step`, `model.run`'s loop a
   batch at a time), with the last half hour of lidar for absence;
3. writes, in `~/.ugv/world/visit_model/`:
   - `things.json`, the records: place, spread, state (present or gone), when
     seen, and the store's observation ids of each, for the daemon
     (`world_state/visit_things.py`);
   - `engine.pkl`, the engine, so a restart carries on where it stopped;
   - `decisions.jsonl`, what each visit decided;
   - `scans/`, the lidar and pose over each batch, in the drive recordings'
     format, so ordinary drives leave recordings behind (kept 14 days);
   - `job.log`.

With no `engine.pkl` it first files the whole history (about two and a half
minutes), with the drive recordings in `~/.ugv/world/visit_model/recordings/`
read in name order, as the desk's `run_versions.sh` lists them.

## What is frozen

`model.py`, `data.py`, `recorded.py`, `lidar.py`, `poses.py`, `walls.py`,
`wallplace.py` and `colour.py` are version 4's as frozen on branch `visit-model`
at `cabe003`, with batch entry points added to the loader, wall places and
colour that compute each region as before. `likeness.py` is that branch's
`metric.py` without the fitting. `metric-1015.npz` (the learned likeness),
`scale-1015.npz` (its scale) and `walls.npz` (the building's walls) are part of
version 4 and are not refitted here.

## What proves it

- `test_engine.py`: fed a history in pieces, the engine files every region as
  one run does (12 and 40 pieces of the 2026-10-17 drives, identical).
- On the rover (2026-10-17): the inputs built from the live store in pieces are
  bit-identical to those built whole, agree with the desk's to a millionth, and
  the engine on the rover files the history exactly as the frozen desk run
  that passed M5 session 7 (same groups, same record names).

## What reads it

Nothing yet. The voice tool `find_thing` is to rank the model's records first,
with the store's own things as the answer when the records are missing, older
than fifteen minutes or on another map (the plan's step 8).
