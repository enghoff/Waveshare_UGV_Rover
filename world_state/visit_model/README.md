# The world model kept per visit, on the rover

Version 4 of [the world model kept per visit](../../docs/plans/a-world-model-kept-per-visit.md),
chosen by the owner over the store's own filing on 2026-10-17
([why](../../docs/progress/2026-10-17-version-4-against-the-deployed-store-on-unseen-pictures.md)),
kept up to date on the rover beside the store. The store keeps filing looks as
it always has; nothing here writes to it.

## What runs

Two processes, kept up by `run_job.sh`, which the `jetson` crontab starts at
boot (`install_job.sh` puts the line down) and `restart_job.sh` reloads after
every world_state deploy, both niced:

- `scan_log.py`, inside navigation's ROS environment (RoboStack's conda
  Python), listens to the lidar and navigation's pose and writes what arrived
  every 30 s to `~/.ugv/world/visit_model/lidar/`, kept 40 minutes;
- `job.py`, on the system Python, where version 4 was verified -- the conda
  environment has its own numpy and no PIL. Every minute it:

1. reads the regions the store has added, once they are a minute old
   (`engine.prepare`: the loader, wall places, colour and the learned likeness,
   exactly as the desk pipeline builds them);
2. files every visit that has closed (`engine.Engine.step`, `model.run`'s loop a
   batch at a time), with the last half hour of the logger's lidar for absence;
3. writes, in `~/.ugv/world/visit_model/`:
   - `things.json`, the records: place, spread, height, state (present or
     gone), when seen, how often looked for and not found (`looked_for`), and
     the store's observation ids of each, for the daemon
     (`world_state/visit_things.py`), with `filed_until`, up to when looks are
     filed, and `generation`, what the records' names `v12` are minted under;
   - `engine.pkl`, the engine, so a restart carries on where it stopped;
   - `decisions.jsonl`, what each visit decided;
   - `scans/`, the lidar and pose over each batch, in the drive recordings'
     format, so ordinary drives leave recordings behind (kept 14 days);
   - `job.log`.

With no `engine.pkl` it first files the whole history (about two and a half
minutes), with the drive recordings in `~/.ugv/world/visit_model/recordings/`
read in name order, as the desk's `run_versions.sh` lists them, then its own
`scans/` in time order, under a new `generation`. One it cannot read is kept
as `engine.pkl.bad`, and one filed some earlier way (`job.FILED_AS`) as
`engine.pkl.filed-as-N`, and the history is filed the same way.

## What is frozen

`model.py`, `data.py`, `recorded.py`, `lidar.py`, `poses.py`, `walls.py`,
`wallplace.py` and `colour.py` are version 4's as frozen on branch `visit-model`
at `cabe003`, with batch entry points added to the loader, wall places and
colour that compute each region as before. `likeness.py` is that branch's
`metric.py` without the fitting. `metric-1015.npz` (the learned likeness),
`scale-1015.npz` (its scale) and `walls.npz` (the building's walls) are part of
version 4 and are not refitted here.

One addition, which nothing in the filing reads: each visit counts, for every
standing record with a depth place, whether its place was within 2.5 m and 45
degrees of the camera in three pictures or more and nothing was filed to it
(`model.looked_for`), back to zero when something is. Over every picture to
2026-10-17 the filing with it is version 4's byte for byte.

## What proves it

- `test_engine.py`: fed a history in pieces, the engine files every region as
  one run does (12 and 40 pieces of the 2026-10-17 drives, identical).
- On the rover (2026-10-17): the inputs built from the live store in pieces are
  bit-identical to those built whole, agree with the desk's to a millionth, and
  the engine on the rover files the history exactly as the frozen desk run
  that passed M5 session 7 (same groups, same record names).

## What reads it

All since 2026-10-17, through `world_state/visit_things.py`:

- the voice tools `find_thing` and `go_to_thing`
  (`rover_daemon/rover_recall.py`) and the console's world popup, with the
  store's own things answering when the records are missing, older than fifteen
  minutes or on another map;
- an autonomous run (`autonomy/situation.py`), with **no** fallback: it carries
  the records' names from step to step, and the store names things
  differently. With no records it plans around no things, and says why.

Since 2026-10-18 the daemon sets a record aside as **not found again** -- never
calls it gone -- when it was seen on two visits or fewer, not for a week, and
looked for three times since (`visit_things.SET_ASIDE_*`). Over every labelled
session that set aside 81 of 518 standing records, none holding a labelled
picture; on the rover 127 of 630, each of eight sampled a second record of
something held within 0.4 m or a thing passing through. `find_thing` ranks it
after records in place and says it was not found there again, the console
hides it with the gone ones, and a run plans around it no more than a gone one.

The resolver still files to the store's entities.

## What to know when changing it

- **Run the job on the system Python.** Navigation's environment is
  RoboStack's conda Python, with its own numpy and no PIL; version 4 was
  verified on the system one (numpy 1.26.4, scipy 1.11.4). Only `scan_log.py`
  needs ROS, and it joins navigation from `~/ugv/ros_nav`, under bash, because
  `dds.sh` finds its settings beside itself.
- **A record's `seen_at` is not its last sighting**: `model.Memory.merge` keeps
  the survivor's time. `visit_things.py` reports a record's newest look instead.
- **Things above the lidar's plane are rarely marked gone**, and thin ones (the
  step ladder) seldom give the lidar enough hits: their records stay present
  after they are taken away.
- **Changing `deploy/manifest.json` redeploys every component**, navigation
  included; restarting navigation leaves its position unconfirmed until a refit.
- **Everything it writes is on disk before it is renamed into place.** On
  2026-10-17 a power cut left an empty lidar file under a real name, and every
  batch failed on it for the half hour it stayed in the job's buffer, while the
  records went stale and every reader fell back to the store. An unreadable
  lidar file is now skipped as well.
- **The model is pinned to one map session** (`data.MAP_SESSION`, 67). On
  another, the job reads nothing and its records stop answering.
- **Anything stamped before the clock was set is left out live** -- regions and
  lidar alike (`job.CLOCK_SET_AFTER`). A lidar chunk from a cold boot that began
  in 1970 "covered" 1970 to 2026, and filing the history with it took from
  records the lidar hits that let them be seen gone (74 gone, not 129). The
  history keeps its 1970 regions, as the desk's runs do, so a record's first
  sighting is its oldest look with a real time (`visit_things.oldest_look`).
