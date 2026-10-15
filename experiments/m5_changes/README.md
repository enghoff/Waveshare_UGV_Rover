# M5's first measurements: what an unchanged revisit reports as changed

What [M5](../../docs/plans/autonomous-curiosity.md) has to keep rare is a change
reported where nothing changed. The owner's answer on 2026-10-10 was that the
larger things in the flat rarely move unless arranged for a test, so outside a
test almost every reported change would be a false one. These scripts measure
two candidate sources of "it is not there any more" on the recordings and looks
already held. The results are in
[the progress entry](../../docs/progress/2026-10-10-m5-what-reports-a-change.md).

## Looks: the seen-empty signal

The executive marks an aimed look `seen_empty` when the thing's place was in the
depth camera's view and nothing was filed to it. Run on the rover, against the
episode store and a copy of the world store:

| Script | What it does |
|---|---|
| `empty_looks.py` | counts in-view aimed looks filed and seen empty, by day; whether a second look at a thing repeats the first; `--dump` writes them as JSON lines |
| `why_empty.py` | for each empty look, the region nearest the thing's bearing and which of `aimed.choose`'s tests it failed, against the placement the executive saw (the world snapshot its decision names). Copies the world store to `/tmp/wscopy` first |
| `sheets.py` | contact sheets of empty looks: the picture with the nearest region in red, beside earlier crops of the thing |
| `record_strips.py` | six crops of each record, spread over its history, five records a sheet, for labelling what a record is |
| `labels-2026-10-10.json` | the 79 records looked at in view on 2026-10-08 to 10, labelled from those strips by the coding agent: one object, mixed, not an object, a person, unclear |
| `presence_rules.py` | how often a generous "it is still there" rule says so for labelled objects, against a decoy record of another kind of object at the same place |

`why_empty.py`, `sheets.py` take `--only labels.json OBJ` to keep one label.

## Lidar: the map's own disagreement

| Script | What it does |
|---|---|
| `bag_extract.py` | a recording (`ros_nav/record_drive.sh`) as arrays: scans, odom -> base_link, map -> odom. Run on the rover in the ROS environment |
| `control_count.py START [END]` | what the live change watch logged in a window (a run): each change, its size, and when the place was last seen otherwise; on a control, every furniture-sized one is a false alarm unless shown to have moved. A change logged again on growing to furniture size counts once, at that size. Run on the rover |
| `session.py` | one session of [the plan](../../docs/plans/sessions-that-serve-several-milestones.md): `dry` plans every leg with Nav2's planner and says how near each comes to the dining table's rug, nothing moving; `all NAME` records, drives the survey loop (eight viewpoints, two or more a room, turning to face each before setting off, and since session 4 aiming the camera all round from each inside its dwell: `sweep`), waits in the bedroom until `~/.ugv/sessions/NAME.go` exists, opens an autonomy run a loop's length (3.5 min), drives the loop again, goes home and counts each step from the change log. Each step can be run alone; everything it did is in `~/.ugv/sessions/NAME.json`, with the change watch's report at the end of each step, the run's path, the saved map as the session found it and the watch's state after the wait beside it. A loop whose position check disagrees at two stops running leaves `NAME.stop`, and nothing after it drives: the rover stays where it lost its place. `seen NAME LABEL:X,Y ...`, run after loop A, lists the records the session's looks went to at each place to be staged. Run on the rover, with `control_count.py` beside it |
| `move_targets.py WORLD.db GRID.json THINGS.json OUT.png` | where each thing to be staged can be put so that the record-moving step could follow it: 1 to 3 m away, no record that looks as much like it standing nearer (the step's rival rule), clear of the session's legs and stops, in the camera's sight from two living-room stops. Drawn on the map; written for session 4 (`staged/session4-planned.json`) |
| `score_session.py NAME STAGED.json` | a session's counts by [the proposed rules](../../docs/decisions/m5-sessions-are-counted-before-they-start.md): each staged change found, found small or missed in the first step after the staging that came within reach of it (a loop's stops, a run's path); each untouched room a step visited, with any furniture-sized change in it, whether at a known door and whether a later step's report still had it. STAGED.json is written from what the owner said moved; `staged/` keeps each session's |
| `identity_sheets.py RECORD STAGED WORLD.db REC.npz FRAMES OUT` | criterion 4's crops: every region whose line of sight passes within a metre of a staged place, before the staging for where a thing stood and after it for where it went, drawn from the recording's pose where the store kept none, on a contact sheet per place, with a labels template. On the shakedown it gathers 96-100% of the regions Codex labelled by hand for three of the four things and 69% for the fourth, whose misses were in the bad-pose window, under the size floor or after the room was being put back. Run from the repository root on a SQLite backup of the store |
| `identity_count.py RECORD STAGED WORLD.db LABELS` | criterion 4 counted from those labels by [the rules](../../docs/decisions/m5-sessions-are-counted-before-they-start.md): each staged move kept, split or unresolved, and any record holding two things by ordinary matches. On Codex's shakedown labels: 0 of 3 kept, 838 a confident wrong merge, as its audit found |
| `arm_replay.py WATCH.npz RECORDING.npz GRID.json FROM TO` | what one arm of a session found on its own scans: the watch as it stood after the wait, fed only that window of the recording as `change_node` feeds it, so the second arm does not inherit the first's sightings. Split and resumed from a saved watch, scene 1's recording logs the same changes within 0.3 m as straight through |
| `change_detect.py` | hit and pass-through counts per 5 cm map cell for two recordings; cells solid in one and seen through in the other, grouped and kept when seen across more than 20 s. Runs anywhere with numpy and scipy |

```
ssh orin; cd ~/ugv/ros_nav && . ./env.sh; . ./dds.sh
python3 bag_extract.py recordings/bags/m3-final-145539 /tmp/m3-final-145539.npz
python change_detect.py REF.npz TEST.npz --png out.png
```

A recording cut off by a power loss has no `metadata.yaml`; copy it and run
`ros2 bag reindex COPY -s mcap` before extracting.
