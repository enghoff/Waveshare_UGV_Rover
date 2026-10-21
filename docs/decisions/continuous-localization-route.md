# Why a tracker against the map owns the rover's position, and the mapper does not

**Status:** decided 2026-10-21 (the route); the owner chose the same day to keep
the map updating itself rather than freeze it.

## The question

slam_toolbox in mapping mode has said where the rover is since 2026-08-31. On
2026-10-21 it was 20-35 cm out often enough to strand the rover in a doorway, and
0.8-0.9 m out after wheel slips it never recovered from
([the plan](../plans/continuous-localization.md)). What should keep the rover's
position fitted to the stored map instead?

## Decided

A continuous correlative tracker (`ros_nav/localizer.py`) matches every fifth of a
second against the map, in a window around odometry's prediction, and owns
`map -> odom`. slam_toolbox maps on, taking the tracker's pose as its odometry.

## Ruled out, and why

- **slam_toolbox's localization mode.** Its code tracks each scan exactly as
  mapping mode does, against recent scans through the same penalised window, so it
  would have made both of the measured errors
  ([survey](../plans/continuous-localization-survey.md)).
- **Tuning mapping mode.** Covering a 0.8 m slip needs a search four times as
  wide each way; a weaker distance penalty brings back the 1.33 m drag among moved
  furniture of 2026-10-12; eager loop closure risks welding rooms together.
- **AMCL.** Replayed here with tuned settings it matched the tracker on the clean
  drives (3.9 cm median against 4.0). It was not chosen because the tracker was
  already shown holding the moved-furniture case and injected slips, and has no
  particle noise in its output; it remains the fallback if the tracker disappoints
  on the floor, and is the less code to keep.
- **Freezing the map.** The survey's recommendation, for a stable frame and no
  mapper drifting unseen. The owner preferred the map to keep updating itself;
  the mapper is therefore fed the tracked pose and paused if it parts from it.

## What would reopen it

The floor test in the plan failing on a criterion the tracker is responsible for;
or the mapper being paused often, which would say that keeping it updating needs
more than feeding it the tracked pose.
