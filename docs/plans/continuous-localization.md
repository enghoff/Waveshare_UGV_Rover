# Keeping the rover pinned to its map, scan by scan

**Status:** built 2026-10-21 at the owner's request; offline replays pass; the
floor test passed the same evening, 19 of 19 legs and the position right after 18
([progress](../progress/2026-10-21-the-localizer-on-the-floor.md),
[R-NAV-15](../requirements/navigation.md#r-nav-15) settled). What is left is below.

## Why

On 2026-10-21 the rover got stuck repeatedly in an 80 cm doorway and by the
charger, behind obstacles that were the map's own furniture drawn 20-35 cm off
([progress](../progress/2026-10-21-the-live-layer-and-a-rover-off-by-its-width.md)).
Recorded drives that day showed why the position goes that far wrong:

- **A turn leaves the heading a few degrees out, and the mapper keeps it.** After
  one 127-degree turn on the spot the heading was 4.5 degrees out (the gyro
  over-counted 3, the mapper added 2); 8 m later the believed track was 26 cm to
  the side of the real one, with 50-70% of each scan on the map's walls against
  100% at the true pose.
- **A wheel slip is counted as travel.** Scraping past something, odometry counted
  0.8 m in 4 s that the rover did not move, and later, wedged against the bed,
  1.26 m against under 0.1 m; the position ended 0.8-0.9 m out and stayed there.
- **The mapper does not pull it back.** slam_toolbox in mapping mode matches each
  new scan against the rover's own recent scans, which carry the same error, and
  its distance penalty -- set on 2026-10-12 so that moved furniture could not drag
  the pose 1.33 m -- resists exactly the large correction a slip needs. Only an
  occasional loop closure snaps it back. Its own corrections are small (a median 2
  cm and 1 degree over 736 of them since 10-17) precisely because it does not
  correct what it does not see.

## What the field has learned

[continuous-localization-survey.md](continuous-localization-survey.md) has the
survey with its citations. In short: track against a fixed reference (the stored
map), not a moving one (recent scans); keep it a bounded tracker, never an
automatic global fit; weight long-lived structure over what moves; and expect
the hard cases to be moved furniture and corridors. Routes, as judged against
this rover:

| Route | Verdict |
|---|---|
| slam_toolbox localization mode | same matching as mapping mode; would fix neither failure |
| Nav2 AMCL against the stored grid | replayed here: tuned, as good as the tracker on the clean drives; untried among moved furniture |
| a continuous correlative tracker owning `map -> odom` | chosen; see below |
| tune mapping mode | runs back into the 2026-10-12 trade-off |

The decision is [continuous-localization-route.md](../decisions/continuous-localization-route.md).

## What was built

- `ros_nav/localizer.py`: the tracker, no ROS. Each scan is matched against the map
  in a window of 25 cm and 8 degrees around odometry's prediction, with
  `refit.py`'s field and scorer; returns under 1.5 m count a quarter; one step
  corrects at most 10 cm and 3 degrees; parked, a correction needs three scans
  running; a scan that fits under 55%, or one swept turning faster than 60 deg/s,
  keeps odometry.
- `ros_nav/localizer_node.py`: runs it at 5 Hz and is the only publisher of
  `map -> odom`, at 20 Hz.
- slam_toolbox keeps mapping -- the owner chose on 2026-10-21 to keep the map
  updating itself rather than freeze it -- but takes the localizer's pose as its
  odometry (`odom_frame: map`), keeps its own frame `slam_map`, publishes no
  transform, and its grid is republished as `map`. If its own estimate parts from
  the tracked pose by 30 cm or 8 degrees for 3 s it is paused, so that it adds no
  walls in the wrong place, until navigation is restarted.
- The map keeper hands a restore's and a refit's pose to the localizer
  (`nav_map.hand_pose`). A refit no longer reloads the graph, which on 2026-10-21
  had grown the mapper to 4.5 GB until it was killed for memory.

## Replayed (offline, against recorded drives)

Each scan's best fit to the map within 1.2 m of the deployed pose is the
reference; scans whose best fit is under 85% are left out. Error: median / 90th
percentile; then the share of scans more than 10 cm or 3 degrees out.

| Drive | deployed | tracker |
|---|---|---|
| doorway, after the 127-degree turn | 25 / 27 cm; 81% | 4.0 / 8.0 cm; 6% |
| back to the charger | 7.1 / 11 cm; 42% | 4.9 / 6.8 cm; 12% |
| short tour, 13:04 | 5.7 / 11 cm; 38% | 4.4 / 10 cm; 14% |
| the doorway drive with 0.8 m of slip injected over 4 s | -- | 3.9 / 7.7 cm; 6% |
| the same with a 6-degree gyro error injected | -- | 4.1 / 7.9 cm; 6% |

Among moved furniture (the M5 shakedown, 2026-10-12, 37 minutes): parked at the
end, the refit's answer was (-19.42, -13.71, 140 deg); the tracker held
(-19.42, -13.67, 140) through the 9 minutes the mapper was 1.34 m out. Weighting
near returns fully or by half instead put the tracker 1.1 m out there too. In the
last two minutes of that session the tracker and the mapper disagree by up to
1.2 m with neither confirmed. On the same reference AMCL with tuned settings was
3.9 / 9.0 cm and 6% on the doorway drive; it was not replayed among the furniture.
The first replay lost track on that session and on an injected slip; the
slip turned out to be an artefact of the injection (a slip rotated with the
rover's heading), and the furniture case was what set the step cap and the near
weighting.

A step costs about 9 ms on a workstation.

## What would count as done

Predeclared before the floor test:

1. On a drive that includes the doorway, the dining corridor and the living room,
   the rover's own position check (`measure`) after every stop agrees within 10 cm
   and 3 degrees at least 9 times in 10, against the deployed system's rate on the
   same route measured beforehand.
2. No stop is caused by the live layer marking the map's own walls (no orange
   marks within 0.5 m of mapped walls with the pose checked right).
3. The mapper is not paused during the drive, and the map keeper writes the graph
   as before.
4. CPU for the localizer under 25% of one core on the Orin, and no Nav2
   extrapolation errors from the new transform.

## What is ahead

- The localizer's cost: 24% of a core averaged, most of it Python's share of the
  transform tree and the scan rather than the search.
- Re-anchoring a paused mapper without a restart.
- Whether the near-return weighting costs accuracy in small rooms (the short tour
  went from 5% to 14% out), and whether AMCL among the furniture would do as well.
