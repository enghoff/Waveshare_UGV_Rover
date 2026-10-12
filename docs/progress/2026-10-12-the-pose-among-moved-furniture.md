# The pose among moved furniture: reproduced in a second mapper, and a scan-matcher setting that stops it

**The 1.33 m error of the M5 shakedown
([2026-10-12](2026-10-12-m5-shakedown.md)) is the mapper's own scan matching,
and a second mapper fed the session's recording from the graph the session
started with makes it again, to 1 cm.** No loop closure and no single jump:
among the living room's moved armchairs and bean bags, each scan's best match
moved the pose 5-30 cm and a few degrees, and the matcher applies its best
match however poor. With the matcher charging for distance from where odometry
puts the rover, the same replay ends 2 cm from the refit's answer. That setting
is deployed. It is shown on replays only; the rover has not yet driven with it.

## Reproduced

The session script kept the mapper's graph as the session found it
(`~/.ugv/sessions/shakedown-1012-map/`). A second slam_toolbox on its own ROS
domain loaded it at the pose the recording starts from and was fed the
recording, with the live mapper's `map -> odom` taken out and the lidar's
mounting published (a replay started part way in never plays the recording's
one `/tf_static`). Its pose against the live mapper's: within 2 cm through the
first loop, the wait and the run's first eight minutes; the same walk from
08:00:58; the same end, (-19.47, -15.05, 131) against the refit's
(-19.42, -13.71, 140). From a graph saved at 08:00:11 the same replay gave 1.33 m.

| Matcher setting, from the 08:00:11 graph | End, from the refit's answer | Turns of 2026-10-10, heading error rms / worst |
|---|---|---|
| as deployed until now (distance 0.5 / 0.5, angle 1.0 / 0.9) | 1.33 m | 2.6 / 3.3 deg |
| distance and angle 0.1, floors 0.2 / 0.5 | 1.12 m | |
| distance and angle 0.05, floors 0.1 / 0.3 | 0.02 m | 5.9 / 8.2 deg |
| search window 0.3 m (from 0.5) | 0.02 m | 2.6 / 3.3 deg |
| angle search 10 degrees (from 20) | 1.27 m | |
| **distance only 0.05, floor 0.1** | **0.02 m** | **2.3 / 2.9 deg** |
| distance only 0.1, floor 0.2 | 1.13 m | 2.4 / 2.9 deg |

The turns column is the twelve turns on the spot of
[2026-10-10](2026-10-10-turning-heading.md), replayed from an empty map with the
harness of `captures/2026-10-10-turns`: what the gyro gets wrong in a turn is for
the matcher to correct, and penalising the heading stopped it doing so.

## The whole session with the chosen setting

Replayed from the start graph through all 37 minutes, distance only (0.05, floor
0.1) tracks the live mapper within 15 cm everywhere the live mapper was right,
and is right where it was not: at 07:58:09 entering the living room it fits the
map at 0.93 against the live 0.13, and through 08:01:30-08:02:00 it wobbles
0.6-0.9 m before settling on the truth by 08:02:20, where the live mapper went
on to 1.33 m. A search window of 0.3 m did as well here, but it caps every
correction at 15 cm a scan, and the recordings of 2026-10-10 and 11 hold up to
eleven corrections each of more than 15 cm, the largest 33 cm.

**What it costs.** It resists a large correction whether that correction is
right or wrong. Driving home at 08:17:44-56, beside the armchair that now stood
by the charger's spot, every replay and the live mapper alike went 0.57 m out;
the live mapper then corrected 0.35 m of it in one scan, and this setting would
not. It ended 0.47 m from where the position check put the rover, against the
live mapper's 0.21 m. That slip itself is not explained.

## Deployed with it

- The position check's answer carries `far`, the same search on the returns
  beyond 1.5 m (3c34299), for the heading check's changed-room verdict
  ([2026-10-12](2026-10-12-heading-check-in-a-changed-room.md)).
- The replay tool kills its own mapper (dd34179): `ros2 run` does not pass a kill
  on, and each replay had left a mapper running on domain 43.

Still to show: the setting on the rover's next drives, and whether the session
script's two-stop rule or the periodic drift check
([R-NAV-14](../requirements/navigation.md#r-nav-14)) catch the next slip.
The faults stay reproducible without the rover
([R-NAV-11](../requirements/navigation.md#r-nav-11)) as long as each session
keeps its starting graph.
