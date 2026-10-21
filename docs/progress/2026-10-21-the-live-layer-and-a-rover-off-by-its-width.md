# The phantom obstacles in the doorway: the map's own furniture, drawn by a rover 20 cm off

**On 2026-10-21 the rover was stuck for five minutes in the 80 cm doorway by the
corridor, and stuck again by the charger, behind lines of orange marks on open
floor. Both were the planner's live layer drawing the map's own furniture back in
as new obstacles, because the rover's place on the map was 20 cm and 3.5 degrees
out in the doorway and 35 cm out by the charger.** Laying the scan on the map at
the small shift that fits it best before judging what is new took the doorway's
14 marks to none in replay, marked all 22 returns of a person put in the doorway's
middle (the deployed rule marked 11 of them), and kept the marks on two bean bags
newly set down on that morning's drive. It does not mend the rover's position,
and the bridge's route watch still reads a position that far out as something in
the way. Mending the position itself is [R-NAV-3](../requirements/navigation.md#r-nav-3)'s
question, and that is the owner's call.

## What happened

From 09:00 to 09:21, 22 drives were started and 4 arrived; 12 ended blocked and
the planner refused a route 14 times. On every day from 10-18 to the morning of
10-21, nearly every drive arrived. No navigation code or setting had changed
since 10-15, and the doorway on the map is the same as on every day since 10-14:
the clearest line through it is 40 cm from the walls at its widest and 32 cm at
its narrowest, the same to the centimetre on all five saved grids.

- **09:03-09:08, parked in the doorway.** The drift check reported the rover 19
  cm off at 09:03:52, and a measure at about 09:07 said 20 cm and 3.5 degrees,
  with 99% of the scan on a wall at the fit against 67% where the rover stood.
  The live layer had 7 to 14 cells marked in a line 0.6-0.9 m to the rover's
  left, 20-25 cm in front of the mapped furniture there. The planner answered
  "start occupied" and "no valid path"; turning on the spot did not mend the
  pose. "Refit to map" at 09:08 moved the rover 17 cm and 3.7 degrees, and the
  marks went to none.
- **09:20, by the charger.** Stopped with 25 to 33 marks, every one 0.30-0.38 m
  from mapped furniture. During the next turn the mapper moved the rover 35 cm
  and the marks went to none at once. This drive followed a daemon restart of
  ours at 09:14:56 (deploying f85cd3d), which cut the odometry for several
  seconds; that restart may have caused this error.
- Two other blocked drives, at 09:13 and 09:14, came with the drift check
  agreeing to 4 cm at 09:13:50. What blocked them is not known: the layer's marks
  were not recorded then.

## A pose that far out is not ordinary

*Corrected the same day.* This section first said the rover is often 10-30 cm
off between corrections. That came from the size of the jumps in `map -> odom`
itself, and a small turn of that frame about the map's origin, 15-20 m away,
moves its translation a long way while moving the rover hardly at all. Measured
on the rover's own pose, across the same seven sessions (736 corrections,
`odom_error.py` in the session's scratch directory), each correction moved it a
median 2 cm and 1 degree, 4-8 cm and 2-4 degrees at the 90th percentile. Turning
on the spot, the correction was 0.3% of the turn at the median, so the gyro scale
is right; driving straight, odometry under-read the distance by 1.3%.

So 20-35 cm is a failure, not accumulation. The one at the charger followed our
daemon restart, which cut odometry for several seconds. The one in the doorway
built up during a single 6 m drive from the living room, 09:03:21-09:03:37,
while the mapper was folding scans the whole way. It was sideways to the
passage, which odometry's 1.3% along the way of travel cannot make. Why the
mapper went wrong there is not known; the lidar log that covered it keeps 40
minutes and had been overwritten by the time it was looked for. The live layer's
margin, 0.10 m plus 0.087 m a metre of range
([2026-10-09](2026-10-09-live-layer-marks.md)), covers a 20 cm error only beyond
about a metre. Parked, or turning on the spot, nothing corrects the position,
and a rover the planner will not move cannot drive far enough for the mapper to
put it right.

## The change

Before marking, the layer searches shifts of the whole scan of up to 0.40 m and
6 degrees about the laser, at 5 cm and 1.5 degrees, for the one that puts the
most returns within a cell of a mapped wall. When that beats the believed pose by
20 returns and a tenth of the scan, every return is judged new or mapped at the
shift instead. Marks still go where the believed pose puts them, which is where
the planner thinks they are. Judging only at the shift matters: judged at both
poses, the simulated person's returns 40 cm from the doorway's walls came out as
11, not 22, because a pose 20 cm out puts half of them within the wall margin.

Replayed (`align_replay.py` and `incident1.py`, in the session's scratch
directory, against the grids in `tmp/`):

| | deployed rule | aligned |
|---|---|---|
| the 09:05 doorway scan | 14 marks | 0 (shift 20 cm, 126 to 235 of 235 returns on walls) |
| ...with a person in the doorway's middle | 11 of the person's 22 returns | 22 of 22 |
| `insight-1021` drive, by the two new bean bags | 31.7 marks a scan | 31.6 (shift believed on 37 of 890 scans) |
| ...near mapped walls / open floor | 1.98 / 4.42 | 1.99 / 4.40 |
| `md-day2-1020` drive, near walls / open floor | 4.69 / 10.01 | 4.65 / 9.51 |

On an ordinary drive the shift is rarely believed and changes almost nothing; it
is for the rover that is clearly off. The C++ test (`test_live_layer.cpp`) has
a doorway scanned 20 cm off: the old rule marks its walls, the new one marks
none, finds the 20 cm to a cell, marks a person in the doorway's middle, and
with the pose right believes no shift and marks only the person. Built and
tested on the rover (`behaviors/build.sh`, both tests passing, no undefined
symbols). `align_window_m: 0` switches it off.

A successful refit now also asks the lidar again at the next tick. The drift
verdict from before the refit had kept the console's "vs lidar" row at "OFF BY 19
cm" and the change watch paused for five minutes after the rover was put right.

## What it does not do

- It does not move the rover on the map, so the planner still plans from a
  wrong start, and a rover within 20 cm of a mapped wall at its believed pose is
  still "start occupied" (the bridge backs off once).
- The bridge's route watch checks the next metre of route against the
  controller's own costmap, which is the real room around the rover. With the
  rover 25-35 cm out in an 80 cm doorway, the route drawn through the doorway's
  middle runs into the real frame, and the watch stops the goal as "something
  in the way".
- Correcting the position automatically would close all of these, but
  [R-NAV-3](../requirements/navigation.md#r-nav-3) makes fitting a person's act.
  A refit now also costs 20-40 s, because the pose graph has grown to 234 MB
  plus 133 MB of scans; it was 163 MB on 10-11. Graph saves, once a minute while
  driving, hold the mapper for about 5-20 s each, read from gaps in the log.
