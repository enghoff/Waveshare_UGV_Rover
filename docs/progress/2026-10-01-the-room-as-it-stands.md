# The room as it stands: the heading is the largest bearing error now, and identity fails as before

**M0 does not pass, and this drive fails every tolerance its manifest declared.**
Three things are new. The largest is that the rover's belief about its own heading,
rather than the gimbal or the lens, is now the dominant error in a bearing. At the stop
where it only turned on the spot, every look pointed 8 to 23 degrees away from the
taped target. The second is that a rover carried by hand keeps its old position as
confirmed and goes on recording directions from it, which breaks
[R-WS-16](../requirements/world-state.md#r-ws-16). The third is that bare patches of floor
and blown-out window are being placed as things again, so
[R-WS-12](../requirements/world-state.md#r-ws-12) has instances to build against. Identity
fails exactly as it has on every drive.

## How this run was different

The targets were not staged. A scouting loop picked six things out of the room as it
stood, by what they are rather than by how well the rover placed them: the pink bucket,
the black cabinet, the painting above it, the landscape painting over the dining
table, the tissue box and the toolbox on the table. The owner taped each one's
distance from two walls: wall A carries the cabinet, and wall B the landscape
painting and the kitchen doorway. Heights were taped for the raised ones. The
lidar's spin centre was taped from the same walls at a parking spot before the
drive and at another after it. Those two readings alone fix the walls onto the map: a
rotation and a translation, with no fitting to any target. They agree with the
rover's map to 3 cm over 2.2 m.

Claude drove, over the daemon's tool protocol (`drive_to`, `turn_in_place`), with the
owner in the room. At each stop the rover turned to face each target in turn and
waited four seconds for its own looking loop. Everything was frozen before the rover
moved, in `captures/m0-2026-10-01/MANIFEST.txt`, which also records each deviation
from the plan. The recording is `~/.ugv/archive/world-2026-10-01-acceptance.db` with
`frames-2026-10-01-acceptance/`, copied into the same capture folder: 12:46 to 13:02,
154 looks holding 630 sightings, and 71 things. The scoring scripts are kept beside
it.

Deployed: world_state `1af8b6f`, rover_daemon `c4edfc5`, oak_depth `c4c2b67`,
ros_nav `9d1a5c2`. That is the refitted fisheye lens and the OAK on the gimbal rail,
and this is the first drive through both.

## Against the declared tolerances

| declared before the run | mark | measured | |
|---|---|---|---|
| separations between the six targets | all within 0.30 m | 8 of 15; worst 1.48 m | **fail** |
| each target where the tape puts it | within 0.35 m | 4 of 6 | **fail** |
| height above the floor | within 0.20 m | painting over table +0.13; painting above cabinet +0.63; toolbox +0.31 | **fail** |
| ranges land on the target | 70% within 0.5 m | 35 of 54, 65% | **fail** |
| no wrong association in a trusted thing | zero | at least three things confirmed on full frames | **fail** |
| a target is exactly one clean thing | 3 of 6 | 0 of 6 | **fail** |

The four that pass the placement mark are the bucket (6 cm), the cabinet (26 cm),
the landscape painting (25 cm) and the toolbox (17 cm). The tissue box is the one
that wrecks the separations: the rover placed it **1.77 m** from where it stands, from
a single look, while claiming ±0.17 m. That look carried a range of 2.61 m to an
object 0.95 m away, which is the furniture behind it, and a bearing 23 degrees out.

The painting above the cabinet is placed 0.39 m out and 0.63 m too high, which is
not a lens or camera-height question: the camera's height above the floor, re-taped
the same day at 23.5 cm, agrees with the constant the code uses.

## The heading after a turn on the spot

Every look at a target was compared with the direction from the camera to that
target's taped position. That is the measurement
[R-WS-10](../requirements/world-state.md#r-ws-10) has been waiting for, and it does
not lean on the rover's own placements at all.

| | looks | median error | 90th percentile | within 1.5 deg |
|---|---:|---:|---:|---:|
| all looks at targets | 108 | 4.0 deg | 13.5 deg | 22 |
| at a stop the rover drove to | 83 | 3.5 deg | 8.7 deg | 18 |
| at a stop where it only turned on the spot | 25 | 6.7 deg | 15.3 deg | 4 |

The first stop is the clearest case. The rover had stood at the parking spot since
the scouting loop and then turned to face four targets without driving. All eight
of its looks err the same way, by -8 to -23 degrees with a median of -14. The
stops reached by a drive have medians of -2.5 to +3.5.

The same thing was seen directly once. Navigation was restarted mid-drive as the
manifest asked, with the rover standing still. The heading it believed moved from
163.7 to 172.0 degrees, and the drift check straight afterwards found the scan agreeing
with the new heading to 0.5 degrees. So before the restart the rover had been 8 degrees
out of agreement with its own map, and did not know it. The 23 looks taken after the
restart line up with the things placed before it (median 0.3 degrees), so the
restore itself did no harm.

**Why is not established.** `config/slam_toolbox.yaml` folds a scan into the map
after 0.2 rad of turning, so the mapper should have been correcting these turns. The
2026-09-04 measurement, in which a turn on the spot moved the map heading about 15
degrees further than odometry admitted, points at the gyro scale. What would settle
it is cheap: park the rover, turn it in steps, and after each step compare the
believed heading with the drift check's own full search (`map_measure`), which moves
nothing.

## A carried rover keeps a confirmed position

The rover was lifted and carried to another spot to be measured: 2.3 m, and
roughly half a turn round, from where it had parked. Its look at 13:11:54 recorded
six regions with directions computed from the old position, and two of them started
new things. The drift check ran at 13:13:23 and reported that it could not find the
rover "anywhere near here", which is right. Nothing acts on that report, though:
`position_trusted` and `map_settled` stayed true throughout, and those are the two
flags the capture gate reads. Those rows are in the live store only. The acceptance
archive was taken at 13:02, before the carry.

Carried back near the spot, the rover was refitted (`refit_pose`). The scan fitted
the map to 3 cm and 0.0 degrees, scoring 0.953 against a rival at 0.643, and the
second parking reading was taken there.

## Identity

Every one of the 71 things was given a verdict at 200 px, and the doubtful ones were
checked on full frames. Three are confirmed to hold different objects:

- `object:33`, a dining chair, holds a look at the tissue box and two at the dark door.
- `object:70`, the painting above the cabinet, holds three looks at the cabinet
  beneath it and two at other paintings.
- `object:27`, also that painting, holds looks at two other paintings on the same
  wall.

Composite boxes are more doubtful, such as the toolbox with the landscape painting
behind it (`object:53`, `object:67`) or a chair with the bucket at its foot
(`object:68`). Those are not counted as faults here.

The splits are as bad as the merges, and they are what fails the last criterion. The
cabinet is three things, the bucket two, the painting above the cabinet four and the
landscape painting five. The two armchairs, which look identical, are spread over at
least three. No target is exactly one clean thing, so "go back to it" has nothing it
could trust.

## Bare patches

Between three and five placed things are a patch with no object in it:
- `object:9` and `object:63`, strips of floor;
- `object:17`, a single look at a floor strip;
- `object:10` and `object:14`, blown-out windows.

Rugs, lamps and the ceiling fan are excluded, because they are objects. That is the
fault [R-WS-12](../requirements/world-state.md#r-ws-12) was opened for. The
re-review of 2026-09-07 could not reproduce it, and it reproduces here.

## What the drive itself got wrong

- The living-room stops failed: two drives refused to start, and one ended pressed
  against an armchair beside a floor cable. The identity sample is mostly the
  dining end.
- The first edge-of-frame pass over-turned and lost the tissue box altogether. It was
  redone, and the box sat at the frame's edge.
- 56 looks found the depth camera not answering. It switches off after standing
  still and takes seconds to wake, and the facing dwells kept the wheels still for
  that long.
- Turning to face each target is the very thing that exposed the heading error. An
  owner driving by hand turns less often, which may be part of why the drives of
  2026-09-08 passed separations within 14 cm. The fault is still the rover's: its own
  looking loop records looks during and after every turn.

## Requirements

- [R-WS-10](../requirements/world-state.md#r-ws-10) stays `failing`. This is the retaken
  acceptance measurement, and it fails: a median of 4.0 degrees against the tape, with
  the heading after a turn on the spot as the largest term.
- [R-WS-11](../requirements/world-state.md#r-ws-11) stays `open`. The camera's place on the
  rover is measured, 23.5 cm up and 2 cm ahead of the lidar on the centre line, so that
  blocker is gone. Heights against the tape pass for one target of three.
- [R-WS-12](../requirements/world-state.md#r-ws-12) stays `open`, and now has reproduced
  instances.
- [R-WS-13](../requirements/world-state.md#r-ws-13) stays `open`; M0b fails.
- [R-WS-16](../requirements/world-state.md#r-ws-16) moves from `settled` to `failing`: a
  carried rover recorded directions from a position its own drift check had not
  confirmed, and the check's verdict cannot withdraw trust.

## Next

1. Measure the heading error on the spot (above), then fix it. It is the largest term in
   R-WS-10 and it explains the tissue box.
2. Let the drift check's "cannot find the rover" withdraw `position_trusted`, or
   detect a lift, so that the capture gate refuses directions (R-WS-16).
3. Open the depth map saved beside frame `20261001-124649-33a6ab` and find out why a box
   0.95 m away read 2.61 m. It was the first stop after the depth camera had been off,
   so a stale depth map is the first suspect.
4. R-WS-12 now has instances to filter against.
5. The six targets and the two-wall frame can be reused for the next drive as they
   stand. Only the parking readings need taking again.
