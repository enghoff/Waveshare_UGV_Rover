# A gyro that turned a parked rover: navigation now calls it faulty and stops saving the map

**When the gyro reports a turn that neither track shows, navigation now calls it
faulty within three seconds.** It then holds the heading and stops saving the map.
The console's fault line says so, and autonomy will not drive. The same happens
after two seconds of the gyro reporting exactly nothing. Both faults are latched
until navigation restarts, because every gyro fault on record has lasted until
the power was cut. [R-SAFE-17](../requirements/safety.md#r-safe-17) stays
proposed: the new check has not yet met a real fault on the rover.

## What happened

On the 20:38 boot the gyro came up reading a steady 210.6 deg/s with the rover
parked and nothing driving it. The console showed that turn rate at a speed of
zero. Navigation took it for a real turn, because since 2026-10-10 any reading
over 3 deg/s counts as one. It is the only way to see a spin done through the
board bridge, where the tracks run opposite ways and their average count stands
still. So for twelve minutes the believed heading went round about three times a
second. The base logged "gyro bias: not estimated yet (rover not still)" every
half minute, and the live map became a disc of free floor drawn across the
walls.

The saved map was untouched. It had not been written since 12:55, and the
keeper does not save a restored map whose place has not been confirmed. That
restore was never confirmed, because the scan matched nothing near a heading
that kept turning. I first told the owner the rover had probably been moved
while it was off. That was wrong: it was the gyro. The owner rebooted it, and
the gyro came back healthy at +0.42 deg/s.

The existing guard against a faulty gyro could not see it. That guard refuses
autonomy when the learned resting offset reads exactly zero or more than
5 deg/s. Since the reading counted as a spin, it was never learned as an offset,
so there was no number for the guard to refuse.

## The rule

The board already reported each track's own count, but only their average
reached navigation. Navigation now gets both counts, which separates a real turn
from a gyro that is wrong:

- At rest, each count stands still: a whole minute without a tick, recorded
  tonight.
- On the spot, the counts move opposite ways at about 0.18 ticks a degree. That
  was recorded through a 65-degree spin on 2026-10-05. At the slowest turn the
  rover makes, 9.2 deg/s, each track ticks about every 0.6 s.

A turn navigation commanded itself still counts as it comes. A turn it did not
command is held until either track moves, and dropped if none does within
1.5 s. Three seconds of a turn with no track moving latches the fault. So does
two seconds of the gyro's running total not changing at all: healthy, the
longest such run in a still minute was 0.11 s, while frozen on 2026-10-05 it did
not change for the whole 3 s recorded.

Once latched:

- The heading is held.
- `nav_status` carries the reason as `gyro_fault`, and the console's fault line
  shows it.
- Autonomy will not open a run or dispatch a drive.
- The world state records looks without a direction.
- The map keeper saves neither the map nor the pose.

The cost is that a rover lifted and turned in the air is no longer noticed,
however fast it turns. Turning it while it stands on its tracks still counts.

## Evidence

- The fault, reproduced through the real odometry code: without the track
  counts, a parked minute at 210.6 deg/s turned the heading 12,636 degrees, the
  turn rate read 210.6 and no offset was learned. With them, the heading did not
  move, the fault was not yet called at 2.5 s and was called at 3.5 s ("+211
  deg/s"), and once called even a commanded turn moved nothing.
- Real turns through the same code, with the tracks moving: 60 deg/s for 3 s
  read 180 degrees, and 9.2 deg/s with a tick every 0.6 s read about 55. Neither
  was a fault. A second turned in the air was dropped, and it was not a fault
  either.
- Recordings through the same code: 30 s at rest tonight gave no fault, an
  offset inside 0.2-0.8 deg/s and no heading change. The frozen 3 s of
  2026-10-05 gave "stopped updating". Both recordings are test fixtures now
  (`ros_nav/fixtures/gyro-*.json.gz`).
- The map keeper refuses to save while the base reports a fault. The daemon
  refuses a run on that verdict and gives the console the same reason. A look's
  pose carries no direction while it stands.
- Suites: ros_nav 821 passed on the rover, rover_daemon 1248 passed here.
- Deployed at `e3ec2c5` (ros_nav, rover_daemon, autonomy). On the rover the
  board now passes each track's count (-14 and 15, at rest). The base reported
  an offset of +0.40 deg/s with an empty fault, and the console's fault line was
  empty.

## What it does not do

- It cannot catch a gyro that goes wrong while the rover is being driven,
  because a commanded turn moves the tracks either way. A fault is caught the
  next time the rover stands still.
- A gyro frozen at a small non-zero value would still pass.
- The live map can still be smeared if someone drives by hand while the gyro is
  faulty. Only the saved map is protected, and a power cycle brings it back.
- Nothing yet power-cycles the board. That is the owner's fix.

## Also seen

Navigation's restart for this deploy came back unsettled. The pose on disk was
saved at 12:55 facing 108 degrees, and the rover has stood facing about 152
since. The keeper writes the pose only after the rover drives, and a boot that
confirms its place by the lidar alone does not write it either. The position
check fits the scan 93% at the rover's real heading, against 38% where it
thinks it is and 81% for the next-best fit. A refit would put it right, and
would also write the pose to disk.
