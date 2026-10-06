# A rover that cannot feel itself turn no longer moves by itself

**The daemon now refuses to open an autonomous run or dispatch a drive, and ends
a run in progress, while the gyro's bias reads exactly zero or beyond 5 deg/s.**
That is what the base reported during all three of the rover's rotation-sensor
faults on record. One of them stalled a trial leg today
([the console stop](2026-10-06-the-console-stop-at-speed.md)). Nothing in a
run's checks had looked at it, and navigation called the position trusted
throughout each fault. [R-SAFE-17](../requirements/safety.md#r-safe-17) is
proposed for it. A look is still allowed, because it turns nothing.

## The record

The base logs its gyro bias every half minute while the rover stands still.
Every reading on the rover since it became a Jetson, 21,528 of them:

| | Readings | Bias |
|---|---|---|
| healthy, from 2026-09-08 on | 20,942 | 0.2 to 0.8 deg/s |
| 2026-10-03, 16:17-17:04 | 96 | +133 to +150 deg/s |
| 2026-10-05, 15:36-17:33 | 236 | exactly 0, the sensor no longer updating ([that day](2026-10-05-frozen-imu.md)) |
| 2026-10-06, after a reboot that did not cut the board's power | 7 | -2,063 deg/s |
| early weeks, 2026-08-31 to 09-08 | 51 | 1.5 to 5.4 either way, single readings |

A full power cycle cleared the second and the third. The first is the
afternoon the console's runs [looked the wrong way](2026-10-03-console-runs-looked-the-wrong-way.md)
and [looped](2026-10-03-the-first-console-run-looped.md). Whether this was why
was not looked into.

## The rule

`permission.rotation_fault`: a fault when the reported bias is exactly zero or
its size is over 5 deg/s; not one when the base has not reported a bias yet.
It is checked when a run is opened, at every drive's dispatch, and on every
watchdog tick of an open run. Replayed over the 21,528 readings, it refuses all
339 taken during the three faults. It also refuses one other, a single -5.4 on
2026-09-02, and no healthy reading since.

## Evidence

- The replay above, run on the rover against its own logs.
- `rover_daemon/test_autonomy.py`: the recorded fault values are refused and
  healthy ones are not. A run is not opened on -2,063, "and says why". A bias
  that falls to zero mid-run refuses a drive, allows a look, and the watchdog
  ends the run and stops the wheels. rover_daemon 1116, ros_nav 574 and
  autonomy 785 passed.
- On the rover, deployed at 4e12efb (rover_daemon, ros_nav and autonomy; their
  suites passed there). The daemon then reported a bias of +0.57 deg/s.
  Asked to open a run with a deliberately invalid budget, it got past the pose,
  map and rotation checks and refused only on the budget, so nothing was
  opened. The deployed rule refuses -2,063. Navigation came back settled. No
  refusal on a real fault has been seen yet; that waits for the next one.

## What it does not do

It reads one number, the base's running estimate. A sensor that froze at a
plausible value would pass. So would one that failed while the rover drove,
until it next stood still. Navigation's own trust in its position is
untouched, and still says "trusted" through a fault. That belongs to
navigation, not to autonomy.
