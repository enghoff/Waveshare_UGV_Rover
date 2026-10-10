# A turn on the spot no longer leaves the heading wrong: the mapper never looked, the scans were bent, and odometry was late

**After a turn on the spot the rover now knows its heading to about a degree.**
Twelve turns at the charger on the fixed code left the navigator 0 to 2.5
degrees from where a still scan fits the map, 1.1 typical. The same turns that
morning had left it 5 to 16.5 degrees out, adding up from turn to turn.
Arriving at a goal with a final turn, which is where the fault showed
([2026-10-09](2026-10-09-arrival-heading.md)), now ends within half a degree.
There were three causes, each reproduced in a replay of the recorded turns
before it was changed: slam_toolbox ignored turns on the spot, scans taken while
turning were bent, and odometry was stamped late. Two more faults came to light
along the way: the gyro's scale was 5.5% out, because its calibration had been
checking it against itself, and a spin the base node did not command was
counted as standing still. Both are fixed and the gyro re-measured. While
re-measuring it at the charger I left the rover's pose 0.7 m and 48 degrees out,
and the owner refitted it. [R-NAV-3](../requirements/navigation.md#r-nav-3) and
[R-NAV-13](../requirements/navigation.md#r-nav-13) hold.

## What was wrong

Twelve turns on the spot at the charger, recorded (`ros_nav/record_drive.sh`),
each followed by a still scan fitted against the map. Then every one of the
2,669 scans was fitted against the map at the rover's measured position
(`captures/2026-10-10-turns/turn_analyse.py`), so that odometry, the mapper and
the walls could be compared scan by scan.

- **The mapper never corrected a turn.** Its `map -> odom` correction did not
  move by 0.05 degrees in four and a half minutes of turning. slam_toolbox 2.8.5
  takes a scan only once the rover has moved 0.16 m, whatever it has turned,
  unless `check_min_dist_and_heading_precisely` is set. So the heading after a
  turn was odometry's, until the rover next drove.
- **The gyro read 6.2% long,** against the scans.
- **The rover turns at up to 120 degrees a second**, where Nav2 asks for 29.
  The D500 sweeps once in 0.1 s, starting at the rover's left and going
  clockwise, so a scan taken mid-turn is bent by 12 degrees.
- **A scan was 48 ms later than odometry said,** in proportion to the turn rate:
  odometry is stamped as it arrives from the board, after the delay of the board
  link.

## Reproduced, then changed

Each recording was replayed into a second slam_toolbox on its own DDS domain,
with the live correction stripped out and the scans and odometry altered offline
(`bag_filter.py`, `turn_replay.sh`). Each line below is the heading error after
each of the twelve turns, against the scans' own fits.

| Replay | Typical | Worst |
|---|---|---|
| as deployed | 9.2 | 15.7 |
| the heading rule on | 4.8 | 10.2 |
| ... and odometry 48 ms earlier | 2.1 | 4.5 |
| ... or each scan put back together | 2.4 | 5.4 |
| ... both | 0.6 | 1.0 |
| ... both, and the gyro rescaled | 0.6 | 1.1 |

The first line matches the rover turn for turn, to the tenth of a degree. So
the replay fails the way the rover did. Deployed as 35230e6:
`check_min_dist_and_heading_precisely: true`, `lidar_node.py` rotating each
point by how far odometry turned before its moment (`scan_deskew.py`), and
`base_node.py` stamping odometry 48 ms earlier (`ODOM_LATENCY_S`). On the rover:

| Twelve turns | Typical | Worst |
|---|---|---|
| before, at the charger | 9.2 | 16.5 |
| after, at the charger | 1.1 | 2.5 |
| after, gyro re-measured, in the bedroom | 1.2 | 3.0 |

Two goals, turning 155 and 14 degrees in all, from the living room to the
bedroom and within it, both ended within half a degree. Neither had a doorway
refusal.

## The gyro, and a spin nobody commanded

The gyro's scale had been checked on 2026-08-23 against slam_toolbox's heading
through each turning burst. On the spot that heading was the gyro's own plus a
constant, so the check found 1.0, 0.96, 1.01 and 1.03. `calibrate_chassis.py`
now fits a still scan before and after each burst (d8dbd77, 8fc4e22). Six bursts
of about 300 degrees put the gyro 5.5% long, at 1.048 to 1.073, and the scale
went from 15.31 to 16.15 LSB per degree a second.

The first attempts found something worse. The calibration spins the rover
through the board bridge, which `base_node` neither commands nor sees in the
wheels, because the tracks turn opposite ways and their mean count stands still.
It counted that as a rover at rest: each burst's rotation was thrown away and
learned as gyro offset, which reached 2.0 degrees a second. A hand turning the
rover on its tracks would be the same. A turn rate 3 degrees a second above the
offset now counts as moving (5b24d72). `test_odometry.py` drives the real
`debias` through such a spin: before, 0 degrees and an offset of 3.6; after, 180
degrees and the offset untouched.

Running the calibration at the charger, before that was found, was my mistake.
One burst turned the rover 180 degrees to face the wall. Turning it back,
odometry added the burst it had thrown away, 1.26 m of travel at once. That left
the pose 0.7 m and 48 degrees out, and the owner refitted it.

## Also found

- The rover turned 32 degrees on the charger between 08:40 and 09:38 with no
  command sent; the gyro and the scan both saw it.
- It still overshoots a turn by 5 to 15 degrees by its own odometry, now in a
  heading it knows. Nav2 asks for 0.5 rad/s and the base turns at about 2.

## Requirements

None moved. [R-NAV-13](../requirements/navigation.md#r-nav-13) holds: a still
rover still integrates nothing, and only a turn the gyro reports above its noise
counts as moving. [R-NAV-3](../requirements/navigation.md#r-nav-3) holds. The
refits here were a person's, and the mapper's own correction does the rest.
