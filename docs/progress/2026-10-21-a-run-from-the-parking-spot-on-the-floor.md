# On the floor, a run started from the parking spot went straight to a bean bag the watch had seen from there only, and the watch confirmed it after the look

The morning's fix had been checked in replay and on the rover's own report,
not on the floor: since 66290c4 the change watch publishes what it has seen
from one place only, and a run looks at each such sighting from 0.7 m or more
away, so that the watch gets its second place
([2026-10-21](2026-10-21-a-run-from-the-charger-has-something-to-look-at.md)).
**On the evening of 2026-10-21 a bean bag was set down at P2 in the parked
rover's sight. The watch held it as seen from one place, the run's first goal
was a look at it from beside the parking spot, and after that look the watch
reported it as a new thing 0.17 m from where it was put.** The run then found
nothing else worth doing and went back.

## What ran

The rover stood on its parking spot between the two armchairs, restarted at
22:53 after dropping off the network at 18:08 (its computer and driver board
at once; why is not known). Its position was trusted and the map settled, the
gyro clean, the GPU up. The owner set a bean bag down at P2, the dining
table's corridor end on the living-room side, 2.7 m in front of the rover, at
about 22:57. The criteria were written down before the run.

At 22:58:15 the watch's report held one sighting from one place: new,
0.70 m across, at (-17.31, -13.52), 0.18 m from P2. Nothing was in its
`unconfirmed` list before. The owner unplugged the rover, and an agent opened
the run at 22:59:16 (`run/28444258/1`, 600 s, the standing limits otherwise),
the battery at 95%.

| Time | What the run did |
|---|---|
| 22:59:22 | drive to a look from (-17.78, -15.84), 0.75 m from the start, "2.4 m from the change" |
| 22:59:24 | arrived at (-17.62, -15.79), 0.60 m from the start; position trusted, 4.5 cm off by the drift check |
| 22:59:24-30 | looked at (-17.33, -13.50) with the gimbal camera |
| 22:59:30-46 | waited for the watch's next report after the drive, with nothing else over the 0.15 minimum |
| 22:59:44 | the watch: NEW at (-17.32, -13.51), 0.75 m across, seen from places 0.68 m apart |
| 22:59:47 | nothing left worth doing; drive back to the start |
| 22:59:51 | arrived at (-17.20, -15.82), 0.18 m from the start, heading 33 degrees from the start's; position trusted |

The run took 42 s, 1.4 m and 3 actions. Nobody stepped in.

## Against the criteria

1. **Goes to look: met.** The first goal was the one-place sighting, from a
   place chosen 0.75 m from where it was seen. The rover stopped 0.16 m short
   of it, as a drive's arrival allows, so it looked from 0.60 m away. That was
   enough for the watch, which needs 0.5 m.
2. **Confirmed: met.** The watch reported the bag 0.17 m from P2 once it had
   seen it from the second place.
3. **Does not end at once: not shown either way.** The run acted before it
   ended, and its sentences carry no thing refused as having nowhere to stand
   (3e3e4c1). But everything else it weighed would have gained 0.03 or less,
   so the flat offered little to refuse.
4. **No phantom passage: not exercised.** Nothing was logged at the passage by
   the corridor, but the run never went near it (b54f8fa).
5. **Navigation under autonomy: met for two drives.** Both arrived, with the
   position trusted after each and the rover home within 0.3 m. One drift
   check, at 22:59:40, could not fit the scan well enough (88% of it on a wall
   against the 90% it needs); the position stayed trusted. Home, the
   rover faced 33 degrees off its starting heading, more than the 6-16 degrees
   arrival turns usually leave.

No requirement changed state. R-AUT-13 held: a run with nothing left worth
doing went back to where it started and ended.
