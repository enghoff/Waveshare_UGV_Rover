# The localizer on the floor: 19 legs, all arrived, the position right after 18

**With the rover's place on the map kept by the new localizer
([plan](../plans/continuous-localization.md)), a 19-leg drive through the
doorway by the corridor, the dining corridor and the living room's armchairs
arrived every time, and after 18 of the 19 stops the position agreed with the
lidar within 10 cm and 3 degrees.** On the same flat that afternoon, with the
mapper owning the position, 24 of 40 legs arrived and the position agreed after
16 of them. The success predicate set before the drive -- 9 stops in 10 --
is met ([R-NAV-15](../requirements/navigation.md#r-nav-15)).

## The drive

16:41-16:48, deployed at `999e019` (localizer `f67648c`, its CPU fix `57e55a7`),
from the charger at 95%, recorded as `floor-1021-1`. Legs J C G D F, K D F D L M,
G J C G D K L R from the tour goals (`tour.py`, the bedroom bed left out by the
script, not by the rover); 59 m driven.

- **Arrived:** 19 of 19. K went round something in three straight legs once; G
  waited once for something in the way; several goals were moved 5-16 cm by the
  goal fitter.
- **Position after each stop** (the bridge's `measure`, or the refit's search
  when the map keeper was writing the graph): 0-2 cm and under 1 degree at 18
  stops; at D after the doorway, 6 cm and 3.5 degrees, which the refit -- now a
  pose handed to the localizer, not a graph reload -- put right at once.
- **No phantom obstacles:** the live layer had no marks at 17 stops; 15 at M,
  among the armchairs, and 1 at the charger.
- **The mapper kept in step:** never paused; its own estimate 3.3 cm and 0.1
  degrees from the tracked pose at the end. The keeper wrote the graph as before.
- **The change watch:** 19 scans over the drive refused as fitting the map better
  shifted, where the afternoon's tour had 886 in 46 minutes.
- **Timing:** no extrapolation errors from Nav2; one from slam_toolbox, 41 s
  behind while the graph was being written, as before this change.
- **Cost:** the localizer averaged 24% of one core over 33 minutes, parked and
  driving together; a search is 32 ms on the Orin, and most of the rest is
  Python's share of the transform tree and the scan.

## Not shown

One drive, one afternoon, in a flat that had been driven all day. Moved
furniture was shown only in replay (the 2026-10-12 session). The bed, which the
lidar cannot see, was avoided by the test script and is not something the rover
avoids.
