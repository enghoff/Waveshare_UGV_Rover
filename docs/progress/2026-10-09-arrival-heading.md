# The parked rover's wrong heading was its own last turn, not a hand on it, and a refit that worked said it had not

**Nobody turned the rover by the charger. Each time it came back there, the turn
it made onto the parking heading left its heading 14 to 16 degrees out, and
standing still it kept the error.** The owner says nothing was moved, and the
looks' own heading checks put the error there within seconds of each arrival.
The entry on the live layer's marks
([2026-10-09](2026-10-09-live-layer-marks.md)) said the owner had turned the
rover to plug it in. That was a guess, and it was wrong. The fault is the one
measured on [2026-10-01](2026-10-01-heading-after-turning.md) and
[2026-10-07](2026-10-07-identity-trial-stopped-on-arrival-heading.md): a turn on
the spot is miscounted, and a parked rover folds no scans that would correct it.
Separately, "refit to map" corrected the rover and reported that it had not. The
fix for that report is 0fb2d05.

## When the heading went wrong

Every look checks its heading against the map
([headingcheck.py](../../world_state/headingcheck.py)), so the store holds what
the navigator believed beside what one scan said:

| | arrived by | navigator minus scan | scan on a wall |
|---|---|---|---|
| 11:51:58 | `run/b21221da/1` returning to its start beside the charger | +14.0 deg | 98.4% |
| 13:12:26 | `drive_to` the charger spot, 7 s after it arrived | -15.5 deg | 98.7%, against 57.6% at the belief |

The second time, the navigator read 131.3 degrees the moment `drive_to`
returned, at 13:12:20. By 13:12:26 it read 108.1, and it then held 108.1 until
the refit. The 23 degrees came in the first six seconds after "arrived", with
the wheels reporting still since 13:12:19: the end of the turn reaching the
navigator, not a parked rover drifting. No recording was running, so whether the
gyro or the mapper made it is not known. [R-NAV-13](../requirements/navigation.md#r-nav-13)
holds: once the error was made, it did not grow.

The parked check of [R-NAV-14](../requirements/navigation.md#r-nav-14) caught
it: at 13:13:54, "the rover thinks it is 2 cm and 15.5 degrees from where the
scan fits the map -- 98% of it lies on a wall there against 56% here". The
morning's 14-degree case was cleared by the navigation restart at 12:08, whose
restore re-anchored the rover.

## What it costs

- **The live layer's marks.** The 56 cells kept marked along the walls by the
  charger that morning were this error. The range-scaled margin deployed then
  covers 5 degrees, not 15.
- **Not the M4 looks.** An aimed look measures the heading against the map
  before it works out the pan (`rover_world._aim_pan`), and each look's bearing
  takes the scan's correction. So neither the re-look nor the chosen viewpoint's
  look aimed from the wrong heading.
- **Navigation.** It corrects itself after about 0.7 m of driving
  (2026-10-07), so a run opened on such a pose starts its first route from the
  wrong heading.

## A refit that worked, reported as one that had not

"Refit to map" was pressed at 13:15:26, not by this session
([R-NAV-3](../requirements/navigation.md#r-nav-3)). It answered "the scan fits
the map 2 cm and 15.5 degrees from here, but the mapper matched it against its
own graph and kept the rover where it was", not fitted. The rover then read
123.8 degrees, and at 13:18:55 the parked check said "the scan agrees ... to
within 3 cm and 0.0 degrees". The note on disk still said 108.1, so a restart
before the rover next drove would have restored it from the wrong heading.

The load counted the rover as landed as soon as it stood within 0.5 m and 20
degrees of the pose handed over. A rover 15.5 degrees out already does, before
the mapper has processed the next scan. So the load returned on the old pose,
and the fit was read as no move. When the rover starts near the pose asked for,
the load now waits up to five seconds for the pose to move. The test in
`ros_nav/test_maprestore.py` gives "not fitted, turned 0.0" on the old code, as
the rover did, and fitted by 15.5 on the new; selftest 652 passed. The same
wording on 2026-10-07 ("a refit was refused") and the angle mismatch noted on
2026-10-01 may have been this. Neither was re-examined.

## Next

Turning onto the parking heading is what leaves the error, so a rover that
must park facing a set way could end its drive with a short straight move. That
is not proposed yet. The pre-run check that reads `map_drift`, proposed in the
live-layer entry, would have caught the second pose. The restart cleared the
first before any run opened on it.
