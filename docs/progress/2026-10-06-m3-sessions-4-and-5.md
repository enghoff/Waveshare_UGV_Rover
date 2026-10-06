# M3 sessions 4 and 5: 44 drives and none failed; the battery, not the run, ended both

**Two more fenced sessions ran in the charger room after the walk began taking
the bridge's goal fit ([the change](2026-10-06-no-room-for-the-body.md)).
None of their 44 drives failed, against 3 of 19 in
[session 3](2026-10-06-m3-session-3.md) and 7 of 87 actions in session 2.**
Both were ended by me on the battery: session 4 on a reading that turned
out to be sag under load, session 5 at 30% resting. Neither ran out of things
to do. Not one of their 46 looks improved a placement, as the
[record predicts](2026-10-06-looks-seldom-reach-their-thing.md).
[R-SAFE-16](../requirements/safety.md#r-safe-16) held. The owner was present.

## The runs

Both were opened over the protocol by the charger, at (-17.1, -15.9), with
900 s, no travel or action limit, and the same safe area as session 3. Both ran
at 0b89519 for autonomy, with the 0.5 m margin of the time.

| | Session 4 (`run/be1da2f4/7`) | Session 5 (`run/be1da2f4/8`) |
|---|---|---|
| Episodes | 801-839 | 840-846 |
| Lasted | 496 s, 21.1 m, 79 actions | 90 s, 8.6 m, 12 actions |
| Geometry goals | 39 | 7 |
| Drives | 38, none failed | 6, none failed |
| Looks | 19 stored, 16 the same picture, 3 refused | 3 stored, 3 the same picture |
| Placements improved | 0 | 0 |
| Ended | `stop_driving` from me: the battery read 10% | `stop_driving` from me: 30% at rest |

Three looks in session 4 were refused because "an inspection has been running
for 6 s". The rover's own look outran the five seconds an autonomous look waits
for one in progress. Each ended its goal, never two in a row.

## The battery

Session 4 began at 65% after the morning's trials. My watch stopped it on
three readings of 10%. At rest the pack read 40% (11.38 V), so the readings
were sag while driving. Session 5 began at about 30% resting and read 10% at
rest a minute in. I stopped it there rather than run towards the cut-out, and
drove the rover back to the charger. The two sessions and the trials before them
took the pack from 80% to 15-25% in about 25 minutes.

## What it means

M3 now stands at five sessions and about 33 minutes of autonomy time, against
20 sessions and 120 minutes. Session 5's minute and a half barely counts as a
session. Failed drives, the thing that ended sessions 1 and 3 early, did not
happen once in 44. What ends sessions now is the battery and a supervisor
watching it. A reading taken while driving says nothing about how much is left.
The resting voltage, or one averaged over several seconds, is what to watch.
