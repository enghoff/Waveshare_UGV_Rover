# The identity trial collected both views of the painting and came home

**The prepared visibility trial ran to completion on its twelfth attempt of the
day.** It drove to both viewpoints, turned so the dining-room landscape painting
was in the middle of the picture, took a retained-depth look at each, and drove
home by itself, with the stop verified. It took 66 s. The two views are
1.15 m apart. They are the independent views of one painting that the matching
candidate in [the run plan](../plans/entity-evidence-drive.md) has been waiting
for; whether they include a clear and an occluded view is for the review to
say. Nothing has been scored yet.
R-WS-13 stays open; R-WS-17 and R-WS-18 stay proposed.

## The run

Session `visibility-independent-20261007-12`, trial source db3da99, map
`7da19bef3888`, 70% (11.76 V) at the start.

| | |
|---|---|
| start | (-17.45, -15.42); the leg turn to face B was judged by the scan's heading, navigation's lagging after the turn before |
| B | reached; painting 75° off centre, one turn, then 27.9° off; look `20261007-143743-392a69`, 10 regions stored |
| C | reached; painting 76° off, one turn, then 24.4° off; look `20261007-143804-87fd46`, 10 regions stored |
| home | returned by way of B; stop verified; collection complete; world and navigation recordings complete |

## What it took

Each earlier attempt was stopped by a check stricter than its purpose. Each
was changed with a test and committed before the next attempt.

1. **A freshly booted rover was called "in use"**, because its motor reading is
   empty until the first move (083491a).
2. **Arrival was judged by navigation's heading, which lags after turns.** A
   decisive scan fit now sets the pose: trusted, scoring 0.90 or more, 0.30
   above navigation's own pose and 0.15 above its rival (ef6ab4e, 8761052).
3. **A view needed its heading within 5°**, which the gyro's 9-10% over-count
   cannot meet. A view now counts when the target is within 30° of the middle
   of the picture (8761052). It also gets one turn to centre the target
   (6496e4c), since B was planned with the painting already 27.7° off.
4. **The trial's look was refused while the rover's own was running.** It now
   waits and asks again (753a9ef).
5. **The return had to begin within 60 s**, which two drives, two centring
   turns and two looks do not fit. It is now 90 s (2943c12).
6. **Each leg was planned to arrive at the view heading**, which bent most
   routes past their length check. Legs are now a turn to face, then a straight
   drive, returning the way they came (db3da99).

None of these touched what the trial measures. The views, the looks and the
evidence they record are as the run plan prepared them.

## Next

The scoring is offline and follows [the run plan](../plans/entity-evidence-drive.md):
blind review of the full photographs and stored outlines, frozen physical
labels, then the candidate's replay against the recorded calls. Retention of
existing connections, no clean cross-object connection, and a genuine
clear/occluded painting connection decide it.

Evidence: `captures/visibility-independent-20261007-12/` (diagnostics and world
recording, copied from the rover; the trial's own hashes verified on copy).
