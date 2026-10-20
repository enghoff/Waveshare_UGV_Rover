# Why autonomy hardly went to look in the multi-day benchmark: on day 1 it ended before the change watch had reported the moved things, and on day 2 the watch never reported them, because they were moved in its sight

The [first round of the multi-day benchmark](2026-10-20-the-multi-day-benchmark-ends-after-two-days.md)
failed because autonomy hardly went to look. That entry put it down to the
moved bean bags and stool being reported as small changes, below the size a
run would look at. **Re-decided from the rover's own record of each decision,
that is not what happened.** When the runs decided they had nothing left worth
doing, the change watch's report held one change each time, a door. On day 1
the moved things reached a report a second after the run's last decision, and
the run was already driving home; on day 2 the watch never reported the black
bean bag's move or the stool leaving its old place at all.

Both autonomy faults are fixed: a run that has driven now waits for the watch
to report on where it stopped before it ends, and a change smaller than
furniture is now a goal. Day 2's fault is in the change watch and is not fixed.

## How it was replayed

Every deliberation a run makes is kept with a snapshot of what it was made
from (R-AUT-8). The rover's episode store was copied on 2026-10-20 at 18:10,
and each decision in the two autonomy windows was re-decided from its snapshot,
by the code deployed then (`7066b8c`) and by the code changed here. The
deployed code chose what the run had chosen in all four.

| Decision | Report it read | Changes in it | Chose, recorded and replayed |
|---|---|---|---|
| day 1, 07:54:29 | 07:54:14 | a door | a look at record `v495`, across in the corridor |
| day 1, 07:54:53 | 07:54:14, 28 s old | a door | nothing, then home |
| day 2, 07:50:13 | 07:49:46 | a door | a look at record `v1422` |
| day 2, 07:50:28 | 07:50:16 | a door | nothing, then home |

## Day 1: the run ended a second too soon

Arm A started on the charger at 07:54:22, drove 10.8 m to look at a record in
the corridor, and at 07:54:53 found nothing left worth doing. The situation it
decided from was read at 07:54:43, and the newest report then was from
07:54:14. The watch logged the red bean bag's new place and the stool's old
one at 07:54:44, and its report at 07:55:14 held six changes; the run ended at
07:55:22 on its way home. **The watch reports a change once it has 20 s of
evidence for it, every 30 s, so what the rover sees from a new place reaches a
decision up to 50 s after it stops; the run did not wait for any of it.**

Re-decided with the 07:55:14 report in place of the one it read:

| Change in the report | Size | What it was | Deployed code | Changed code |
|---|---|---|---|---|
| gone, (-13.55, -13.09) | 1.15 m | in the corridor, not staged | chosen first | chosen first |
| gone, (-19.06, -16.72) | 0.60 m | the stool's old place | offered | offered |
| new, (-20.05, -13.41) | 0.55 m | the red bean bag at P1 | not offered | offered |
| new, (-17.18, -13.29) | 0.15 m | the stool at P2 | not offered | offered |
| gone, (-12.58, -12.90) | 0.40 m | a door | not offered | refused as a door |
| new, (-10.74, -12.41) | 0.25 m | a door | not offered | refused as a door |

So waiting alone would have taken the run to the stool's old place; waiting
with small changes offered takes it to both moved things' new places as well.
The black bean bag stood at P4 on day 1 as it had at the end of M5 session 8,
so there was no change to report.

## Day 2: the things were moved in the watch's sight

Over the whole of day 2 the watch logged three changes: something 0.25 m
across near P3, where the stool had been put (07:43); a door; and something
0.3 m across near P1, where the red bean bag stood both days (07:50). Nothing
at P4, where the black bean bag had been, nor at P5, where it was put, nor at
P2, which the stool had left.

The owner set the room out by 07:40 with the rover switched on, from 07:33 at
the latest, and parked at the charger, 1.1 m from P4 and 2.6 m from P5; the
restart check ran from 07:37 to 07:38, and the watch's own tallies have P5 seen
clear until 07:39. **The watch compares one visit to a place with the next, and a
visit lasts as long as the place stays in sight**, so a place solid and then
seen through within one visit is neither, and nothing is reported. That fits
everything the record shows, but it is not reproduced: the watch's state saved
at 11:58 has later visits in place of that one. On day 1 the rover was booted
after the room was set out, and the watch reported the moves.

Neither autonomy change helps here: re-decided by the changed code, both day-2
decisions are what they were. In this flat the rover is parked at its charger
far more than it drives, so a move made in its sight is the ordinary case, not
a quirk of the benchmark's protocol.

## Small changes, from the M5 sessions

Over the eight M5 sessions and the drives of 2026-10-17 held on the desk, the
watch reported 63 changes smaller than furniture: 40 within a metre of
something staged that session or the one before, 8 on the four doors the owner
marked on 2026-10-11, and 15 neither, all but one in the corridor and bedroom.
A change in a doorway is now offered and refused as the door; each of the
other 15 would have cost one look.

## What changed

In `autonomy/` (R-AUT-13 held, its text amended for the wait):

- a run that has driven, finding nothing worth doing, waits a nap at a time for
  a report written 20 s after its last drive ended, and decides again; never
  longer than 90 s, and not for a watch that is not counting;
- every change the watch reports is a goal, its value its size; one in a marked
  doorway is refused as the door (`places.json`, `"kind": "door"`), and one
  within 0.8 m of a larger change is taken in by that one's look unless that
  look came before it was seen.

## What this does not show

Nothing drove. Whether a look at a moved thing makes the answer right depends
as well on the world model filing the look to the thing's record and on a
stale record not outranking it, which is the other half of what failed: two
answers were wrong in every state because a record outlived its thing.

## Next

1. **The watch should report a move made in its sight**: judge a visit on its
   recent evidence rather than its whole, so a place solid for minutes and then
   seen through for minutes reads as changed. That is navigation's change
   watch, and wants replaying on recordings with a move made while the rover
   was parked, against the controls, before it is deployed.
2. **A record should give way when its thing is seen elsewhere** (R-WS-17).
3. Then a second round of the benchmark.
