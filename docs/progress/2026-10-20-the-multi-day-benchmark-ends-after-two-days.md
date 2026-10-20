# The multi-day benchmark's first round ends after two days: autonomy could no longer answer better than the fixed loop, so it fails, while the memory held across the restart

The first round of the [multi-day usefulness
benchmark](../plans/autonomous-curiosity.md#multi-day-usefulness-benchmark) was
judged by [rules set before the first day](../decisions/the-multi-day-benchmark-is-judged-so.md).
It ran on 2026-10-19 and 2026-10-20 and was stopped there, on the owner's
decision of 2026-10-20, because the third day could no longer change the
verdict. **It fails criterion 3, autonomy answering better; the other three
held.** That leaves M5 standing and the multi-day usefulness claim unproven.

## The two days

| | Day 1, `md-day1-1019` | Day 2, `md-day2-1020` |
|---|---|---|
| Time, order | 07:54-08:04, autonomy then the loop | 07:41-07:50, the loop then autonomy |
| Staged (as asked both days) | red bean bag P1, black bean bag P4, stool P2 | red bean bag P1, black bean bag P5, stool P3 |
| Autonomy (arm A) | drove 10.8 m in 60 s, looked at the large change the watch reported by the corridor, and ended with nothing left over its 0.15 minimum; the bean bags and stool were reported as small changes, not goals | did not move: nothing left worth doing, its three candidate records cooling off after looks that had not improved them, the small changes not goals |
| The fixed loop (arm B) | 8.5 min, 16 stops, every position check trusted | 8.3 min, two laps, 16 stops, 18 position checks all trusted |
| Right of 9: morning, A, B | 6, 6, 8 | 6, 7, 8 |

Day 2 began on the charger with the restart check: all nine answers were the
same after the daemon and the world model's job restarted. Each day's rows are
in [the ledger](README.md); the answers are in `experiments/multiday/days/`.

## Against the four criteria

1. **Retained evidence answers after the restart: met.** Every one of nine
   answers was identical across the day-2 restart.
2. **Stale coordinates drive nothing: met.** No goal the autonomy arm chose was
   at a record the morning held as seen gone or set aside.
3. **Autonomy answers better: fails.** Over 18 questions, arm A has 13 right
   and 3 wrong, arm B 16 right and 2 wrong; the bar over 27 was A at least 3
   ahead with no more wrong. The six control questions were answered right by
   every arm in every state on both days, so a day can move the gap by at most
   the three moved things: A could at best have drawn level. Only arm B
   answering three or more controls wrong on day 3 could have changed that, and
   that would be the loop failing, not autonomy answering better.
4. **Nothing gets less safe: met.** No stops or interventions in either arm.

## What it shows

Of the six questions about moved things over the two days, the loop answered
4 right, autonomy 1 and the morning's records none. The memory was not what
failed: it held across the restart and answered the controls every time.
Autonomy lost because it hardly went to look. It counted the moved bean bags
and stool as small changes, below its minimum, and stopped looking at records
whose looks had not improved them, so it drove for 1.4 minutes over two days
against the loop's 16.8.

Two answers were wrong in every state, including the loop's, because the
record outlived its thing: the black bean bag on day 1, answered from a record
two days old, and the stool on day 2, answered from its record at day 1's P2
although the loop's change watch reported something new at its new place.

So the next work is twofold. On the autonomy side, a moved thing must become
something worth a look. On the world-model side, a record must give way when
its thing is seen elsewhere, which is the plan's existing item on records that
outlive their things (R-WS-13, R-WS-17, R-WS-18).

Reported alongside, without a bar: moved-or-not answers right, autonomy 0 of 3
and the loop 1 of 3; battery per arm 17% and 6%, which are not comparable,
since the first arm each day takes the drop off the charger.

## Why the third day was not run

The decision asked for three days. After day 2, criterion 3 could pass only if
the baseline broke, so a third day would have cost a charge and about 45
minutes of the owner's morning to confirm a verdict already fixed. The owner
agreed to stop. The departure is from the decision's length, not its rules,
and is stated here so the round is not read as having run three days.

R-AUT-13 held on both days. No requirement changed state.
