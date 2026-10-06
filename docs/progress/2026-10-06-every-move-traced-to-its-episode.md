# Every move the rover made under autonomy traces back to the episode that asked for it

**Across the five M3 sessions, navigation's own log of every move it made was
matched against the episode record. Every move made while a run was open
matches a `drive_to` that a recorded episode dispatched.** The only two moves
with no episode were my hand drives back to the start between runs, after one
run had ended and before the next opened. Every failed action was followed by a
fresh recorded decision, never by a retry without one.
[R-SAFE-9](../requirements/safety.md#r-safe-9) is settled. M3's criteria 6 and
7 are met on the record.

## How

`trace_moves.py`, kept in `captures/m3-trials-2026-10-06/` with its output.
It ran on the rover against two records written independently. One is the
episode store: every call an executive made, with its episode, time and answer.
The other is the navigation bridge's ROS log, which names each `goto`, `turn`,
`drive` and `explore` it carries out, with its outcome, whoever asked. Episodes
were grouped into sessions by gaps of over 150 s. That joined sessions 4 and 5,
and the two runs of session 1. Each logged move inside a session was paired
with the nearest unpaired `drive_to` within 5 s.

| Sessions | Episodes | Drives recorded | Moves logged | Paired | Unpaired moves |
|---|---|---|---|---|---|
| 1 (two runs) | 695-733 | 39 | 39 | 38 | 1, between the runs |
| 2 | 734-778 | 44 | 43 | 43 | 0 |
| 3 | 780-800 | 20 | 20 | 20 | 0 |
| 4 and 5 | 801-846 | 44 | 45 | 44 | 1, between the sessions |

**The two unpaired moves** came 56 s after run 1 of session 1 ended on its
action limit, and 28 s after I stopped session 4. Each was 9-11 s before the
next run's first episode. They are the hand drives back to the start that
[session 1](2026-10-05-m3-session-1.md) and
[sessions 4 and 5](2026-10-06-m3-sessions-4-and-5.md) describe, with no run
open. **Two recorded drives have no logged move within 5 s** (episodes 725 and
755). Both timed out, so the bridge's line for each lies further from the
executive's record of it.

**After each of the 21 failed actions**, the next thing to happen was a new
episode, opened by a recorded decision with its candidates and reasons. In
three of them, all in session 4, the decision chose the same thing again. Each
time the failure was a look refused because the rover's own look was still
running ("an inspection has been running for 6 s"), so nothing had been learnt
that would have put the thing aside. A failed drive puts its place aside for
half an hour, and no failed drive was followed by the same goal.

## What it means

Of M3's twelve criteria, the ones about the record (6 and 7) and the stops
(9 and 10) are now met on the rover. So is duplicate dispatch (part of 11).
What remains is mostly the count: 15 more supervised sessions and about 87
minutes. Two other things remain. Voice and manual requests during a run need
showing against the declared priority, and a map change or lost pose needs
showing to revoke a moving drive (12).
