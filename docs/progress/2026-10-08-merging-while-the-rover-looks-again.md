# Joining records every 300 looks: fewer split objects, more wrong looks

**Run every 300 looks on a replay of map session 67, the proposer's joins
followed by co-fit's link more of each labelled object's looks on both
independent label sets in all three runs, and split fewer objects -- but two
points more of the 10-03 set's looks end up under the wrong thing.** Joined once
at the end of the session the cost was under one point
([2026-10-08](2026-10-08-merging-by-cofit.md)); letting the resolver carry on
against joined records costs twice as much, as merging while the rover looked
did on [2026-10-04](2026-10-04-merging-while-the-rover-looks.md). Changing the
resolver's own records this way is not ready for the rover. Measured offline;
nothing deployed. R-WS-17 and R-WS-18 stay proposed.

## The runs

`replay_session.py`, all 2,094 looks of map session 67, with the
`merge_propose_every` and `merge_both_every` variants: every 300 looks the
journalled merge (`merging.apply`) joins the proposer's pairs round after round,
and in the second variant then co-fit's at 0.3 both ways. The resolver as
committed is run alongside, all three on today's code, each as recorded and
with 2% of unlabelled regions left out under seeds 1 and 2. (The bench's
earlier controls were run before its reach cache and are not comparable.)
Scored by `score_session.py`; the three numbers are the three runs.

| | as committed | proposer every 300 | proposer and co-fit every 300 |
|---|---|---|---|
| things at the end | 365, 382, 391 | 320, 319, 332 | 293, 283, 300 |
| foundings | 365, 382, 391 | 417, 426, 431 | 460, 452, 456 |
| 10-07 trial: same-object look pairs linked | 32, 18, 27 | 28, 50, 20 | **41, 31, 35** |
| 10-07 trial: different-object pairs linked | 0, 0, 0 | 3, 0, 0 | 0, 1, 0 |
| 10-05 depth drive: same-object pairs linked | 15, 13, 9 | 10, 14, 17 | **17, 21, 19** |
| 10-03: split objects of 16 | 9, 8, 8 | 7, 6, 6 | 8, 5, 6 |
| 10-03: looks under the wrong thing | 8.6, 7.7, 7.9% | 10.7, 8.9, 10.1% | **11.0, 9.6, 10.2%** |
| 10-03: different-object pairs linked | 17, 22, 0 | 3, 4, 4 | 0, 15, 10 |

## What it shows

- **The gain is real on the sets the weights never saw.** With co-fit after the
  proposer, the trial set's same-object links rise in all three runs (by 9, 13
  and 8) and the depth drive's (by 2, 8 and 10), against the proposer alone,
  which rises in one run and falls in two.
- **The cost is in later looks.** Joined records accept more looks of other
  things: the 10-03 set's wrong-look share rises by 2.4, 1.9 and 2.3 points,
  against 0.3 to 0.9 when the same joins are made once at the end. The resolver
  also founds a quarter more new things after joins, most beside a thing
  already placed.
- **The 10-03 set's cross links are too noisy to read**: 17, 22 and 0 with
  nothing changed.

## What it means

Joining is better done where the resolver does not read it: a table of which
records are one object, kept beside them, as the
[world-state plan](../plans/semantic-world-state.md#implementation-and-integration)
already proposes. The executive and the console would read the table; the
resolver would go on matching against records it built itself, so later looks
are filed as today. What the table holds is what the end-of-session joins
measured.
