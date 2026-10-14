# Identity across a move: acting on step 3's moved events keeps three of six moved things' records on replay, against one

2026-10-14. [R-WS-17](../requirements/world-state.md#r-ws-17), M5 criterion 4.
Development on sessions already seen; the next staged session is the test.

**What it means:** a record can follow its thing when the thing is moved, and
the camera-and-lidar change events of the what/where plan's step 3 say when.
Replayed over the shakedown and both M5 sessions, acting on them kept three of
six moved things' records where the rover kept one. Every move it made was
the right object, and it refused the one pairing that would have swapped the
two armchairs. It is not on the rover, and it has not been tested: all three
sessions were seen while it was built.

## Counting identity across sessions

[M5 session 2](2026-10-14-m5-session-2.md) moved the plain armchair and the
black cabinet. Labelled this time by the coding agent, its live store keeps
the armchair (weakly, through record 838), splits the cabinet to a new record,
and 838 holds both armchairs: session 2 fails criterion 4 as session 1 did.

The shakedown's two armchairs are the ones of sessions 1 and 2: the one by the
charger then carries the throw in its photographs. So a record "held the thing
before" if it held a labelled picture of the same object in any earlier
session, as the counting rules allow
([the decision](../decisions/m5-sessions-are-counted-before-they-start.md)).
That gives six trials: the shakedown's balcony armchair and red bean bag (moved
3.7 m), both armchairs of session 1, and the plain armchair and the cabinet of
session 2.

## The replay

From the rover's snapshot of 2026-10-11 23:04, the 529 looks since, through
the end of session 2, were fed to the live resolver. Step 3's events were
built once a minute as the shadow runner builds them, and each moved event was
acted on the minute it would have been known: the arrival's pictures filed
under the record that held the old place, and the record placed at the arrival.
It abstains when that record was not placed at the departure, does not look
like the arrival, or another look-alike record stands nearer the arrival.

| | Kept | Split | Records holding two things |
|---|---|---|---|
| live store | 1 | 5 | 3 |
| replay as recorded | 1 | 5 | 4 |
| with lidar ranges | 1 | 5 | 1 |
| with lidar ranges and moves | 3 | 3 | 1 |

Of nine moved events, three were acted on: the shakedown's charger armchair,
the red bean bag and session 1's plain armchair, each the right object by the
labels. Of the six abstentions one prevented a swap (the charger armchair's
departure paired with the other armchair's arrival), three would have been
right moves refused by a threshold, and two cannot be judged. The last record
holding both armchairs is the shakedown's 838, which neither change touches.
Scripts and decisions are in `experiments/identity_moves/` on the branch
`what-where`.

## The test

The method is frozen as committed. On the next staged session that moves
things the store already holds, a world snapshot is taken just before the
staging, and the session's looks are replayed from it with and without moves,
counted across sessions by labels made before either replay runs. It passes if
it keeps more trials than without moves and no record takes a second thing
that it did not take without them. Only then would the step move from the
experiment into the world state.
