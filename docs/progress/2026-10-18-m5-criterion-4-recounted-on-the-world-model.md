# M5's criterion 4 recounted on the world model as the rover runs it: no wrong merge in seven sessions, and session 8 planned for what is left

**M5 failed on one count: the old store's records mixed the two bean bags
across sessions. The world model that replaced it, as it stands on the rover,
makes no such merge in any of the seven staged sessions.** It was developed on
six of them, though, so the clean test is a session it has not seen. Session 8
is planned as that test, and to show criteria 5 and 6 on the floor with
autonomy planning around the model's records. M5 stays not passed until then.
[R-WS-17](../requirements/world-state.md#r-ws-17) and
[R-WS-18](../requirements/world-state.md#r-ws-18) stay proposed.

## Criterion 4, from the rover's own filing

The records the rover had written at 10:31 on 2026-10-18 (628 of them, named
`ee1d222ead6b0ff6`), scored against the labelled pictures of every staged
session by `score.py` on branch `visit-model`. The armchairs are left out of
the trials, as agreed
([decision](../decisions/the-two-armchairs-cannot-be-told-apart.md)), and
wrong merges are counted across sessions.

| Sessions | Version 4's role | Moves kept | 95% interval | Split | Unresolved | Wrong merges |
|---|---|---|---|---|---|---|
| shakedown, 1, 2 | development | 3 of 3 | | 0 | 0 | 0 |
| 3, 4 | development | 3 of 3 | | 0 | 0 | 0 |
| 5 | development | 3 of 4 | | 1 | 0 | 0 |
| 6 | development | 3 of 6 | | 2 | 1 | 0 |
| **shakedown to 6** | development | **12 of 16** | 51-90% | 3 | 1 | **0** |
| **7** | its test | **5 of 7** | 36-92% | 2 | 0 | **0** |
| **all** | | **17 of 23** | 54-87% | 5 | 1 | **0** |

M5's own acceptance sessions, 1 and 2 of 2026-10-14, hold two of these
trials, both kept (the red bean bag and the cabinet). The splits are the step
ladder and the stool, twice each, and the red bean bag once in session 6.

## Criteria 5 and 6 against the model's records

Both criteria were shown on 2026-10-14 against the old store's things. Since
2026-10-17 autonomy plans around the model's records, whose worth is worked out
another way, so two scenarios now set a change against a record
(`autonomy/scenarios/changes.json`):

- **Criterion 5:** a changed place outranks a record seen on twelve visits,
  0.12 m wide, equally far.
- **Criterion 6:** a change looked at is worth nothing, and the run goes on to a
  record with something to gain.

All 65 scenarios come out as expected. That is offline; session 8 is to show
both on the floor.

## Session 8, planned before it runs

`experiments/m5_changes/staged/session8-planned.json` covers two bean bags only:

- **Set out** before the rover is unplugged.
- **Moved** while it waits in the bedroom. The red one goes to where the black
  one ended session 7 yesterday: a look-alike put down where the other stood
  the day before, the case that failed version 2 in session 5.
- **Then** loop B, a 3.5-minute autonomous run and home, as in session 7.

**Criterion 5** is shown if the run's first choices are looks at the
reported changes, ahead of any record. **Criterion 6** is shown if no change
is chosen again once looked at.

**Criterion 4** fails on any record taking pictures of a second thing. It is
judged from labels made before the live model's filing is read.

Prepared without moving: every leg plans (30.4 m in all), the pre-flight reads
READY with the pose trusted and the map settled, and the photo check run
during loop A works on session 7's data.
