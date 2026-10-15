# Localising on stable cells: the matcher no longer walks off among moved armchairs without its strict setting, but the declared test does not pass

**With the lidar returns from cells that move taken out of its scans, the
mapper's original setting no longer walks 1.33 m off among the shakedown's
moved armchairs: it sits within 3 cm of the owner's refit, as the strict
setting deployed on 2026-10-12 does.** So the stable cells remove what caused
that walk. But the test declared for them on 2026-10-12
([first principles](../plans/perception-from-first-principles.md#what-could-be-tried-on-the-recordings-now),
test 1) does not pass, on two of its four conditions. Session 3's 0.5 m error
at 08:07 is not fixed by any setting tried.
[R-NAV-11](../requirements/navigation.md#r-nav-11) held: every run was a replay
of a recorded session, without the rover moving.

## What was built

`experiments/nav_replay` on branch `visit-model`:

- **`stable_cells.py`** lays each recording's scans on the map. A cell is
  stable when it is occupied in four in five of the recordings that saw it. For
  the shakedown it learned only from the eleven recordings of 2026-10-10 and
  11, so it could not know which armchairs would move.
- **`stable_filter.py`** sits between the recording and the replay mapper. It
  drops every return that lands neither near a stable cell nor somewhere never
  seen before. Where a return lands is worked out at the pose the replay
  mapper itself believes, as a live filter would have to. It dropped about a
  fifth of the returns.
- **`graph_replay.sh`** runs it with `STABLE=cells.npz`. Its two topic remaps
  are now passed after one flag: given twice, the second replaced the first,
  and the first such run had to be thrown away.

## The shakedown, whole session from its start graph

| | Current setting (distance 0.05, floor 0.1) | Stable cells, original setting (0.5, 0.5) | Test condition |
|---|---|---|---|
| Stuck stretch 08:02:30-08:11, from the refit | 2-3 cm, 0.2-1.3 deg | 2-3 cm, 0.2-1.3 deg | within 0.3 m and 3 deg: passes |
| Within 0.15 m of the best fit, elsewhere | 86% | 85% | at least the deployed setting's: fails |
| Drive home, 08:17:44-08:18 | ends 0.5 m out | ends 0.5 m out | no worse: passes |
| Corrections over 0.15 m that neither the live mapper nor the deployed setting made | | 8, 0.16-0.29 m | none: fails |

The original setting without the filter is what walked 1.33 m on 2026-10-12.
The replay of that day started from a graph saved at 08:00:11, which is gone,
so both runs here start from the session's start graph. The current setting
reproduces 10-12's result from it.

## Session 3

| From 07:57 to the board-reader stall at 08:10:45 | Within 0.15 m and 3 deg of the best fit | Worst |
|---|---|---|
| live | 92% | 0.60 m, 08:07:13 |
| replay, current setting | 89% | 0.60 m |
| stable cells, current setting | 85% | 0.60 m |
| stable cells, original setting | 94% | 0.60 m |

The stable cells were learned from every recording before session 3.

**At 08:07 the rover turns about 120 degrees at 48 deg/s while moving.**
Afterwards it stands at stop living-2, and the scans fit the starting map best
0.5 m away, with the heading right. Every run does the same. That stop stands
beside the cabinet's new place, and its fits are poor either way (0.45 against
0.56), where the session script already records that staged furniture beside
a stop misleads the position check. Whether 08:07 is a slip in the turn or the
measure itself is not settled.

## What it means

- **Stable cells are the right idea and not yet a better navigator.** They
  remove what made the matcher follow moved furniture, so its looser setting
  becomes safe. Combined, they did best over session 3. On the shakedown they
  tie the strict setting, except for eight small corrections the strict one
  did not make.
- **Nothing is deployed.** The strict setting stays.
