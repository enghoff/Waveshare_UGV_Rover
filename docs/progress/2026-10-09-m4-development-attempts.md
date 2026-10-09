# M4's development attempts: a look again from where the rover stood did as well as driving to the chosen viewpoint

**Seventeen development attempts on the things taped on 2026-10-03, each paired
with its re-look and scored against the tape: the chosen viewpoint's look improved
its thing 3 times, the re-look 4 times, neither ever made one worse.** The paired
difference in gain was -0.07, 95% interval -0.27 to +0.09, over five things. In
the one room these runs drive, nearly every target is within the depth camera's
reach of wherever the rover already stands, and turning to it there gains what
driving to the chosen spot gains. That is the condition on which the
[M4 decision](../decisions/m4-measures-where-things-are.md) said its comparison
should be reopened; it is the owner's to decide. Two faults in the trial code had
to be fixed on the rover first, and the session ran unattended at the owner's
word. [R-WS-13](../requirements/world-state.md#r-ws-13) stays open.

## The runs

Fenced to the living and dining room, opened by `experiments/m4/start_trial.py`
with `captures/2026-10-09-m4-dev/truth-development.json`: the six things, 57
records labelled from their photographs (six mixed records left out). The plan
and every reading are in `captures/2026-10-09-m4-dev/PLAN.txt`.

| Run | Opened | Attempts | Ended |
|---|---|---|---|
| `run/140546c0/1` | 11:14, 85% | 0 | at once, nothing to do (the first fault) |
| `run/140546c0/2` | 11:32, 80% | 11 | 11:36, every target cooling or set aside |
| `run/b21221da/1` | 11:49, 55% | 6 | 11:52, the same |

No stuck drive, block, failure or alert in any run. The rover did not charge
between the second and third, 75% to 55% "on charge", and did after.

**Unattended.** The owner said "it's yours to control. I won't be around, so try
to run autonomously", and again before the second run. Against
[R-SAFE-6](../requirements/safety.md#r-safe-6)'s standing rule that a person
watches; nobody could press the console's stop, free a stuck rover or report a
contact below the lidar, so none is recorded either way.

## Two faults, fixed before any attempt

- **The trial's own targets were never offered** (def3bcb). Goals were generated
  for the twelve worst-placed things in the house and the trial then refused all
  twelve. The trial's targets now filter before that limit.
- **Every viewpoint of every target was refused for a wall in the way**
  (c7f0753). A thing on a wall or a cabinet is on cells the map calls occupied --
  the cabinet's records 0.3 m inside its mapped front, the paintings' up to 0.4 m
  behind the wall's face -- and the line of sight stopped at the first of them.
  The run of non-free cells straight back from the thing, up to 0.5 m, is now its
  own surface; a cupboard's wall still blocks.

Each reproduced in a test that fails on the previous code; autonomy 840, 49 of
49 scenarios; deployed and checked on the rover with a read-only deliberation:
no trial candidates before, then all refused for nowhere to stand, then 13 over
18 records. A thing hanging on the far face of a shared wall would now pass from
this side and be looked at in vain (raised by the other session); not guarded.

## The scores

`score_attempts.py` on the rover, against sqlite copies of both stores and the
snapshots in place (`captures/2026-10-09-m4-dev/report-17.json`). All 17
scorable, five things (T2, T3, T7, T8, T9; T4's fourteen records had nowhere to
stand clear of the dining chairs).

| | Improved | Unchanged | Worse | Mean gain, 95% interval |
|---|---|---|---|---|
| chosen viewpoint | 3 | 14 | 0 | 0.44 (0 to 0.89) |
| re-look | 4 | 13 | 0 | 0.51 (0 to 0.98) |

- **Where the re-looks were taken.** The four that improved their thing were
  1.7 to 3.3 m from it. Three of the improvements were on the same attempt for
  both looks.
- **Honesty**: the tape inside the stated figure 11 times in 17 for both (65%,
  against 68% for one sigma); overconfident 3 for both.
- **The rover's own account** (criterion 8): where the claim changed, it had the
  tape's sign every time, 3 chosen looks and 4 re-looks over two things.
- **Predicted gain**: 0.06 to 0.87 m on every attempt, and 14 of 17 gained
  nothing.

## What it means

M4's comparison was set against a re-look because re-looks were expected seldom
to improve anything. Here they improved as often as the chosen viewpoints, and
`score_attempts.py plan` finds no pair count at which the chosen viewpoint could
be shown better at these rates. What the viewpoint planner adds, where the thing
is already in reach, is a drive the battery pays for. The decision's own way on
is a comparison with the nearest reachable viewpoint, at well over a hundred
pairs; the other is to take this as the answer for things in reach -- turn and
look first, drive only when the thing is out of reach -- and keep M4 for things
that are not. The owner decides before any acceptance attempt.

The snapshots (17, 3.4 GB) are kept on the rover until that decision, in case
the attempts are wanted again.
