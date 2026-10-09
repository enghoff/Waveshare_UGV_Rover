# Out of reach, the chosen viewpoint improved its thing five times in thirteen, the re-look once

**Thirteen development attempts at things out of reach, each paired with its
re-look and scored against the tape: the chosen viewpoint's look improved its
thing 5 times, the re-look once, and neither ever made one worse.** The paired
difference in gain was +0.48, 95% interval +0.01 to +0.73, over five things.
That is the comparison
[the decision of the morning](../decisions/m4-asks-of-things-out-of-reach.md)
expected out of reach and could not get in reach, where the same score was
-0.07 ([the earlier attempts](2026-10-09-m4-development-attempts.md)). At these
rates `score_attempts.py` puts the acceptance set at 42 pairs. Development
evidence only. [R-WS-13](../requirements/world-state.md#r-ws-13) stays open.

## The runs

Trial runs on the six things taped on 2026-10-03, 57 records, opened by
`experiments/m4/start_trial.py` from the bedroom with no fence, so each started
with its targets out of reach. The rover drove to one, made its attempts there,
and came back. Since the morning's decision, a thing in reach is refused as
`in reach`, so every attempt here was at a thing out of reach of where the rover
stood when it chose. The plan and every reading are in
`captures/2026-10-09-m4-dev/PLAN-out-of-reach.txt`.

| Run | Opened | Attempts | Ended |
|---|---|---|---|
| `run/b21221da/2` | 12:54, 65% | 3 | 12:56, every target cooling or in reach |
| `run/b21221da/3` | 13:11 | 0 scorable | stopped by me at 20% at rest, 6.4 m into its first drive |
| `run/3cae0ea1/1` | 13:44, 65% | 7 | 13:48; its drive back blocked 0.4 m short of the start |
| `run/3cae0ea1/2` | 13:50, 50% | 2 | 13:52 |
| `run/3cae0ea1/3` | 13:52, 45% | 1 | 13:53 |
| `run/3cae0ea1/4` | 14:02, 20% | 0 | at once: every target set aside |

It ran unattended across the whole flat at the owner's word, against
[R-SAFE-6](../requirements/safety.md#r-safe-6)'s standing rule that a person
watches. The session ended at 15% while recovering, above the owner's 10% floor,
because the targets ran out: after an attempt that files nothing, its record
and the records grouped with it are set aside for two hours.

## The scores

`score_attempts.py` on the rover, against fresh copies of both stores and the
snapshots in place (`captures/2026-10-09-m4-dev/report-oor.json`, episodes 1387
onwards). Thirteen scorable over T2, T4, T7, T8 and T9; the one attempt the stop
cut short has no chosen look.

| | Improved | Unchanged | Worse | Mean gain, 95% interval |
|---|---|---|---|---|
| chosen viewpoint | 5 | 8 | 0 | 0.73 (0.01 to 1.29) |
| re-look | 1 | 12 | 0 | 0.25 (0 to 0.65) |

- **Every improvement was a look the depth camera ranged**: 5 of 5, and none
  of the 8 that went unranged.
- **The gains were large where they came.** The cabinet record (T8, 493) went
  from 0.51 m off to 0.02 m, and a dining-room painting record (T4, 468) went
  from a stated 2.54 m to 0.20 m, 0.31 m from its tape.
- **Honesty**: the tape inside the stated figure 7 of 13 after the chosen look
  and 8 of 13 after the re-look; overconfident 2 for both.
- **The rover's own account** (criterion 8): where the claim changed, it had the
  tape's sign every time, 5 chosen looks and 1 re-look.
- **The one re-look that helped** (T8, run 4) was taken from inside the living
  room, after an earlier drive in the same run had brought the rover there.

## What it means

Out of reach, going somewhere chosen is what improves a placement, and a look
from where the rover stands seldom does. That is the question M4 now asks, and
these attempts say it can be answered. Thirteen pairs over five things is too
few to call, and the acceptance set is sized at 42.

The acceptance attempts will run on different code. The owner asked another
session to make the gimbal aim instead of the rover turning (the gimbal lands
within 0.6 degrees over ±150, where a turn leaves the heading 6 to 16 degrees
out; [the arrival heading](2026-10-09-arrival-heading.md)). That change was held
until this set finished, so the set has one set of conditions. It may change both
rates, so the count is worth re-reading from the first acceptance runs.

## The pose after arriving

At the last arrival by the charger, navigation's pose was 0.6 m and 12 degrees
from where it had been sent, with 11% of the scan on a wall. Someone was standing
beside the rover. Between 14:07:22 and 14:07:45, with the wheels still since
14:06:31, it moved 0.25 m and 10.5 degrees onto a pose the scan agrees with to
4 cm and 0.0 degrees. Together with the 13:12 arrival, that is a pose changing
for up to a minute after arrival with the wheels still, once away from the truth
and once onto it. It points at the mapper rather than the gyro. Nothing was
recording the transforms, so it is not shown.
