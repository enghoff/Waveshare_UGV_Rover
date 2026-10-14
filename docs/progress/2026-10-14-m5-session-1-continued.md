# M5 session 1, continued: the run went to look at every change, identity fails again, and step 3 is revised for session 2

**Three things measured on session 1 after its first entry:**
- **The run's choices.** The autonomous run chose to look at all five changes
  the watch had reported, one after another, in its first 75 seconds, and
  never chose any of them again once looked at. That is what M5's criteria 5
  and 6 ask.
- **Identity.** Criterion 4 fails, as on the shakedown. One record took
  pictures of both armchairs as ordinary matches, and neither armchair had two
  pictures filed before the staging, so neither move was a trial.
- **Step 3.** It failed its test in [the session's first entry](2026-10-14-m5-session-1.md),
  and a version 2 is now fixed for session 2.

## The run's choices

The run (`run/21a58f06/1`) made 15 decisions in 210 s. Its first was a look
at the armchair's new place by the black cabinet, at 11:33:39. By 11:34:54 it
had chosen to look at the other four reported changes too: the old place by
the balcony, the bean bag, the armchair at the balcony door, and the charger
armchair's old place. Each look succeeded. It spent the rest of the run
improving the placement of things it remembered and chose none of the changes
again. The run could choose to look at changes because the switch has been on
since 2026-10-12.

The looks are also why step 3 named both armchair moves in the run and not in
loop B: on a loop the camera points where the rover drives.

## Identity (criterion 4)

Labelled by the coding agent from the session's crops (`identity_sheets.py`),
by the rules fixed before the sessions
([labels](../../experiments/m5_changes/labels-session1-1014.json)). The two
black armchairs can be told apart by the patterned throw on one. Backlit by the
window it reads purple, which at first passed for a different chair. The
identical dining chairs have no individual identity and were left out.

| Thing | Pictures filed before the staging | After |
|---|---|---|
| plain armchair, to the balcony door | 1 (and 1 unfiled) | 27 labelled, over records 320, 379, 838, 466, 929 |
| armchair with the throw, to the cabinet | none: behind the rover on the charger | 21 labelled, over records 379, 784, 488, 317 |
| red bean bag (added, not counted) | — | records 930, 914 |

**No trials, and one confident wrong merge**: record `object:379` holds five
pictures of each armchair as ordinary matches. **Criterion 4 fails**, as it did
on the shakedown with record 838. The remedy the identity audit proposed, a
record's place history kept apart from lasting identity
([R-WS-17](../requirements/world-state.md#r-ws-17),
[R-WS-18](../requirements/world-state.md#r-ws-18)), is not built.

## Step 3 version 2

Developed on session 1 and the earlier recordings, all now seen. Details are
in the branch `what-where`, `step3.md`.
- **Naming** is asked over the whole session, finding still in M5's first step.
- **"New"** means new to this place for a furniture-sized arrival. A small one
  must still be new to the whole house: on the controls, the weaker test made
  three small lidar blips into new things.
- **Moves are paired first,** except for a thing new to the whole house.
- **An arrival's pictures** are cut off only by a departure from the same place.
- **The scorer** replays everything after the staging in one piece and matches
  each staged change to one change only. Re-scored that way, session 1 gives 5
  of 7 found and no false alarm for both methods, as M5's live count did.

On the evidence it was fixed against:

| | Watch | Step 3 | Named right | False alarms, watch / step 3 |
|---|---|---|---|---|
| session 1 | 5 of 7 | 5 of 7 | 6 of 6 | 0 / 0 |
| shakedown | 5 of 6 | 5 of 6 | 6 of 7 | 1 / 2 |
| seven controls | | | | 1 event in all |

It passes on both, which says only that it was fixed against them. Session 2
is its test, by the rule unchanged since before session 1. No requirement
moved.
