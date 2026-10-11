# M5: the executive goes to look at a change, and stops once it has

**The executive has a third kind of goal, a look at what the change watch says
has changed, and on scene 1's recorded decisions it behaves as M5's criteria 5
and 6 ask.** Given the furniture-sized changes the watch had logged, 11 of the 13
decisions that had one chose to go and look; once each change had been looked
at, all 13 chose exactly what they chose with no changes at all. It is not yet
allowed to move the rover: until the sessions' runs have recorded what it would
do, it is generated, scored and refused (`investigate_changes`, off). Its names
for changes are guesses while the world state holds one object as several
records.

## What was built

`investigate_change` ([autonomy/README.md](../../autonomy/README.md)): each
furniture-sized change in the watch's report, read through a new daemon call
`nav_changes`, is a look aimed at the change, level, taken from where the rover
stands when the place is in reach and otherwise from the nearest place within
the certified band. Every attempt is written down; a look taken since the watch
began seeing the change makes it worth nothing, an attempt without a look halves
it, two such attempts refuse it. Each is named after the record nearest it
within 0.6 m. The acceptance set gained 14 drawn cases
(`autonomy/scenarios/changes.json`): 63 of 63 as expected, among them a changed
place chosen over a thing equally far that has nothing left to gain from a look
(criterion 5, 0.49 against 0.01) and a change looked at since it was seen being
worth nothing (criterion 6). The executive's checks pass (871), the daemon's
(1181).

## Replayed on scene 1

Every decision the runs of 2026-10-11 14:12-14:28 recorded, 21 of them,
re-scored with the furniture-sized changes the watch had logged in the 15
minutes before each and the goal allowed to act:

| | Decisions |
|---|---|
| with a furniture-sized change logged in the 15 minutes before | 13 |
| ...that now choose to look at a change | 11 |
| ...that, once each change was looked at, choose as with no changes at all | 13 of 13 |

The changes are the log as it was, so 3 of the 11 chose the bedroom's false
wall report, which the map gate refuses since 71d8414; the other 8 chose an
armchair's new or old place. Each of those 8 had four records within 0.6 m, the
nearest 0.12 to 0.17 m off and the others within 0.35 m, none of them labelled:
the store held 663 records. The nearest names the change and the rest are kept
beside it, but which record is the armchair is not something this can say
(criterion 4, the world state's).

## For M5

Criteria 5 and 6 are shown on the acceptance set and on replayed snapshots, as
the plan asks before the hardware. What is left before the goal acts: its
shadow record from the sessions' runs, and the owner's word to turn it on.
