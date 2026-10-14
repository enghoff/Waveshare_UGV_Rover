# A record that follows its thing when the thing is moved

M5's criterion 4 asks that a thing the world store already held keeps its
record when it is moved. The resolver cannot do that: a record stays where its
thing stood, the moved thing's new pictures meet nothing there, and they become
a new record or wait unfiled. Step 3 of the what/where plan already says when
something moved, by pairing the change watch's departure and arrival with
pictures that look alike. This acts on those events.

| File | What it is |
|---|---|
| `timeline.py` | step 3's events over a session, built once a minute as the shadow runner builds them, each dated by the minute it first appeared |
| `relocate.py` | one moved event acted on in the world store (below) |
| `moves_replay.py` | the rover's world from its 10-11 snapshot, the looks since fed through the live resolver, each moved event acted on when it would have been known |
| `count_all.py` | criterion 4 on the shakedown and sessions 1 and 2 by identity_count.py's rules; `--across` counts a record as holding a thing before if it held a labelled picture of the same object in an earlier session |
| `labels-shakedown.json` | Codex's shakedown labels, renamed to the staged things |

## What relocate does

1. The record is the one holding most of the remembered pictures at the old
   place, and it must have been placed within `FROM_M` (1.0 m) of the
   departure and look like the arrival (`PAIR_COS`, 0.7, on masked DINOv2
   exemplars).
2. It abstains when another record, filed before the move, looks as much like
   the arrival (within 0.05) and stands nearer the arrival by 0.3 m: of two
   look-alikes the nearer is the likelier to have come.
3. Otherwise the arrival's pictures are filed under the record, any record made
   only of looks since the departure is folded in, the record is placed at the
   arrival, and from then on its placement is refitted from its looks since
   the move only.

The rival rule was changed once, after running the step on the end-of-session
store (where every look-alike nearby blocked it); everything else was set
before any replay. **All three sessions were seen while building it**, so what
follows is development, not a test.

## Development result

Replayed from the 10-11 snapshot through session 2 (529 looks, 3,578
regions), with the lidar ranges deployed on 2026-10-14. Six trials: the
shakedown's balcony armchair and red bean bag, both armchairs of session 1, and
session 2's plain armchair and cabinet. Counted `--across`; the shakedown's
charger armchair is the one with the throw, by its photographs.

| | Kept | Split | Records holding two things |
|---|---|---|---|
| live store | 1 | 5 | 3 |
| replay as recorded | 1 | 5 | 4 |
| with lidar ranges | 1 | 5 | 1 |
| with lidar ranges and moves | 3 | 3 | 1 |

Nine moved events; three acted on, each the right object by the labels:

| Event | Decision |
|---|---|
| shakedown: charger armchair's old place to the balcony armchair's new one | abstained: the balcony armchair's record looked as alike and stood nearer (the swap a blind relocation would make) |
| shakedown: a small move of the charger armchair | abstained: the old place's pictures split evenly over two records |
| shakedown: a place 4 m from the black bean bag's arrival | abstained: the record does not look like it |
| shakedown: the charger armchair | moved record 317 |
| shakedown: the red bean bag, 3.7 m | moved record 917; the bean bag's trial now kept |
| session 1: the plain armchair | moved record 466, which held it on the shakedown; now kept |
| session 1: the armchair with the throw | abstained: its record looked 0.68 alike |
| session 2: the plain armchair | abstained: its record placed 1.04 m from the departure |
| session 2: the cabinet | abstained: another record looked as alike and stood nearer |

Three of the six abstentions would have been right moves, one prevented a
wrong one, two cannot be judged. The record that still holds both armchairs
is the shakedown's `object:838`, which neither change touches. No region
moved with a record shows a second thing.

## The deployed version

Deployed on 2026-10-14 as `world_state/relocate.py` and the daemon's
`world_state_relocate`, journalled and undoable, called by the shadow runner
with `--act`. Its decisions are the experiment's, but a moved record takes a
fresh placement stated at 0.3 m rather than keeping the old fix's figures.
Replayed the same way, that record then matched more of its thing's pictures,
and session 1's armchair with the throw was moved too, to its own record of the
shakedown (317): four of nine events acted on, each the right object. The
counts are the same: three kept, three split, the one two-thing record the
shakedown's 838.

## The test

The deployed `world_state/relocate.py`, unchanged, on the next staged session that moves
things the store already holds: a world snapshot taken just before the
staging, the session's looks replayed from it with and without moves, counted
`--across` by labels made before either replay is run.
It passes if it keeps more trials than without moves and no record takes a
second thing that it did not take without them.
