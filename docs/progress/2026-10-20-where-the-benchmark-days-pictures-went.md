# Where the multi-day benchmark's pictures went: version 4 gave each moved thing a new record four times in five, for three different reasons

The multi-day benchmark's first round failed partly on answers given from
records that outlived their things
([the result](2026-10-20-the-multi-day-benchmark-ends-after-two-days.md)), and
answering better at question time does not mend them
([2026-10-20](2026-10-20-recency-does-not-let-a-record-give-way.md)). This asks
the world model's side: when a staged thing was set out at its place, which
record did its pictures go to? **Of the five times it can be judged, version 4
followed the thing once.** Each other time it started a new record. One cause is
mended in replay by a rule that changes nothing on any earlier session. The
other two are not.

## How it was measured

Both days were added to version 4's benchmark (branch `visit-model`,
`experiments/visit_model`). The store was copied from the rover after day 2,
every region coloured there, and the two days' drive recordings extracted, as
for the M5 sessions. Every picture to 2026-10-20 (53,161 regions) was filed with
frozen version 4 on this desktop. That filing gives the same record as the
rover's own for 98.6% of the benchmark days' regions, and for 98.7% overall.

The three things' regions were labelled by the coding agent from contact sheets
of candidates near each spot (`labels/md-day1-1019.json`, `md-day2-1020.json`).
The labels are not blind to the filing, which the earlier diagnosis had already
read. Day 1 has 18 regions of the red bean bag, 11 of the black one and 4 of the
stool. Day 2 has 15, 12 and none: no region the camera cut on day 2 shows the
stool at P3, so day 2 has no stool trial.

## What version 4 did

| Day | Thing | Records holding it before | Its pictures that day went to |
|---|---|---|---|
| 1 | red bean bag, P1 | three, all seen gone | a new record |
| 1 | black bean bag, P4 | five, three standing | a new record |
| 1 | stool, P2 | three, all seen gone | a new record |
| 2 | red bean bag, P1 again | day 1's, standing | day 1's record (kept) |
| 2 | black bean bag, P5 | day 1's, seen gone that visit, and the five | a new record |

Every thing already had three to five records before the benchmark began, and
that is what defeated the red bean bag. Its arrival matched its own three gone
records at 0.809, 0.772 and 0.770, against 0.66 at most for anything else. A move needs
a 0.04 lead over the runner-up, and the runners-up were the bag itself.

The black bean bag and the stool are a different matter: they look as much like
other things as like themselves. The black bag's arrival matched a gone record
of something else at 0.751 and its own at 0.740, 0.730 and 0.728. The stool's,
from two pictures, matched its own at 0.707 and something else's at 0.690.
Between armchairs, in the glare of the balcony window, no appearance rule this
model has will pick them out.

The records that gave the two wrong answers in every state outlived their
things for two more reasons:

- **The stool's record at P2 was never judged by the lidar.** The record is
  only judged when the lidar hit the thing while it was seen. That test looks at
  one scan at one picture's time, and on day 1 the beam passed between the
  stool's legs, or just beyond them. Over each whole visit the lidar hit the
  place 23 to 75 times and passed it 0 to 4 times. On day 2 it passed through the
  empty place 400 times, and the record stayed standing.
- **The black bean bag's record from 2026-10-17 is placed where the lidar
  always hits something.** At every sighting the lidar's return along the
  bearing came 0.6 to 0.9 m short of the record's place. The record sits beyond
  the bag, on something permanent, so no scan can ever see it empty. Version 4
  places a thing by the depth camera alone, and for this dark bag against the
  window the depth camera read long. On 2026-10-12 its three depth places lay
  about 0.5 m beyond where the lidar put the bag in the same visit's pictures.
  On 2026-10-17, with the bag standing at P2, one depth box at 4.9 m, where the
  lidar said 2.4 to 2.8 m, placed that sighting 2 m beyond the bag, and it was
  filed to this record.

## Two changes tried

Both are switches in the branch's `model.py`, off by default. With both off the
filing is version 4's, byte for byte.

- **A thing's own gone records are not its rivals** (`ONE_THING_RIVALS`). In a
  move, a gone record that matches the leading candidate by 0.70 or more, with
  colours matching and never in one picture with it, counts as the same thing
  recorded twice. The lead is taken over the best candidate that is not.
  Over every pair of records holding labelled pictures (`pair_likeness.py`),
  records of two different things matched at 0.63 at most, and 27 of 98 pairs
  of records of one thing matched at 0.70 or more.
- **Solid judged over the whole visit** (`SOLID_BY_VISIT`): hits against
  passes over every scan while the thing's pictures were taken, instead of one
  scan.

| | Moves kept on the earlier sessions (development, test, sessions 5-8) | Benchmark days | Records mixing two things | Gone calls on records of never-moved things |
|---|---|---|---|---|
| Version 4 | 22 of 33 | 1 of 5 | 0 | 3 |
| Own records not rivals | 22 of 33, every session as before | 2 of 5: the red bean bag on day 1 | 0 | 3 |
| Solid over the visit | 21 of 33: session 2's cabinet lost | 1 of 5 | 0 | 4: adds the rug's, on 2026-10-18 |

Version 4 was judged on five draws each without 1% of the pictures that hold
no labelled region, as well as on every picture (`draws_seq.sh`, the same
draws one run at a time). On those draws the doubles rule made no wrong merge,
and neither did version 4. It kept the red bean bag on day 1 in every draw. On
the earlier sessions it kept as many moves as version 4 in four draws and one
more in the fifth.

The visit-wide solid check does what it was built for. On day 2 it calls the
stool's day-1 record gone as the loop passes P2. But it also makes 41 more gone
calls over the whole history, among them one on the rug's record. A rug is flat, so
whatever the lidar hit there stood on it. Height does not separate the cases:
measured from the camera, the rug's regions sit at -0.08 m, the stool's at
+0.08 and a box's at -0.01. It is not adopted.

## What it means

The doubles rule is a candidate for version 5. It follows one more move and
changes nothing else, but on its own it would not have changed a single
benchmark answer, because the red bean bag was answered right either way. The
two answers wrong in every state need two other things:

- **a record placed where the lidar can judge it**: the lidar's own return, for
  a thing that reaches its plane, rather than a depth place 0.6 to 0.9 m beyond
  it;
- **absence judged over the visit for things the lidar does reach**, without
  taking the hits of what stands on a flat thing.

Criterion 3 asks autonomy to answer at least three more questions right than
the fixed loop. Those are the questions the loop got wrong, and these two were
among them, so the second round still waits on them.

No requirement changed state. R-WS-17 stays proposed.
