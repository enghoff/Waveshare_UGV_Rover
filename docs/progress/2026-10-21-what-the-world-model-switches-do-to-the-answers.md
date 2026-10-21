# Filing a moved thing to the record it already has does not, on its own, make the benchmark's answers right: a record that has gone still outranks it

The multi-day benchmark's second round waits on records that outlive their
things (R-WS-17,
[2026-10-20](2026-10-20-where-the-benchmark-days-pictures-went.md)). Two
candidate rules for the world model were scored on its trials then. This
entry scores them on what the benchmark judges: **the answers.** Neither
makes another one right. The rule that follows the red bean bag into its old
record makes one worse, and the one that judges the lidar over a whole visit
turns a wrong answer into an honest "it has gone".

## How it was measured

`experiments/visit_model/desk_answers.py` (branch `visit-model`) files every
picture to the end of each benchmark day and asks the nine questions here.
It asks them as `find_thing` does on the rover: each record ranked by its
average look against the phrase's vector from the rover's sidecar, with
near-ties within 0.01 ordered standing first, newest first. It answers after
both arms' pictures together, not each arm alone, so it compares filings; it
does not re-run the benchmark. It is right within 1.0 m; an answer from a
record seen gone or set aside is unresolved, as the benchmark counts it.

Under version 4 it reproduces both answers the benchmark had wrong in every
state:

- day 1's black bean bag, from `v1470`, placed 2026-10-17 at (-16.13, -10.95);
- day 2's stool, from day 1's record at P2.

All twelve control answers come out right.

| Filing | Right | Wrong | Unresolved | What moved |
|---|---|---|---|---|
| version 4 | 15 | 2 | 1 | day 1's red bean bag answers from a record of it seen gone |
| a thing's own gone records not rivals | 14 | 2 | 2 | day 2's red bean bag now also answers from a gone record |
| solidity judged over the visit | 15 | 1 | 2 | day 2's stool now answers from its day-1 record, seen gone |
| both | 14 | 1 | 3 | both of the above |

**Why following the move made an answer worse.** On day 2 the red bean bag's
pictures went to its old record, `v1728`, instead of day 1's new one. A
record is ranked by its average look, and `v1728` carries older pictures that
match "a red bean bag" less well. It now scores more than 0.01 under `v1471`,
an older record of the same bag seen gone, which then answers. The bag has one
record too many either way. What decides the answer is a gone record
outscoring a standing one of the same thing.

## Where the black bean bag's record went wrong

`depth_vs_lidar.py` sets the depth camera's range for each labelled region of
a thing standing on the floor against where the lidar put it at the same
moment. Over 9 such things and 262 regions, the depth range runs past the
lidar's return by 0.13 to 0.16 m at the median. That is about half a bean
bag's depth: the lidar hits the near face. It runs 0.6 m or more past in 3%
(the red bean bag) to 24% (the armchair with the throw). On the benchmark
days the lidar's place was no nearer the spot each thing was set out at than
the depth camera's. So placing things by the lidar in general would not help.

Of the 100 records lower than 0.8 m judged this way, 7 have depth places
mostly 0.5 m or more past the lidar's return. Paintings and the wardrobe
behind the bed show the same pattern, rightly, and are excluded by height.
`v1470` is one of the seven: 4 of its 4 depth places. On 2026-10-17 the bag,
standing at P2, was filed to it on one depth box at 4.9 m, where the lidar
said 2.4 to 2.8 m. That put the sighting 2 m beyond the bag, near the record's
old place.

## What it means

- **Following moves helps R-WS-17 and not the answers.** A thing with several
  records, some gone, keeps answering from whichever scores best.
- **Judging the lidar over the visit is the more honest of the two.** The
  rover would say the stool has gone from where it was, not send someone
  there. It is not adopted: it also calls the rug's record gone
  (2026-10-20's entry).
- **The black bean bag's wrong answer needs its record placed right.** A
  narrow rule (a low thing whose depth places mostly run 0.5 m past the lidar
  is placed by the lidar) would touch 7 records. Whether it mends the answer
  is not yet measured.

No requirement changed state. R-WS-17 stays proposed.
