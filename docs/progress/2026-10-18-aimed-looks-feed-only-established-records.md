# Under the world model, an aimed look feeds a record seen on six visits or more about half the time, and one seen on fewer almost never

**Autonomy now aims only at the world model's records a look can feed.** The
morning's first run on the records aimed 14 looks and none was filed to its
record ([entry](2026-10-18-the-first-run-on-the-world-models-records.md)). The
store's own history says why. Every look it recorded as aimed at a thing in the
M3 and M4 sessions was judged again under version 4: what decides whether a
look feeds its record is how established the record is.
[R-WS-17](../requirements/world-state.md#r-ws-17) stays proposed.

## What was measured, on the rover's history

The store's `aimed_looks` table holds 107 looks. For 106, the record intended
is the one that held most of the aimed-at thing's earlier looks as version 4
filed them.

| The record before the look | Looks | Aimed region filed to it | Any region of the picture |
|---|---|---|---|
| seen on six visits or more | 100 | 52 | 59 |
| seen on five or fewer | 6 | 0 | 1 |
| all | 106 | 52 | 60 |

The morning's run adds 14 more. Twelve were at records seen on five visits
or fewer, and two at records seen on 15 and 22 visits but placed from sight
lines alone, with no depth place. None was filed to its record.

The records the model holds, at 07:50, 502 of them in place:

| Visits | Records | Spread over 0.12 m |
|---|---|---|
| 1 | 211 | 159 |
| 2 | 73 | 48 |
| 3-5 | 67 | 36 |
| 6 or more | 151 | 29 |

So ranking by spread, worst first, ranks by how little a record has been seen.
That is the opposite of what a look can feed.

## What changed

- **A record is a target only when seen on six visits or more, with a depth
  place.** The others are not offered.
- **A look is predicted to gain a record half of what filing it would**, since
  about half land.
- Deployed (`autonomy` at `5703bb0`). `decide.py` on the rover took 2.5 s and
  would look from where the rover stands at `v1494`, seen 18 times, 0.31 m
  to an expected 0.25 m. Replayed on the 2026-10-17 reading: 0.6 s, 12
  geometry goals, 9 usable, at records seen 17 to 152 times.

## What it leaves

The 284 records seen on one or two visits are the model's to settle, and most
are probably strays or second records of something held elsewhere: a look at
one found something 0.12 m away that the model filed to another record. A look
that sees such a place close up and finds nothing there marks nothing gone,
because version 4 takes absence from the lidar alone.
