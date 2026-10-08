# What predicts an aimed look filing: big records seen in depth, and only by a factor of two

**A record at least 0.3 m across, or one whose looks were often ranged by the
depth camera, is about twice as likely to be filed by a look aimed at it -- but
even then only about a third of such looks file.** M4's criterion 5 asks for the
median attempt to gain; with at most one attempt in three filing, choosing
targets better cannot reach it on this store. Measured offline; nothing was
changed.

## How

`experiments/entity_association/target_features.py` lists, for each aimed look
with a filing outcome, facts about its target and whether the look filed. Two
sets:

- **Recorded**: the 99 aimed looks of 10-02 to 10-06 whose target was still in
  the store, put through `aimed.py` by `replay_aimed.py` (24 filed). Facts read
  from the store the replay started from, so a filing cannot feed back into them.
- **Today**: M4 sessions 1 and 2, 60 looks whose target is still placed (9
  filed). Facts read from the store after the sessions, so the filed looks are
  in them; this set leans towards the facts filing adds, such as ranges.

## What separates them

| | Recorded: filed | Recorded: not | Today: filed | Today: not |
|---|---|---|---|---|
| extent, median | 0.34 m | 0.24 m | 0.30 m | 0.23 m |
| share of looks ranged, median | 0.31 | 0.13 | 0.44 | 0.36 |
| observations, median | 47.5 | 29 | 15 | 25 |
| viewpoints, median | 7 | 7 | 5 | 7 |
| uncertainty, median | 0.64 m | 0.66 m | 0.73 m | 0.78 m |
| height above the floor, median | 0.88 m | 0.87 m | 0.53 m | 0.80 m |

Split at 0.3:

| | Recorded | Today |
|---|---|---|
| extent at least 0.3 m | 13 of 36 filed (36%) | 5 of 20 (25%) |
| extent under 0.3 m | 11 of 63 (17%) | 4 of 40 (10%) |
| at least 0.3 of looks ranged | 12 of 34 (35%) | 6 of 33 (18%) |
| under 0.3 ranged | 12 of 65 (18%) | 3 of 27 (11%) |

Exemplar consistency did not separate the session's targets
([2026-10-08](2026-10-08-why-aimed-looks-miss.md)), and nor do viewpoints,
uncertainty or height here.

## What it means for M4

Of today's 9 filings, 6 improved their target, so an attempt improves at
roughly two thirds of the rate it files. Criterion 5 asks for a positive
median realised gain over the whole attempt set, which needs more than half of
attempts to improve; the best subset found here files about a third of the
time. What stands between is the store: records that are not the object they
claim, or not where they say ([2026-10-08](2026-10-08-depth-placement.md),
[2026-10-08](2026-10-08-why-aimed-looks-miss.md)). Preferring large, often-ranged
records would roughly double the useful attempts, and leave the criterion out of
reach.
