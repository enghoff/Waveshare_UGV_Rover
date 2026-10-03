# Re-solving a whole session by EM finds the duplicates, and loses too much to use

**Expectation-maximisation is not worth adopting, either over a whole session or after
every look, so nothing was changed.** Run over every look of today's session at once, it did
what the deployed resolver cannot: it joined the door the rover had split into `object:96`
and `object:102`, and every other pair of look-alike things standing within half a metre.
It also did it in 3 to 5 seconds, so cost is no longer what stands in the way. But as
`cluster.py` specifies it, 117 things became 24. Even with its most damaging rule turned
off, 70 were left: 27 real things were dropped, two of its fifteen merges joined plainly
different objects, and 7% of crops ended up in the wrong thing, against none for the
deployed resolver. Run after every look instead, as it was benchmarked on 2026-09-03, each
pass took 25 to 30 seconds once 250 looks were waiting, and in 25 minutes it got through
less than half the session. [R-WS-13](../requirements/world-state.md#r-ws-13) stays `open`.

Scripts, logs and the copy of the store are in `captures/2026-10-03-whole-session-em/`;
the benchmark is [`world_state/bench_whole.py`](../../world_state/bench_whole.py), and the
per-look timing is `bench_cluster.py --only hard --budget 1500`. The recording is map
session 67 as it stood at 15:38: 339 looks and 2,572 regions, of which 2,407 have a
direction and 791 a range. The deployed build replays it to 117 things against the 126 the
rover holds. That is not an exact reproduction, because the session spans builds deployed
during the day, but it is the baseline everything below is compared with.

## Why it was tried again

EM lost on [2026-09-03](../../world_state/bench_cluster.py) for three reasons, and two had
changed. Two thirds of that recording's regions had no pose; today 94% have a direction.
No ray had a range; today a third do, and the benchmark had named ranges as what would
change its verdict. The third reason, cost, was the open question.

It was also the one approach in the component built to revisit a decision. The door was
split because the resolver commits a look the moment it is offered and never looks at two
placed things together again. `object:96` was founded on a picture with a chair in front of
the door, refused the door's clear views, and a duplicate check missed by 2 cm.

## What it did

The deployed resolver was replayed look by look, and its 117 things were handed to EM as
the starting point, with every ray of the session. The EM is `cluster.py`'s: each look's
best arrangement, a weighted least-squares fit with ranges, shares re-estimated, and fits
within 0.5 m merged.

| | Things | Looks attached | Crops of something else | Bearing miss, median / 90th / worst | Look-alike pairs within 0.5 m | Time |
|---|---:|---:|---:|---|---:|---:|
| Deployed resolver, look by look | 117 | 2,262 | 0 | 2.3 / 12.6 / 157 deg | 14 | 69 s |
| EM as `cluster.py` has it | 24 | 1,788 | 152 (9%) | 3.3 / 10.9 / 28 deg | 0 | 3 s |
| EM, merging only things that look alike | 31 | 2,088 | 102 (5%) | 3.5 / 10.2 / 26 deg | 0 | 3 s |
| EM, look-alike merging, shares held equal | 70 | 2,265 | 167 (7%) | 3.0 / 8.4 / 22 deg | 0 | 5 s |

"Crops of something else" and the bearing miss are `replay.score`'s and
`bench_cluster.misses`'s, unchanged. "Bearings that miss the entity's own position" is
left out: it divides by each thing's own stated uncertainty, and a fit over eighty looks
states 1 to 2 cm. The look-alike pairs are new. They count placed things standing within
0.5 m of each other that were never in one picture together and whose crops score 0.55
or more in the median. That is a list of candidates rather than a verdict, because identical
dining chairs stand closer than that.

One change to `cluster.py` was needed before any of this meant anything. It drops a whole
thing if any single look claiming it was taken from closer than 0.75 m, and a thing seen 81
times from 15 places nearly always has one such look. On the first run that rule alone
dropped several of the most-seen things in the room. The benchmark asks the question of
each look instead, so one close look costs that look. Without that change the three EM rows
end with 20, 27 and 56 things.

## Where the things went

**The shares do most of the damage.** EM weights each thing by how much of the room it
accounts for. Here walls, floor and the big furniture hold up to 121 looks each, and a
small object holds two or three. After one round of re-estimating, 22 of the 117 things
had lost every look to a bigger neighbour, and it cascaded from there. Holding the shares
equal is the standard way to stop that, and it is what keeps 70.

**What is left is looks trading between neighbours.** Even with equal shares, 27 things
were dropped, two of them seen over 20 times. Their looks went to a neighbour a median 0.6 m
away, in 15 cases more than a metre away, or to nothing. Positions were still moving 0.1 to
0.3 m after the 24 rounds `cluster.py` allows, as neighbours passed looks back and forth.

**The merges were mostly right.** All fifteen merges in the equal-shares run were looked at
crop by crop:

- **Twelve are one object:** the door, five paintings, both ceiling fans, the hanging lamp,
  the rug, the armchair, and the owner.
- **Two are different objects:** a ceiling corner joined to a doorway and two paintings, and
  an armchair joined to two paintings and a dark plant.
- **One cannot be told:** three of the identical dining chairs.

The door's eight looks ended in one thing in every EM run, where the deployed replay split
them three and five.

## The per-look version is still too slow

`bench_cluster.py`'s "hard" arm, which runs EM over the leftovers after every look, was
re-run on the same session with the bearings as stored. A pass took 0.2 s at look 30, 15 s
at look 120, and 25 to 30 s from look 130 on, with the slowest at 52 s. After 25 minutes it
had reached look 162 of 339; the deployed resolver replays all 339 in 69 s. The rover
resolves after every look, so this is not affordable. Ranges on a third of the rays did not
bound the work the way the 09-03 benchmark hoped. Where the time goes was not profiled.

## What is left

What EM got right does not need EM. Joining two placed things that stand together, look
alike and were never in one picture is what the store's `merge` already does when something
outside it says so. It refuses two things that shared a look, and it keeps both histories
without reassigning anybody else's looks, which is where EM lost its things. A merge-only pass
over the deployed resolver's output is the cheaper candidate. It is unmeasured, and the two
wrong merges above say that position and median appearance alone would not be a safe gate
for it.
