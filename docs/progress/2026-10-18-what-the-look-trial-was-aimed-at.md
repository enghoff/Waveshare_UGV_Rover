# What M10's look trial was aimed at: a third of the misses went to records of no object, and few to a second record of the same thing

**Of the 82 looks of M10's trial that did not land, 26 were aimed at records
that stand for no object** -- bare floor, a stretch of wall, the view through a
doorway, a person -- **23 at records whose thing was no longer there, 14 at the
six identical dining chairs, 9 at things the region model did not box, and 7 at
a second record of a thing standing there.** Splitting one object into two
records, which the plan named as the cause, is the smallest share. Of the 149
records the planner could aim a look at that evening, 33 stand for no object,
and 6 of its 18 candidate looks were at them.
[R-WS-17](../requirements/world-state.md#r-ws-17) and
[R-WS-18](../requirements/world-state.md#r-ws-18), whose wrong looks include
bare surface and doorways, stay proposed.

A classifier of the records' pictures that would have told the planner which
records are no object failed both of its blind tests, and is not used. What is
used is narrower: a record placed below the floor is no longer looked at,
because a look aimed at its height points at the floor. Of the 149, that is 11
patches of bare floor and 4 of a rug's records.

## The misses

The trial (`m10trial1-1018`, [the entry](2026-10-18-m10-look-trial.md)) had 92
looks with a picture, 10 landed. For each of the rest, `pointing.py` found the
regions of its picture that point at the target -- a sight line passing the
target's place, and a measured place, where there is one, near it -- and where
the world model filed them; `miss_sheets.py` drew each picture with the aim
point beside the target's and the taker's earlier crops, and each was judged by
eye.

| The target record | Looks |
|---|---|
| no object: bare floor 9, a view through a doorway or down a corridor 8, a person 6, a stretch of wall 3 (one record, v41, 45 visits) | 26 |
| a thing no longer there: an armchair moved (5), the pink bean bag, a mop, a dark furry thing on the bed, floor clutter, and old second records of things held elsewhere, among them a cabinet, a painting, a rug and the bed with the wardrobe | 23 |
| the dining chairs, which nothing here tells apart | 14 |
| not boxed: a light switch (4), the bed close up (2), two pictures with no region at all, a painting just above the aim | 9 |
| a second record of a thing standing there: the two black armchairs (3), the two small diamond paintings (2), the snowy-tree painting, a desk | 7 |
| merged into another record during the trial | 3 |

Records of no object hold many visits: the floor patch v152 55, v229 41, v1119
31; the wall v41 45; a boy sitting on the bed, v744, 18. They are not stale --
the region model boxes the same floor and wall again on every pass -- so
nothing in the model sets them aside.

## The records the planner can aim at

All 149 records seen on six visits or more with a height, labelled from three
crops each: 104 objects, 12 fixtures (doors, windows, switches, a pipe), 12
patches of bare floor, 10 views through a doorway, 8 people, 2 of wall or
ceiling and 1 of several things. The labels are in
[labels/records-1018.json](../../experiments/m10_outcomes/labels/records-1018.json),
by the coding agent.

## A classifier, tried and not adopted

A logistic regression on the mean of each record's DINOv2 and SigLIP2 vectors,
frozen before each test, judged on records it had not seen, with the rules set
before the test was labelled: at most 1 in 10 of what it flags an object, and
at least half of the non-objects caught (`objectness.py`).

| | Trained on | Cut | Unseen records flagged | Objects among them | Non-objects caught, estimated |
|---|---|---|---|---|---|
| round 1 | the 149 | 0.64, none of the 149 flagged held out one at a time | 54 | **10 (19%)** | **40%** |
| round 2 | those and round 1's 114 | 0.77, 5% objects in cross-validation | 5 | **1 (20%)** | **8%** |

It is sure of people and of corridors, and takes the side of a wardrobe, a
door, a fridge or a row of kitchen cabinets -- large flat panels -- for a wall
or a doorway. Round 2's test is weak, because round 1 had already labelled most
of what it would flag, but it fails on both counts.

## Below the floor

`autonomy/goals.py` (`RECORD_BELOW_FLOOR_M`, deployed at the commit that
carries this entry): a record placed more than 0.05 m below the floor is not
looked at. Of 638 records, 23 are: 12 patches of floor, 5 of a rug's records,
a door stopper, a dark box and 4 not labelled. Of the 149 the planner could aim at, 15: 11
floor, 4 rug. One of the trial's 8 looks at such records landed, at a rug. Two
patches of floor placed at floor level, v950 at 0.01 m and v809 at 0.10 m, are
not caught.

## What it leaves

Views through doorways and people, 18 of the 149, are still offered, and the
records that outlive their things are still there. Taking absence from the
camera to set the last aside was measured the same day and kept away from
main records only by giving up most of them
([entry](2026-10-18-absence-from-the-camera-is-not-honest-as-gone.md)).
