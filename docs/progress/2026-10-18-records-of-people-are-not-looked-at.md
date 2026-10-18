# Records of people are no longer looked at: a model of the records' pictures picks them out, and passed on records it had not seen

**The world model's job now marks the records a frozen model takes for a
person, and an autonomous run does not aim a look at them.** On the half of
the labelled records it was not fitted on, it flagged 9, all of them people,
of the 13 people there; among 50 records drawn at random from those nobody had
labelled, it flagged none and none was a person. On the rover it marks 22 of
638 records, every one a person by the labels, and the four people among the
planner's candidate looks are gone from them. Nothing else reads the mark:
the filing, the voice and the console are as they were.
[R-WS-18](../requirements/world-state.md#r-ws-18) stays proposed.

## Why people

People were 8 of the 149 records an autonomous run could aim a look at, and 6
of the look trial's misses: someone sitting on the bed, someone crossing the
living room, gone by the time the look is taken
([2026-10-18](2026-10-18-what-the-look-trial-was-aimed-at.md)). A classifier
of every kind of non-object failed its blind tests, but was sure of people;
this one is for people alone.

## The model

A logistic regression on the mean of each record's regions' DINOv2 and
SigLIP2 vectors, each unit-length before and after the mean
(`world_state/visit_model/person.json`, digest `ebb5e66e3f00`). The 318
records labelled that day, 29 of them people, were split by a rule fixed
before anything was fitted -- development where the record id's sha256 starts
below 8 -- and its C and cut chosen on development alone: the lowest cut at
which non-people are at most 5% of what it flags in cross-validation. The rules
it was judged by were set before it was: at most 1 in 10 of what it flags not
a person, at least half the people caught.

| | Records | People | Flagged | Not people among them | People caught |
|---|---|---|---|---|---|
| development, cross-validated | 152 | 16 | 13 | 0 | 13 |
| **held out** | 166 | 13 | 9 | **0** | **9 (69%)** |
| unlabelled, 50 drawn at random of 320 | 50 | 0 | 0 | | |

It misses people sitting in an armchair, whose record is half the armchair,
and blurred figures in the dark. Matching the records' SigLIP2 vectors to the
phrase "a photo of a person" instead, with no training, caught 9 of 16 on
development, against the model's 13.

Fitted again from the labels by `experiments/m10_outcomes/people.py`, which
reproduces `person.json` exactly; the labels are in
`experiments/m10_outcomes/labels/` (by the coding agent).

## On the rover

`visit_model/people.py`, called by the job each time it writes the records,
keeps each record's sums of unit vectors and reads only the regions filed
since; a record that loses regions to a merge is summed again. The first pass
over 29,718 regions took 4.9 s, niced, and each pass after it 0.03 s.
`things.json` carries `person_p` and `person` per record and the model's
digest; the daemon passes `person` on with each record
(`world_state/visit_things.py`); `autonomy/goals.py` leaves a marked record
out of the looks it offers. A scorer that fails leaves the records unmarked
and is logged; the job still writes them.

Deployed at `0f5ee15` (world_state, rover_daemon, autonomy), each component's
tests passing on the rover. After the job restarted it marked 22 records with
`ebb5e66e3f00`, the daemon's reading of the records carried the same 22, and
the planner's candidate looks from where the rover is parked no longer held
v1037, v894, v606 or v1061. Navigation stayed settled through the daemon's
restart.

## What it leaves

Views through doorways, 10 of the 149, are still offered, as are records of
things that have gone.
