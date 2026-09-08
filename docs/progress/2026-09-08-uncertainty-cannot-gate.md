# Placement uncertainty knows something about a merge, and nowhere near enough to refuse one

**The most promising lead the labelling turned up is finished as a gate, and it
took one afternoon rather than another drive to find out.** A thing holding two
objects is placed between them, so its rays agree less, and
[the labelled drive](2026-09-08-every-thing-labelled.md) noticed that the merges
sit at 0.60 m of placement uncertainty against 0.36 m for the real objects. That
is reproduced here. What it cannot do is what
[R-WS-13](../requirements/world-state.md#r-ws-13) asks: **the tightest threshold
that admits no merge at all sits at 0.135 m and leaves one of the fifty-two real
objects eligible.** There is no useful operating point anywhere on the curve.

## The measurement

All 76 verdicts, scored by `python world_state/bench_identity.py`:

| | median | mean | n |
|---|---:|---:|---:|
| real object | 0.359 m | 0.428 m | 52 |
| **holds two objects** | **0.596 m** | 0.640 m | 17 |
| bare floor, wall or glare | 0.368 m | 0.474 m | 7 |

A merge outranks a real object **0.680** of the time. Resampling both
populations puts the 95% interval at **0.517 to 0.827**, so seventeen merges is
barely enough to say the signal is there at all: 1.5% of resamples put it at
or below chance.

Every threshold, and what each would admit:

| admit below | merges in | real objects in | real objects lost |
|---:|---:|---:|---:|
| 0.102 m | 0 | 0 | 52 |
| 0.196 m | 1 | 7 | 45 |
| 0.254 m | 2 | 14 | 38 |
| 0.337 m | 4 | 23 | 29 |
| 0.395 m | 5 | 30 | 22 |
| 0.485 m | 7 | 36 | 16 |
| 0.675 m | 9 | 43 | 9 |
| 0.804 m | 13 | 47 | 5 |
| 1.620 m | 16 | 52 | 0 |

**The control settles it.** Counting a thing's looks and calling the ones with
fewest suspicious — a signal with no theory behind it whatever — separates the
populations at 0.556 and produces a nominally *better* zero-merge gate, at five
looks, keeping 10 of 52 real objects. When an arbitrary control beats the
candidate on the criterion that matters, the criterion is not being met by
either.

## Why it fails, and it is not a matter of a better threshold

The merges reach right down into the well-placed population, and the ones that
do have a shape:

| thing | placed to | what it is |
|---|---:|---|
| `object:124` | 0.135 m | a blue bin with glare and floor among it |
| `object:1` | 0.209 m | a blurred picture with a person among it |
| `object:44` | 0.325 m | 22 looks at chairs with a framed painting among them |
| `object:26` | 0.332 m | a shelf edge with a framed picture |
| `object:9` | 0.339 m | 21 looks at the sofa with a seated person among it |

**Geometry can only see a merge that is spread out.** A person sitting on the
sofa, a chair in front of the painting, glare on the floor beside the bin — the
second object is a few tens of centimetres from the first, so the rays that
belong to it cross where the rays that belong to the first one cross, and
nothing disagrees. That is the same fault the appearance remedies keep hitting
from the other side, and it says the two objects being close together is the
whole difficulty rather than an awkward corner of it.

## What it is good for, and the rover is already doing it

Ranking. The twelve worst-placed things — which is exactly the list
`autonomy/goals.py` builds when it decides what to go and look at again — are
**5 merges out of 12, 42%, against a base rate of 22%**; the worst five are 3 of
5. So a rover that re-looks at what its geometry doubts is about twice as likely
to be re-looking at a mistake as one picking at random, and the shadow decisions
of [the evening before](2026-09-08-shadow-decisions.md) were already ranking that
way without anybody choosing it. That is worth keeping and it is not worth
trusting: it says where to look next, not what is wrong.

## The stronger test, attempted and not trusted

If a scalar cannot see the second object, the obvious next question is whether
fitting *two* positions explains a thing's rays materially better than one. That
was tried on the same 76 things, seeding two positions at the two most distant
crossings and letting each ray join whichever fits it better. **The result is not
reported as a finding, because the fit underneath it is not the rover's.**
Rebuilding each thing's placement by hand from the recording's rays reproduces
the resolver's own stated uncertainty only to 0.317 m median, and a statistic
computed on a placement 0.3 m away from the one that was labelled is a statistic
about something else. Only 49 of the 76 could be fitted both ways at all.

The lesson is procedural: this question has to be put to the resolver, through a
replay, rather than to a reimplementation of it. Nothing in the attempt is kept.

## What was built

- **The 76 verdicts are checked in**, at
  [`world_state/labels/m0-2026-09-08.json`](../../world_state/labels/m0-2026-09-08.json).
  They were in a gitignored capture directory, and they cannot be recomputed:
  a person read the contact sheets. Each verdict carries the observation ids of
  the looks it was given for, which is what lets a later rebuild be scored
  against it — **entity identifiers are minted in discovery order, so every
  thing in the room is renamed by any change to the resolver**, and joining by
  name would silently score the wrong things.
- **`world_state/bench_identity.py`** scores any per-thing signal against them:
  the two medians, the ranking, its bootstrap interval, the whole threshold
  table, the tightest gate admitting no merge and what it costs. `--things`
  takes a later rebuild's things and joins by look overlap.
- Two numbers in the review's working file, `viewpoints` and `rays_agreeing`,
  could not be defined — `rays_agreeing` exceeds the look count on four things,
  so whatever it counts, it is not looks. They are kept in the labelled set and
  labelled unexplained rather than scored or thrown away.
- The suite covers the join, because attaching verdicts to the wrong things is a
  failure that prints a perfectly ordinary number. world_state 830 passed, 0
  failed, up from 798.

## Requirements

- [R-WS-13](../requirements/world-state.md#r-ws-13) stays `open`. One candidate
  remedy is now measured and refused on 76 labelled things rather than argued
  about, which is the first time that has been possible.
- M0 criteria 3 and 8 still fail, unchanged: 17 of 76.
- Nothing about the running resolver was changed. This is a measurement.

## Next

1. **Put the two-position question to the resolver itself**, through a replay
   that reports each thing's own fit, and score it here. It is the only
   geometric idea left that the scalar's failure does not also condemn, because
   it asks about the arrangement of the rays rather than about their spread.
2. **The held-out drive**, with the run manifest in
   [the acceptance runbook](../runbooks/m0-acceptance-drive.md) filled in first.
   Whatever remedy comes out of (1), it will have been chosen on this recording
   and owes a fresh one.
