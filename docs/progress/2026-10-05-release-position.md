# Position does not tell a right release from a wrong one

**A release cannot be decided by where the observation points, either.** Against the
[frozen review](2026-10-05-release-audit.md) of the 200 release-only proposals, the
predeclared position gate would have kept 68 of the 83 measurable wrong releases but
also kept 52 of the 71 right ones, so it fails. It fails the same way on both older
folds judged by the older look labels. The reason is physical. The things wrongly
filed under a record sit right next to its object: a chair in front of the painting,
the rug on the floor, an armchair beside the office chair. Their bearings agree with
the record's position as well as the record's own looks do. Appearance and position
are the two kinds of evidence the repair has, and neither decides a release, so
release-only ends here as an automatic repair. R-WS-13 stays open; R-WS-17 and
R-WS-18 stay proposed. Nothing changed on the rover.

## What was tested

[release_position.py](../../experiments/entity_association/release_position.py)
refits each record's placement from its largest kept cluster, starting from the
stored placement. It scores the release's own ray against that placement with
the repair's existing bearing, range and height measures, and takes the largest.
The gate keeps a release that agrees with its record (score at most 2.5, the
repair's existing veto value) and releases the rest. The pass condition was
committed before any score was computed (3b46ce5). On the frozen review, the gate
had to keep at least half the wrong releases while still releasing at least 80% of
the right ones. On both older folds it had to do the same, judged by the older
labels.

| | Right releases still released | Wrong releases kept | Ranking (AUC) | Passes |
|---|---:|---:|---:|---|
| Frozen review, fresh proposal | 19 of 71 | 68 of 83 | 0.63 | no |
| Older fold 0, older labels | 7 of 17 | 8 of 11 | 0.56 | no |
| Older fold 1, older labels | 9 of 15 | 4 of 7 | 0.51 | no |

18 reviewed releases could not be scored, because the record keeps no cluster or the
observation has no bearing: 14 right and 4 wrong. The older folds are not an
independent recording. 165 of the fresh releases were also released there, and 22
and 18 of their labelled releases are observations the review also judged. What
they add is a second labeller.

## Why, and what was checked after the failure

After the predeclared test failed, these checks were run for explanation only;
none of them selects anything. No threshold on the same score would pass: the
closest, 0.57, releases 73% of right releases and keeps 46% of wrong ones. Bearing
alone ranks at 0.61 and height at 0.60. The median score of a release showing a
different object is 1.5, against 0.8 for a release of the record's own object. In
most cases the different object's bearing passes straight through the record's
position. Examples are the chair backs in the painting records, the rug edge in a
floor record and the wardrobe edge in an office-chair record, which all score 0.
Position stands out only where the release is far from its record: two releases that
point the wrong way entirely, and the two distant armchairs in the office-chair record
(4.3 and 4.4). Those releases are already right.

Range would be the measurement that separates a chair from the painting behind it,
but only 50 of the 200 releases carry one. On those, it does not separate right from
wrong either (0.61 combined, 0.40 for range alone, from 15 right and 23 wrong). That
is too few to rule range out, but it gives no reason to expect more ranging to rescue
the release step.

## What this leaves

Every repair tested so far fails on the same cases. That covers the blanket
appearance gate at attach time, joint repair, the masked semantic channel,
release-only and now position. The cases are degraded views of a thing on one side,
and a different object occupying the same direction on the other. Neither signal
in hand distinguishes them. One kind of evidence has not been tried: what the
selected pixels *are*. A chair back and a painting differ in kind even where they
share a bearing and look alike to the appearance score, while a distant or
washed-out painting is still a painting. The rover already computes a SigLIP image
vector for every observation and has SigLIP's text side for phrases. Scoring those
stored vectors against a few phrases ("a dining chair", "a framed painting", "a rug",
"bare floor", …) needs only the phrase embeddings from the rover. That can be tested
against this same frozen review, with its pass condition committed first, before
anything else is built.

The raw per-release measurements are in
`captures/2026-10-04-reader-validation/release-audit/position.json`; the
[summary](2026-10-05-release-position.json) records the predicate, every score and
hashes. Three new tests and the 26 existing experiment tests pass; only experiments
and documentation changed.
