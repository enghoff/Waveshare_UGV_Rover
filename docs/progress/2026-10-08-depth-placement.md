# Placing things by depth: three times closer to the tape, but only where a record is pure

**A placement taken from the depth readings that agree with it lands a median
0.11 m from the owner's tape, against 0.36 m from bearings, and its stated
uncertainty covers the tape about as often as a one-sigma figure should.** That
holds for looks matched to a target by eye. Across a whole replayed session it
holds only when matching is tight enough to keep records pure, and tight matching
splits more objects. With matching left as it is, mixed views' depth readings
pull the positions off again. Nothing was deployed. R-WS-17 and R-WS-18 stay
proposed; R-WS-10 is retired in favour of
[claims measured honestly](../decisions/bearings-are-measured-not-required.md),
which is what the coverage figures below measure.

## Against the tape

The six targets taped on 2026-10-03 (`captures/2026-10-03-targets/`) are seen in
126 looks matched by eye, all present in the session 67 store. Fed in time
order, the placement was worked out after each look three ways
(`experiments/entity_association/placement_calibration.py`):

| | median off the tape | median stated | tape inside the stated figure |
|---|---|---|---|
| bearings, as the resolver does (`best_fix`, `refine`) | 0.36 m | 0.38 m | 79% |
| every bearing pooled in one least-squares fit | 0.30 m | 0.29 m | 51% |
| depth where it agrees, bearings otherwise | **0.11 m** | **0.15 m** | **68%** |

- **Bearings** never state less than their best crossing, and do not get closer
  with more views: T3 is still 0.43 m off after 29 looks, T7 0.43 after 27. The
  floor is set by systematic bearing error, not by too few views.
- **Pooling every bearing** is badly overconfident. Close-range looks at part of
  a painting weigh most and miss its centre: T4 ends 0.97 m off claiming 0.17.
- **Depth** finishes within 0.03 to 0.15 m on five targets. The sixth is the
  green landscape painting, behind the dining chairs, with two ranged looks: 0.33
  m off against a claimed 0.16.

The depth rule was fixed before scoring, on the taped drive's published finding
that ranges landed within 0.03 to 0.27 m of the paintings. It is the median of
the points the agreeing ranged looks give. Its uncertainty is the larger of
their spread and their own error over the root of the number of viewpoints.

## Across the whole session

Replayed over all of map session 67 (`replay_session.py`), scored against the
tape for the record holding most of each target's looks (`score_tape.py`) and
against the labels (`score_session.py`):

| | taped targets: median off | honest | split of 16 (10-03) | records per object |
|---|---|---|---|---|
| baseline, three runs | 0.30, 0.40, 0.48 m | 3, 1, 0 of 6 | 6, 6, 6 | 2.1 to 2.3 |
| depth placement, matching with it (`ranged_refine`) | **0.11 m** | **4 of 6** | 8 | 3.1 |
| depth position and claim, matching as before (`ranged_claim`) | 0.37 m | 1 of 6 | 9 | 3.0 |

With depth's smaller figure, matching narrows and records stay pure. The
positions come out right, but more objects split. With matching's own
tolerance kept, more mixed looks join, and their ranges drag the positions back
off.

Duplicates cannot be found by position instead. Records of the same labelled
object sit a median 1.0 m apart in both the baseline and the depth replay, and
only about one pair in six lies within 0.25 m. A duplicate is the same object
placed somewhere else, mostly from bearings alone.

## What it means for M4

Two findings point the same way:

- **Goals measure gain on the wrong figure.** Geometry goals predict and measure
  their gain on `uncertainty_m`, the tolerance matching uses, which is never
  less than the best crossing's. What the rover claims is `stated_uncertainty_m`.
  A goal cannot show a gain on a number built not to shrink.
- **An aimed look can deliver the gain on its own.** Of the 16 aimed-look picks
  that showed their target ([2026-10-08](2026-10-08-aimed-looks.md)), 12 carried
  a depth reading. Filed by aim, each would give its target a ranged point,
  which on the taped paintings is typically within 0.2 m.

So the path for M4 does not wait for a clean store. An aimed look names its
target. Its region at the aim is filed to that target. The target's claim
is updated from that look's depth reading. The goal measures the claim. Each
link has been measured offline here; the whole needs a case under R-WS-13 and a
supervised run.
