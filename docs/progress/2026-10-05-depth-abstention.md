# Minority-surface abstention passes a range pilot; entity behavior is unproven

The fixed depth-abstention candidate passes its predeclared development criteria:
it keeps 40 of 41 correct taped readings and removes two of four errors of at least
a metre. It also rejects the new painting's reproduced ambiguous distance. Keep
this candidate for further testing; do not reject it because other identity rules
failed. It is not an accepted automatic gate: transparent furniture can present a
legitimate minority near surface, and the effect on identity and placement has not
been replayed. R-WS-13 stays open, R-WS-17 and R-WS-18 stay proposed. Their provisional
quality target has not been relaxed. Nothing changed or moved on the rover.

## Rule and decision fixed before computing

Commit 125c0d8 fixed the candidate: abstain only if the production-selected nearest
depth band holds less than half of valid projected samples and the overall median
is more than 0.5 m behind its median. Do not choose the farther surface, trim the
mask, change box fallbacks, remove an observation or alter its identity. Pass only
if it rejects the known painting view 68640, retains at least 90% of readings
within 0.25 m of the tape, and removes at least half of errors of a metre or more.
These are point-count development criteria, not calibrated probabilities.

Commit 3547957 then fixed a supplementary check on the separately recorded
2026-10-03 taped-target drive, before computing it, with the same unchanged rule.
Stationary controls had to reproduce within 2 mm and retain at least 90% of correct
readings. Fewer than two gross errors was declared insufficient to evaluate removal.
No threshold search, retuning or manual exception was added.

## Older controls and candidate

The original three taped drives contain 124 target observations with photographs,
poses and depth in the existing untrimmed-outline replay. Their outlines predate
storage, so they were regenerated with the exact cached YOLOE ONNX export on CPU;
every region's box overlap exceeds 0.5. The current production sampler provides the
control, with half-resolution stored-outline encoding and no erosion. The traced
two iterations must reproduce every outline control exactly, including its initial
box-derived range guess. Sixty-six of 73 available control readings agree with the
historical untrimmed replay within 2 cm; all differences are retained. The current
and historical control aggregates agree: 73 readings, 41 within 0.25 m and four
gross errors. The older replay convention projects at zero shutter turn; its old
moving observations are not proof of current motion compensation.

| Recording | Control ranged | Correct within 0.25 m | Gross errors | Abstained | Correct lost | Gross errors removed |
|---|---:|---:|---:|---:|---:|---:|
| 2026-10-01 | 57 | 26 | 4 | 7 | 1 | 2 |
| 2026-10-02 first | 6 | 6 | 0 | 0 | 0 | 0 |
| 2026-10-02 redo | 10 | 9 | 0 | 0 | 0 | 0 |
| Total | 73 | 41 | 4 | 7 | 1 | 2 |

Correct retention is 97.6%, gross-error removal 50%. All seven abstentions come
from the oldest recording, which includes moving and blurred looks and predates
later range safeguards. Four gross errors are too few to establish a general
removal rate, and repeated object views are not independent trials. The two gross
errors removed are cabinet views; both remaining gross errors are small floor
objects. The rule detects surface ambiguity, not every range error.

All seven abstentions and both retained gross errors were reviewed in their full
photographs after scoring. The one correct reading discarded (48456) is a genuine
view of the painting behind chairs: 3.988 m against 4.063 m from the tape model.
One refused region includes cabinet plus painting, and another includes a toolbox
inside a broad region of table/chairs/painting. Keep the original taped-target
mapping and its primary score; those mixed regions limit an object-specific
interpretation rather than authorising silent relabelling.

## Supplementary stored-outline check

The 2026-10-03 recording supplies 70 target observations with depth, of which 14
fall inside the original stationary windows. Nine stationary readings reproduce
within 2 mm of their stored distances, and all seven correct stationary readings
are retained. There are zero gross errors and zero abstentions. The 56 moving
observations are reported separately: 30 stored readings, 22 correct, no gross
errors or abstentions; zero-turn reconstruction does not establish their temporal
fidelity. Stored missing ranges remain missing.

This supports retention on another recording. It cannot confirm error removal:
the test contains no stationary view of the painting behind the chairs and no
stationary gross error. It is same-room development evidence, not independent
identity acceptance.

## Transparent-table limitation

The previously reported glass-table example 59864 was selected from the earlier
outline study and reviewed from its photograph before its new candidate result.
Its box includes a real glass-table edge and metal leg, with background seen through
it. Regeneration matches the stored box at overlap 1.0. Production reports 1.509 m
from a near band comprising 32.4% of 170 samples, while the median is 4.137 m. The
fixed candidate abstains. There is no taped distance for this view: neither the
near band nor the far majority is labelled numerically correct in this study.
The example establishes why minority surface does not mean wrong physical object;
it does not by itself fail the predeclared numerical test.

## Decision and next proof

The narrow claim is settled for these recordings: this fixed rule can remove some
bad range readings while losing one of 41 correct readings in this pilot. This
cannot be compared directly with the earlier identity-pair retention measure.
Preserve it as a candidate and surface ambiguity as evidence.
Do not turn it into a mandatory identity gate or select the majority surface as
truth. Before production use, show what withholding just these ranges does to
correct identities, founders, placements and fragments on a reproduced complete
resolver sequence. Transparent and narrow objects need measured controls, and the
remaining controlled recording needs exact resolver-call provenance.

The rover stayed parked for charging throughout this work; no service was changed,
restarted or called. The companion [measurement](2026-10-05-depth-abstention.json)
contains all 124 primary rows, 70 supplementary rows, hashes, frozen/post-score
review limits and the glass example. The experiment README gives reproduction
commands. Requirements are unchanged; a range-pilot pass is not an identity pass.
