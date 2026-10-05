# Matching-only measured visibility helps a painting; broader use remains unsupported

The saved run identifies different causes of fragmentation, not one general
appearance failure. A bounded visibility change recovers one painting connection.
Restricting it to existing-record matching preserves the reviewed floor-container
connections that broader use breaks. It still fails the frozen older proxy test:
five links lost and one new cross-target link. That new link involves a mixed
painting/toolbox/chair crop, so it is not proof of a clean-object mismatch. Keep
the narrowed candidate for a controlled recording, without deploying it or
relabeling its failed test as a pass. R-WS-13 remains open; R-WS-17 and R-WS-18
remain proposed; R-WS-16 remains settled. No rover calls or movement occurred.

## Corrections before interpreting the trace

My previous analyst label combined two landscape paintings. Frame
`20261005-174049-ced888` shows 68950 and 68951 on opposite sides of the snowy-forest
painting, proving they are distinct. The window-side views 68918/69018 must be
separate from dining-side views 68930/68951/68979/69001. The original frozen labels
and scores remain unchanged, with a correction note on the earlier entry. A
separate post-score corrected artifact contains 26 labels for nine subjects:
28 same-subject pairs and 297 different-subject pairs. The original range
abstention still retains 13 connections and gains none. Reported qualifying
baseline/parallax pairs decrease from 18 to 14. Neither label set is independent
acceptance evidence. The dining-side painting still has four owners across four
views; that failure does not depend on the mistaken six-view label.

The production ray builder catches failed map callbacks. It could therefore
swallow the offline map-invariance check's disagreement assertion. The replay
now retains that failure outside the caught callback and rejects the pass after
the resolver returns. An injected disagreement on the actual stationary recording
was rejected despite the production catch. Repeating the full moving control and
four-range diagnostic under this stronger guard yields identical final identity
checkpoints and owner changes. No map disagreement was exposed on that repeat;
the earlier result survives, with a stronger verified failure path.

## Exact trace and separate mechanisms

The instrumentation reproduces all 21 actual resolver passes, 81 checkpoints and
21,552 ordered map queries. It records 102 pre-look candidate snapshots for frozen
subjects without making extra reach queries or changing matching outputs.

The painting behind the front dining chairs has strong cross-view appearance.
View 68978 is refused by the mapped visibility bound: its stored range is 4.044 m
but the map's first obstacle is 2.550 m away. Its candidate painting record passes
bearing, height, range and appearance checks. View 68928 is also beyond mapped
reach, and at its first look additionally fails the existing 0.20 masking-drop
gate. Occupancy bounds are horizontal first-obstacle evidence; they do not by
themselves identify a wall rather than foreground furniture. Conversely, stored
depth is not independently proven correct merely because it reproduces.

All six pairwise plain scores among the four dining-side landscape views exceed
0.78. Their matches instead choose different old placements along similar
bearings. View 68930, without depth, joins object:400 at 0.80 m; another eligible
record scores higher in appearance. With depth, 68951 joins object:422 at 4.07 m
and rejects object:400 on range. Later unranged views choose object:341 and
object:370 through the whole-look geometry arrangement. Loosening appearance
would not explain or fix this example. The trace does not establish which old
records are pure, duplicated or contaminated; their owners are candidates, not
independent identity truth.

The cropped purple armchair has a different failure. At its first look, object:317
passes geometry but scores 0.495, below 0.55; object:359 scores 0.564 and is chosen.
At the later whole-chair look both pass geometry, but object:317 scores 0.707
against object:359 at 0.603 and is chosen. The two fresh crops score 0.603 plain
but 0.471 masked, and do not produce a valid geometric pair fix. A partial object
and a whole object need not resemble the same historical exemplars. This is not
evidence that simply lowering a global threshold is safe.

## Two predeclared development candidates

Commit `20310bc` froze measured visibility: extend mapped reach to a valid positive
stored depth, never beyond it, while retaining all other gates and measurements.
Every candidate map query must agree across the grids used in its original pass.
The fresh criterion was at least one recovered background-painting pair, all 13
existing useful links retained and zero reviewed cross-subject links.

It passes that fresh criterion: 68948/68978 connect through object:328, giving
14 useful links, no losses and no reviewed cross-subject links. Other assignments
change, including 68928 joining object:301 and the occupied armchair changing
owner. This is not complete painting unification or global correction proof.

All three older original-range controls reproduce the previously saved
development controls exactly. They use one resolver pass per inspection and
historical target mappings, not actual historical call recordings or independent
region-level truth. On October 1 the broad candidate retains 596/643 existing
proxy links (92.7%), loses 47 and gains 50; the two October 2 controls retain all
31 and 51 links. The 98% criterion fails. Lost pairs include 48360/48604: their
queried boxes and full photographs show the same pink floor container. This is
a genuine useful connection lost, not merely a numerical target-label objection.

Before computing a second arm, commit `a61f1c4` restricted the extension to matching
existing records. Discovery and placement refitting keep original map bounds;
no fitted distance/height cutoff or target-specific exception was added.
It again gains the fresh 68948/68978 pair and preserves all 13 old reviewed links.
On October 1 it retains 638/643 old proxy links (99.2%), loses five, gains 41 and
introduces one cross-target pair; both October 2 recordings retain all old links.
The floor-container losses disappear, but the zero-new-cross-target requirement
still fails. Both attempts are reported; the narrower arm does not erase the
broad failure.

The new cross-target pair is 48456/48769. The first box selects the background
painting; the second spans that painting, the red toolbox and chair/table structure.
The target mapping calls the second a toolbox. It is a mixed observation, with
no trustworthy single-subject identity label. Do not call this a clean painting
joined to a clean toolbox, and do not change the frozen score to declare a pass.
The five lost pairs likewise remain reported without claiming every historical
target assignment is pure-subject ground truth.

## Decision and required next evidence

Broad measured visibility is rejected for deployment by a reviewed real loss.
Matching-only visibility is a promising candidate, not an accepted fix: it
recovers a real connection and removes that regression, but its remaining mixed
inputs prevent a firm quality claim. The next decisive test needs a controlled
rover recording with two clearly separated painting views, an intermediate
occluded view, the foreground object retained separately, and raw depth plus
actual matching calls. The current recording has no unoccluded side-painting
region meeting that coverage. Appearance and historical record duplication need
separate treatment; none of these results authorizes an automatic merge.

The [measurement](2026-10-05-subject-fragmentation.json) preserves both candidate
results, original/corrected label hashes, replay hashes and limitations. Raw
artifacts remain under `captures/2026-10-05-depth-drive-2/` and the named `.cache/`
directories. Portable trace, score and visibility scripts are in
`experiments/entity_association/`; the experiments README gives commands. Only
offline experiments and documents changed. The rover resolver remains unchanged
and no service restart is required.
