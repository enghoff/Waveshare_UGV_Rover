# The painting/table fault reproduces; a blanket appearance gate costs too much

The offline replay now reproduces the rover's painting-to-table attachment and
its decision explanation. All 4,026 final assignments, every placement and both
appearance exemplar sets match the real end snapshot. Three candidate safeguards
were tested without changing the rover. The strongest candidate helps the short
fresh recording but loses too much on the longer recording when tested objects
are excluded from fitting. It is retained as diagnostic evidence, not deployed
as a blanket attachment rule. R-WS-13 stays open; R-WS-17 and R-WS-18 stay proposed.

## The reproduction is a result in its own right

The replay starts from the before-snapshot, retaining existing memberships,
placements, exemplars, counters and pending observations. It adds the 232 later
observations with their original IDs, vectors, boxes, poses and ranges.

Resolving every saved frame immediately failed to reproduce the target. The
inference records show that many automatic photographs deliberately did not
settle identity. Respecting that schedule reproduced the target but left ten
fresh assignment differences. Surviving placement-update timestamps support
additional background passes after frames 7, 13, 16, 22, 25 and 27. With those
passes and the earlier saved occupancy map, all assignments, placements,
observation counts and exemplars match. Only two diagnostic notes differ
(64657 and 64661). Input snapshot hashes remain unchanged.

This reconstructs the decisions and final state, not every background timestamp
or live occupancy map. Those were not recorded. The inferred schedule and frozen
map are explicit in the artifact, and the target is decided after frame 16.
No rover connection, perception rerun or movement was required.

## Why the painting was admitted

Observation 64638 is offered to table record `object:415`. Its plain resemblance
is 0.557, above the existing 0.55 floor. Masked resemblance is only 0.429, but the
drop of 0.128 stays below the existing 0.20 refusal threshold. The bearing is
compatible and uses 4.4% of its allowance. Removing this assignment makes the
whole-frame geometry arrangement cost 0.063 more, above the 0.05 tie threshold,
so appearance does not get to choose among rivals. This reproduces the immediate
mechanism; it is not proof that the table's earlier history was pure.

The table's five current exemplars are observations 64556, 64581, 64592, 64603 and
64627: a chair view, two table views, a table part and an overlapping door view
under the analyst drafts. A single physical identity cannot be assumed from the
record name. The existing merge appearance score over those exemplar pairs is
-0.217 for the painting, below its existing zero threshold. That suggested a
testable safeguard without fitting a new threshold to this incident.

## Short recording: a useful gain with a small cost

The same reconstructed schedule, initial state and map were used in all runs.
Labels remain the frozen analyst drafts. The primary score covers 146 clear
object regions; tentative chair/door/part identities are a separate sensitivity.

| Attachment rule | Correct pairs together | Wrong pairs together | Waiting regions |
|---|---:|---:|---:|
| Unchanged resolver | 284 | 3 | 39 |
| Require positive existing appearance evidence | 283 | 0 | 42 |
| Require masked resemblance at least 0.55 | 260 | 0 | 45 |
| Only update exemplars above 0.70 resemblance | 275 | 10 | 37 |

The evidence gate gains two correct pairs, loses three and adds no wrong pair
in either primary or tentative scoring. The painting now joins a record with
another fresh painting observation; the old history of that destination is not
independently labelled, so its overall purity is unproven. The three additional
waiting regions are two partial painting views and a table view. This tradeoff
alone would justify further testing rather than rejection.

The masked floor introduces 23 new tentative wrong pairs despite removing the
target error. Restricted exemplar updates introduce ten new clear wrong pairs,
mixing cabinet and painting observations. Those tests demonstrate why preventing
one attachment is insufficient evidence for deploying a fix.

## Longer recording: reject the blanket rule, keep the diagnosis

The evidence gate was next replayed over all 516 frames of the development
recording in both object folds. Weights were fitted only on objects outside the
scored fold; all records of one physical identity stay together. The disputed
61656 label is excluded and head/body share one person identity. Each evaluation
pair counts once. These are development labels, not independent acceptance, and
the fold weights differ from the default weights used on the short recording.

At the end, correct pairs fall from 3,716 to 2,893 and wrong pairs from 710 to
462. That net improvement in wrong pairs hides five new wrong pairs: one
cabinet/armchair pair and four painting/chair pairs. There are 881 lost correct
pairs and only 58 recovered ones. Of the losses, 613 concern the cabinet, 96 the
green painting, 68 the person, 45 the cattle painting, 38 the lamp, 12 the desk
and nine the armchair. Pair counts are not numbers of damaged observations.
The concentration on the cabinet matters and merits inspecting its split views.
Only two labelled main observations are waiting at the end; much of the lost
pair coverage is fragmentation, not observations simply being refused forever.

| Frame-count checkpoint | Original correct / wrong pairs | Candidate correct / wrong pairs |
|---|---:|---:|
| 103 | 124 / 22 | 124 / 22 |
| 206 | 394 / 120 | 358 / 113 |
| 310 | 1,105 / 247 | 959 / 194 |
| 413 | 2,174 / 472 | 1,717 / 311 |
| 516 | 3,716 / 710 | 2,893 / 462 |

This rejects this blanket live gate, not the appearance model or revocable
reader grouping. The latter's previously measured duplicate-recovery gain is a
separate result. The next task is narrower diagnosis of mixed exemplar histories
and joint repair of neighbouring records, starting with the cabinet and the
painting/chair cases. Measure newly introduced wrong relations, displaced good
observations and fragmentation separately rather than deciding from net pair
totals or from one corrected photograph.

## Artifacts and verification

`replay_attachment.py` and the optional `replay_experiment.py --attachment-gate`
are workstation tools. They alter disposable stores only. The full fresh and
held-out outputs are retained under
`captures/2026-10-04-reader-validation/attachment-replay/`; the baseline decision
snapshot remains in the worktree's `.cache/attachment-replay/inferred-background`.
The [compact result](2026-10-04-attachment-replay.json) records hashes, counts,
counterexamples and the schedule limitation. Sixteen evaluation/review tests
pass, including missing/ambiguous/backend-mismatched evidence abstention and
restoration of experimental hooks on failure. Documentation links resolve.
No registered component changed, so no deployment or restart was required.
