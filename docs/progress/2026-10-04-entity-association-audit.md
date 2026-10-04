# The identity tests need better labels; groups can preserve the resolver's gain

**Grouping duplicate records for readers, while leaving the resolver's inputs alone,
passes the five development checkpoints if a person's head and body count as one
person.** With that interpretation and a disputed painting observation excluded,
it keeps 216 more same-object pairs together at the end without adding a known
different-object pair. Its apparent earlier failure was entirely head-versus-body
pairs marked as different objects. This is a promising candidate, not acceptance:
the identity policy awaits the owner, the recording has been used repeatedly, and
most labels are the coding agent's. R-WS-13 remains open; R-WS-17 and R-WS-18 remain
proposed. No running service or world database was changed.

## What was reproduced

Map session 67 in `captures/2026-10-04-association-likelihood/world.db`: 3,540
regions in 516 recorded looks, with its saved occupancy map. Source was frozen at
`30c3bce5c50f73ffbc3ec9271f7c9a14044dc919`, independently of another session's dirty
merging and autonomy edits. The unchanged resolver reproduced **every one of the
3,540 previous replay memberships**, including pending observations: 182 entities,
2,887 attached regions. Source database hashes remained unchanged through replay.

The workstation harness is [experiments/entity_association](../../experiments/entity_association/README.md).
Raw outputs and traces are in `.cache/entity-audit/`. The
[compact results](2026-10-04-entity-association-audit.json) include recording and label
hashes, every checkpoint, training exclusions, and the two traced failures.

## What the old evaluation got wrong

**Pairs were counted twice across the folds.** The published 4,357 same-object pairs
are 4,213 unique pairs, and 32,520 pairs of either kind appear in both fold totals.
The new scorer assigns each known relation once. Neither the old nor the new pair
count is a count of independent trials: many pairs come from the same object and
nearby pictures. Large objects receive more weight; per-label-group scores are
reported separately. A floor/skirting label remains in those development scores,
so the group counts are not an acceptance census of real objects.

**The appearance AUC test did not hold out both objects as it claimed.** It excluded
one entity record at a time. Also, the two green-painting records crossed the old
replay's folds, so the same physical object could appear in training and testing.
The new appearance test excludes both physical objects of each scored pair. The
new replay excludes all records of each test object from training. Cross-fold
evaluation pairs compare a held-out object to a known object and are assigned to
one fold; they are distinct from the stricter appearance test.

**The supposed same-picture duplicate was a label problem.** Observations 61654 and
61656 in frame `20261003-091409-2e7688` were both labelled as the cow painting. The
original photo puts 61654 on the painting beside the cabinet and 61656 on a different
framed picture beyond the door. That cannot establish duplicate boxes on one object.
The earlier review's confident claim is withdrawn. `label_review.json` preserves
the finding; the sensitivity runs exclude 61656 from training and scoring, without
editing the old labels or inventing its true identity. Owner confirmation is still
owed. With that exclusion, none of the audited same-object pairs shares a picture.
This does not prove that duplicate boxes never occur elsewhere.

**Head and body were silently defined as different objects.** `object:332` is "the
owner, seated" and `object:385` is "the owner's head". All 44 apparent extra errors
at the fourth grouping checkpoint join those two records. The crop sheets support
their being views of one person. Whether a head should also be a separately represented
part needs an explicit policy; it should not silently condemn a person-level grouping.

## What grouping actually did

Every five minutes of recorded time, the experiment merges a disposable copy of the
resolver's store. Readers receive the resulting aliases, and the resolver continues
with its original entities, positions and exemplars. The complete final memberships
remain identical to baseline in every grouping run. Groups are recomputed from scratch,
so a later pass can withdraw a grouping.

The experiment retains shared-picture and backend checks and adds R-WS-8's hard
geometric rejection before ranking: squared ellipse distance at most 13.8155, the
nominal 99.9% two-dimensional Gaussian limit. The ellipses are not calibrated to
that confidence, so this is a threshold, not a probability guarantee. Nothing here
adds that limit to the rover's deployed merge call.

Two complete grouping replays were run for each interpretation: original labels,
61656 excluded, and 61656 excluded with head/body one person. In the last case the
appearance model was **retrained**, holding both person records out together and
preserving the other objects' folds. This was not merely a favourable rescore.

| Checkpoint, by look count | Baseline same-object pairs together | Grouped same-object pairs together | Baseline / grouped different-object pairs together |
|---|---:|---:|---:|
| 103 | 124 | 124 | 22 / 22 |
| 206 | 394 | 394 | 120 / 120 |
| 310 | 1,105 | 1,165 | 247 / 247 |
| 413 | 2,174 | 2,218 | 472 / 472 |
| 516 | 3,716 | 3,932 | 710 / 710 |

At every checkpoint grouping preserves or improves same-object pairing and adds no
different-object pairing under this policy. A freshly recomputed grouping gives the
same counts as the scheduled view at those checkpoints. Under the original
head-versus-body definition the fourth checkpoint instead adds 44 "wrong" pairs.

This does not fix contamination: all 46 originally labelled odd looks remain with
their original main objects. Some duplicates also remain. It tests identity grouping,
not whether a reader's grouped position, executive memory or stable names work correctly.
Five checkpoints do not prove that no intervening view was wrong.

## Why changing the resolver's own entities split things

The five-minute live-merge replay was rerun with physical-object folds. Under the
original labels it retained 3,389 same-object pairs against baseline's 3,721, while
reducing different-object pairs from 710 to 437. That is a tradeoff, not proof that
all live revision is worse. These two specific refusals were traced again with 61656
excluded, using the same frozen resolver:

- **Green painting, observation 62091, look 239:** baseline accepts its entity with
  appearance 0.653 and all geometry checks passing. After live merging, appearance
  improves to 0.765, but the entity's position fails the map visibility check. Its
  distance from this viewpoint has changed from 2.35 m to 3.67 m; range and height
  still pass. No other region in that picture has already claimed it. The observation
  is therefore left unassigned at this step.
- **Cow painting, observation 62893, look 338:** baseline accepts at appearance 0.578.
  The live-merged entity still passes geometry, but its current exemplars score 0.5496,
  just below the fixed 0.55 rejection threshold. It is not occupied in this picture.
  The observation joins another entity instead.

These establish the immediate rejection mechanisms for two cases. They do not isolate
which earlier merge, refit or subsequent attachment created every bad state, nor do
they explain the whole regression. A controlled ablation is still needed before fixing
live membership revision. The duplicate-box mechanism remains unproved here.

## What the appearance score supports

With both objects excluded from training and 61656 excluded, all three crop features
give AUC 0.9485 overall, 0.9503 against different labelled objects, and only 0.7921
against wrong looks already inside their own entity. Plain DINO gives 0.9392 overall
and 0.7455 on that hard comparison. Under the one-person policy, all-three AUC is
0.9439 overall and 0.7740 on the hard comparison. Appearance remains useful, but its
easy overall score does not prove that it can clean up mixed entities.

The earlier 0.956 and these scores use different exclusions, same-frame scoring and
prior normalization; the difference cannot be attributed solely to training leakage.
AUC measures ranking. Subtracting a fitted model's training prior does not establish
calibration for the actual geometry-admitted candidate population.

## Verification and what follows

Eight evaluation-contract tests pass, including unique pair allocation, complete
duplicate holdout, unknown odd identities, pending observations, and preserving
historical labels. Documentation links and requirement identifiers were checked.
The tools live outside registered components and need no deployment or restart.
The rover did not answer a TCP 8769 status check or the SSH alias, so live observation
and a fresh acceptance drive were not possible. No identity requirement is closed.

The blinded pilot pack is `.cache/entity-audit/owner-review/review.html`, with 101
numbered regions and a blank `labels.csv`. It samples available frames without old
entity identities; missing frames are reported. It is still this development recording,
not independent acceptance.

Next: settle object-versus-part identity and review the disputed labels, then judge
groups on a fresh independently labelled drive. In parallel, trace the exact state
change causing the two refusals before trying a narrow placement/exemplar safeguard.
Do not add another automatic membership rewrite on the strength of these results.
