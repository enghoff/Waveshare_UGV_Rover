# Joint repair helps, but creates fresh chair/table mistakes

Rebuilding mixed records together is worth pursuing: on the longer saved recording,
repair followed by reader grouping removes 60% of labelled wrong relations while
retaining 98% of the baseline's correct relations. It is not ready to use on the
rover. The fresh recording reveals new chair/table mistakes that the clear-label
score alone misses. R-WS-13 stays open; R-WS-17 and R-WS-18 stay proposed.

## What was tested

The earlier repair bench selected overlapping neighbourhoods around labelled
records. This experiment instead partitions every placed record into deterministic,
disjoint spatial tiles. Each assigned observation is decided once. The target is
512 observations per tile; an existing oversized record would be reported rather
than silently split. Both recordings have 58 tiles and none is oversized. Pending
observations remain pending. Missing or incomparable appearance evidence remains
unchanged; none of the assigned observations in these recordings needed that fallback.

Within each tile, observations start separately and join using the earlier bench's
sum of appearance evidence, with fixed threshold 1.0 and geometry veto 2.5. A union
without a geometry fit is refused; the old bench allowed it. Same-picture and
embedding-backend conflicts propagate through joins. Geometry checks the median
standardized miss on the smaller side of a join, not every constituent. Singleton
pieces are released to waiting. These rules were fixed before scoring this run.

The second stage runs the existing reader grouping preview over reconstructed
records in an empty temporary store. It uses their fitted placements and copied
observation evidence, without old record names, descriptions or exemplar caches.
Groups can cross tile boundaries. Both stages are disposable identity proposals;
they do not supply persistent IDs, reader destinations, rollback or resolver
rejection records. No source snapshot or rover service was changed.

The longer recording has 3,540 observations: 2,979 assigned and 561 pending.
Appearance weights are fitted separately in two physical-object folds, excluding
all records of each test-fold object. Cross-fold pairs compare a held-out object
with a training-fold object; this is not the stricter appearance audit that holds
out both objects of every pair. Each pair is scored once. Observation 61656 is
excluded as disputed; head/body use the audited one-person policy. These are
development labels, not independent truth. The fresh recording uses the existing
default weights with no fitting or tuning, its full historical store, and the
previously frozen coding-agent drafts.

## Results, including the costs

| Longer recording, two folds combined | Correct pairs together | Wrong pairs together | Labelled main observations waiting |
|---|---:|---:|---:|
| Original records | 3,716 | 710 | 0 |
| Repair proposals | 3,390 | 197 | 9 |
| Repair then reader groups | 3,658 | 282 | 9 |

Neither candidate adds a labelled wrong pair relative to the original longer
recording. However, grouping restores **85 wrong pairs** removed by repair, along
with 268 correct pairs. The final result still contains 282 wrong pairs and loses
58 correct pairs overall. It fails the plan's requirement that no two labelled
objects share a thing. Net improvement is not acceptance. Spatial tile boundaries,
singletons and the same-picture prohibition can also fragment valid objects and
parts. Pending evidence is not repaired by this experiment.

| Fresh recording, 146 clear object drafts | Correct pairs together | Wrong pairs together | Waiting |
|---|---:|---:|---:|
| Original records | 284 | 3 | 39 |
| Repair proposals | 384 | 0 | 42 |
| Repair then reader groups | 439 | 0 | 42 |

The painting observation 64638 is separated from table record `object:415`.
This reproduces an improvement on real saved evidence, but does not establish
that the rest of the table's history is pure.

Including all 201 tentative object drafts changes the fresh result materially:
wrong pairs rise from 34 to 72 after repair and 78 after grouping. Correct pairs
rise from 407 to 487 and 548; waiting rises from 42 to 45. There are **55 newly
wrong draft pairs** after both stages, despite the apparently clean primary score.
They comprise 29 doorway/chair pairs, 20 pairs between tentatively distinguished
chairs, and six chair/table pairs. Those chair identities and overlapping doorway
masks remain uncertain; the sensitivity cannot be presented as established truth.

Post-score image inspection does confirm a narrower fault: chair selections 64520
and 64692 are newly joined to table selections 64581, 64592 and 64735. The saved
masks select chairs in the former and the table structure excluding the chairs in
the latter. Identifying which similar chair is unnecessary to establish these six
cross-object errors. All five observations end up in `repair:3:0:2` before and after
grouping. Thus the first repair stage itself makes this mistake; grouping is not
its sole cause. The original drafts were kept unchanged. This is an explicit
post-score analyst audit, not new blinded acceptance labels.

## Decision and next experiment

Keep joint repair as an experimental direction, and reject this candidate for
automatic use. Rejecting its deployment does not show that all joint repair is
wrong. The new result is stronger than the earlier overlapping-neighbourhood
claim, and the fresh counterexample is stronger than an aggregate score alone.

The join trace rules out a simple constituent-positive fix. Table 64735 joins a
66-observation cluster containing both chairs with mean appearance evidence 2.344
and geometry miss 1.899; table 64581 joins the resulting 67-observation cluster with
mean evidence 2.246, minimum 1.113 and miss 0.873. Table 64592 joins a later
74-observation cluster with mean evidence 1.751 and miss 0.987. The appearance
threshold is 1.0 and geometry threshold 2.5. Each smaller side here is a singleton:
changing its median geometry check to a maximum would make no difference.

All six direct chair/table appearance scores are also above 1.0 (1.509 to 3.132),
including masked evidence. Requiring every direct pair to pass the existing test
cannot distinguish these observations. The trace and direct features are recorded
in the summary. It does not prove whether context in image embeddings, semantic
similarity, mask quality, or geometry uncertainty is the principal cause.

Next inspect those observation features and their provenance, separating selected
object pixels from shared context and distinguishing part/parent relations from
neighbouring objects. Test whether that evidence can distinguish chairs from the
table before prescribing another linkage rule or blanket threshold. Include
conflicting constituents in proposal review and abstain where identity is unsupported.
Fix the six confirmed errors without new wrong pairs and without
erasing the longer-recording gains; retain the uncertain cases as a separately
reported sensitivity. Reattachment prevention and stable reviewed proposals are
still outstanding. Observe an eventual passing change on the rover before treating
it as finished production work. The rover need not be online for this next replay.

## Reproduction and artifacts

Commands and algorithm boundaries are in the
[experiment README](../../experiments/entity_association/README.md#bounded-joint-repair).
The [measurement summary](2026-10-04-bounded-identity-repair.json) records input and
private artifact hashes, per-fold scores, new wrong pairs and the post-score audit.
Full proposals are preserved under the private capture directory. The source
snapshots remain byte-identical. Twenty experiment/review contract tests and the
documentation validator passed. Only experiments and documentation changed;
there is no registered component change to deploy, and no rover movement occurred.
