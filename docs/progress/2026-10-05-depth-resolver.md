# Withholding ambiguous depth corrects one painting attachment; blanket use remains unproved

Depth ambiguity is a reproduced contributor to incorrect entity matching. In an
exact replay of the latest recording, withholding the fixed flagged ranges moves
painting observation 68640 from a chair record to a record containing that same
painting. All 15 existing frozen same-subject connections survive. Preserve this
positive result; the candidate is not another total failure. It is also not ready
for blanket deployment: an older complete-drive sensitivity loses genuine cabinet
connections, and narrow/transparent objects still lack measured controls.

R-WS-13 remains open; R-WS-17 and R-WS-18 remain proposed. Nothing on the rover
was called, changed or restarted. It stayed parked for charging. All work was
offline with original recordings opened read-only.

## Criteria and controls

The complete-drive criteria were committed in ef9b646 before measurement, and the
bounded fresh reconstruction in fc11b8b before its measurement. The rule is
unchanged: withhold only a stored range whose reconstructed production outline
band has less than half the valid samples and lies more than 0.5 m ahead of the
median. Never choose the farther surface, drop the observation, re-encode a vector
or invent a missing range. Box fallback is unaffected.

The taped experiment reconstructs production outline ranges for all originally
ranged observations with usable archived images/depth, then compares those ranges
with the same ranges selectively withheld. Original stored ranges are a third
arm. Original missing ranges remain missing. It uses one resolver pass per
inspection and the saved October 2 map: a complete fixed offline sequence, not
the recorded live schedule. Original and outline controls each repeat exactly.
Instrumentation of actual founding calls reproduces every arm's final memberships
and placements, including the uninstrumented runs.

The original-range controls differ from the recorded final owner IDs in 402, 10
and 50 observations respectively, and their placements differ. These are strict
state comparisons, not physical identity errors: creation-order changes also
change IDs. Thus the taped experiment is a sensitivity, not proof of the historical
live faults. Original target-to-entity mappings are proxy labels; broad and mixed
regions are retained in the primary score rather than quietly relabelled.

## Complete taped-drive sensitivity

All 1,871 observations in 228 inspections are replayed. Of 372 originally ranged
observations, 369 can be remeasured and three lack a usable mask. The rule flags
52 ranges across all subjects, rather than selecting the seven labelled cases
from the preceding distance pilot. Flags number 40, seven and five by drive.

| Drive | Outline same-target pairs | Abstention pairs | Existing pairs lost | New pairs | Entities, outline to abstention | Waiting, outline to abstention |
|---|---:|---:|---:|---:|---|---|
| October 1 | 557 | 559 | 58 | 60 | 79 to 73 | 168 to 171 |
| October 2 first | 31 | 31 | 0 | 0 | 19 to 18 | 589 to 592 |
| October 2 redo | 50 | 50 | 0 | 0 | 27 to 28 | 507 to 507 |

Pooled retention is 580 of 638 existing proxy same-target connections, or 90.9%,
below the predeclared 98%. Assigned target counts meet retention; there are no
new cross-target pairs. The control has no cross-target pairs to remove, so this
corpus cannot demonstrate reduced identity contamination. The cabinet's majority
position improves from 14.9 cm to 5.8 cm error on October 1; other scored majority
positions are unchanged. Median improvement is zero, below the fixed 10 cm
placement-benefit criterion. The already-unplaced tissue-box target stays unplaced.

Eleven flagged observations found single-range entities in the outline controls;
none found a single-range entity under abstention. Flagged observations still
participate in three two-bearing foundations across the drives. Retaining the
observations therefore does preserve routes to later placement. It does not
guarantee preservation of their previous correct connections.

Post-score photographs show that lost pair 48293/48447 consists of two clear views
of the same black cabinet. The connection loss cannot all be dismissed as proxy
label noise. Conversely, mixed regions and tiny parts make the entire pair count
unsuitable as an independent accuracy estimate. No labels were changed to rescue
the criterion. A small net pair gain hides replacement of existing connections.

## Latest live state reproduced exactly

The October 5 before/after snapshots contain 7,982 final observations, including
125 new observations in 14 nonempty inspections. Surviving changed placement
timestamps were clustered into passes separated by more than two seconds and
mapped to the last completed inspection. The predetermined boundaries are
1, 2, 3, 6, 8, 8, 9, 12, 13 and 14. Repeated passes after frame 8 are preserved.

This one reconstruction reproduces all 7,982 recorded assignments, the complete
entity set, every plain/masked exemplar byte and every placement exactly. A second
control is identical. It is an exact **final-state** reproduction, not a recovered
log of every live call: no-change/overwritten passes and concurrent captures remain
unlogged, and the initial map is a frozen sensitivity. This closes the earlier
provenance limit far enough to test this recording, without claiming to have
recovered its full execution history.

All 25 stored ranges in the five archived depth frames reproduce exactly from
stored outlines or their original box fallback. Four are flagged: 68578, 68582,
68599 and 68640. There is no reranging intervention in this fresh experiment.
Withholding only those four ranges changes six assignments and seven placements;
it creates no entity-set change. All 15 frozen same-subject connections survive,
with no new same-subject or cross-subject pairs in the frozen fresh labels.

Painting 68640 moves from object:375 to object:330. Post-score historical photo
review shows dining chair observation 60724 in the original record and clear views
61104/61340 of the same green landscape painting in the destination. This is a
concrete corrected attachment. That review is diagnostic evidence, not a frozen
primary acceptance label; it does not establish purity of either whole historical
record. Other fresh painting views stay split or waiting, so cross-view recognition
is still unsolved. The fresh pair-preservation criterion passes; the frozen fresh
pair score establishes no new painting connections.

The other five changed assignments are reviewed and retained separately. A doorway
region becomes waiting; mixed/edge chair-table regions change records; valid chair
part 68693 becomes waiting. Individual chair identity is unresolved. The clean
fresh score therefore does not establish safety for all regions. There is no new
taped distance and no numerical placement-accuracy claim for this drive.

## Decision and next evidence

The blanket conclusion that nothing tried can correct an attachment is too strong.
This fixed rule corrects one real attachment in a model that reproduces the live
final state. The blanket conclusion that this rule is safe to activate is also
unsupported: the older sensitivity loses genuine connections, and the frozen
fresh labels do not cover every affected region. Keep the successful range flag
and corrected case; do not deploy a mandatory gate or loosen the contamination
requirement to make an experiment pass.

The next decisive evidence needs the rover: a measured foreground/background scene
plus transparent and narrow objects, exact resolver/capture order and contemporaneous
map provenance. First verify optional recording at rest; then use a controlled
drive for independent bearings. Further threshold guessing on these same labels
would not settle hardware fidelity or minority-surface truth.

The [measurement bundle](2026-10-05-depth-resolver.json) preserves all rows,
criteria commits, input/code hashes, founding traces, fresh reproduction differences
and post-score review provenance. Full arm states and review photos are archived
under `captures/2026-10-05-depth-resolver/`. An early fresh run stopped on a relative
path error before replay; a second completed all arms but stopped while serialising
a NumPy Boolean. The corrected third run retained the same arm states. Those
incomplete outputs remain in separate scratch directories and are not evidence
of a candidate failure. No source recording changed.
