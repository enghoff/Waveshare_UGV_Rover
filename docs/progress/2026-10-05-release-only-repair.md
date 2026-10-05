# Releasing isolated observations is promising; automatic repair remains unproven

Removing only isolated observations preserves substantially more correct identity
links than splitting every proposed cluster. On the older recording it removes
281 of 710 scored wrong pairs while losing 61 of 3,716 correct pairs. Keep this
as a candidate for reviewed repair. Do not reject it on the strength of the new
apparent table/painting error alone: inspecting that region shows mixed selected
pixels, making the draft label's confidence questionable. R-WS-13 stays open;
R-WS-17 and R-WS-18 stay proposed. Nothing was applied to the rover.

## What was compared

The experiment retains the previous three-channel appearance model, sum linkage,
threshold and geometry veto. Each original placed record is analysed independently.
Previously pending observations remain pending. Full splitting keeps each resulting
multi-observation cluster separately and releases singletons. Release-only uses
the same clusters but releases only those singletons, retaining every other original
membership. No subsequent reader grouping is allowed in either proposal.

An executable invariant checks complete observation coverage, forbids assignment
of originally pending observations, and requires every proposed record to descend
from exactly one original record. Thus neither proposal can introduce a new pair
of observations together. This guarantee ends when the ordinary resolver runs.

The two older physical-object folds use the established labels, excluding disputed
61656 and treating head/body as one person. Every evaluation pair is counted once;
cross-fold pairs can include one training object. Fresh scoring uses coefficients
fitted on all older development evidence, never the fresh labels. These remain
analyst drafts, not independent acceptance evidence. Only placed histories with
usable features are analysed; this is not a complete repair of every observation.

| Older recording | Correct pairs together | Wrong pairs together | Main observations waiting |
|---|---:|---:|---:|
| Original | 3,716 | 710 | 0 |
| Full split | 3,288 | 242 | 9 |
| Release only | 3,655 | 429 | 9 |

Full splitting loses 11.5% of existing correct pairs, including 223 cabinet pairs
and 54 floor-lamp pairs. Release-only loses 1.64% while removing 39.6% of wrong
pairs. Nine of 365 main labelled observations return to waiting. These are pair
and observation measures respectively; the pair loss is not an object-loss rate.
Neither method resolves all mixed histories or joins fragmented identities.

| Fresh frozen draft | Clear correct pairs | Clear wrong pairs | Clear waiting | Tentative wrong pairs |
|---|---:|---:|---:|---:|
| Original | 284 | 3 | 39 | 34 |
| Full split | 274 | 0 | 42 | 27 |
| Release only | 284 | 0 | 42 | 29 |

The clear score covers 146 observations of 16 identities; the tentative score
includes uncertain identities. Release-only retains all 407 tentative correct
pairs as well. It proposes releasing **200 observations across the whole saved
store**, not just the three newly waiting clear observations. Most have no fresh
label, so the fresh score cannot establish that all 200 releases are sound.
The known green painting observation 64638 is released from table record 415;
the table observations remain together. No new appearance weights were selected
using these outcomes.

## Release is not the end of the lifecycle

Three idle resolver passes were run on disposable copies of the fresh snapshot,
using its frozen occupancy map and the existing experimental detach procedure.
That procedure refreshes counts, rebuilds bounded exemplar history and refits
placements. There were three arms: untouched baseline, release followed by the
normal resolver, and release with a temporary refusal of the original parent.
The release IDs exactly match the release-only proposal.

| After three passes | Clear correct pairs | Clear wrong pairs | Clear waiting | Released observations back in original parent |
|---|---:|---:|---:|---:|
| Untouched baseline | 284 | 3 | 39 | Not applicable |
| Release, normal resolver | 314 | 1 | 32 | 36 |
| Release, temporary refusal | 305 | 1 | 33 | 0 |

Both release arms put painting 64638 into green-painting record 351. This fixes
the particular table attachment in this replay; it does not prove all of record
351's history is pure. The baseline remains unchanged. The temporary refusal
stops return to the original parent across these passes, but neither establishes
durability after restart nor correctness of a new destination. No new observations
were supplied, and three passes are not evidence of steady-state or driving safety.

## A scored error whose visual evidence needs qualification

Both release arms introduce the same newly scored wrong pair, 64601/64625.
Observation 64625 leaves record 302 and joins record 247, which already contains
64601. The frozen draft calls 64601 a dining-table part and 64625 the green painting.
On those labels the pair is wrong, and the reported primary score keeps it wrong.

Post-score inspection of both full frames and reconstructed saved masks changes
the confidence of that interpretation. The painting's selected pixels retain a
prominent foreground chair back. The narrow region called a table part selects
thin horizontal/vertical furniture structure; its earlier note about a brown
angled table leg is not clearly supported by the selected pixels. These are not
two unambiguous isolated object views. The painting label describes the intended
background subject but does not establish a pure appearance input.

Do not silently rewrite the frozen labels to improve the score, call this a
confirmed clean-object misassociation, or claim the new attachment is correct.
The observed fact is rerouting of a mixed region; single-object ownership is
disputed. The paired full-frame and selected-pixel images and original label notes
are preserved with the private results. This doubt is narrower than the six
previously inspected chair/table errors from joint repair and does not erase them.

## Decision and next step

Retain release-only for a reviewed proposal path; full splitting has too much
measured fragmentation to prefer here. Before automatic application, audit the
200 proposed releases by their actual selected pixels and multi-view support,
separating mixed detections, valid parts and clear outliers. Freeze that audit
before another score. Trace evidence for the new destination of disputed regions
and test any resulting policy on both recordings, not just this pair. A mixed
region should not be used as confident single-object truth by the evaluator or
treated as proof for an automatic identity change.

The temporary refusal experiment supports pursuing durable rejection memory, but
does not by itself approve that integration. R-WS-17/R-WS-18 acceptance and hardware
validation remain outstanding. The source snapshots were unchanged; all 26
experiment tests passed. Only experiments and documentation changed, so no service
deployment or restart was needed. The rover was not used or driven in this step.
The [measurement summary](2026-10-05-release-only-repair.json) records hashes,
fold results, lifecycle checkpoints and the post-score label dispute.
