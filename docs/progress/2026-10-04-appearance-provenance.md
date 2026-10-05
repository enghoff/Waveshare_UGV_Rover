# Masked semantic evidence merits a test; simpler feature changes do not fix identity

The chair/table fault survives removing the semantic feature. Using masked
appearance alone removes four of its six wrong relations, but creates new errors
and loses correct relations elsewhere. Tightening the DINO mask has essentially
no overall benefit on the fresh recording. These are measured tradeoffs, not
reasons to abandon joint repair.

A new diagnostic is more specific: semantic features computed from selected
object pixels reduce similarity for all six wrong chair/table pairs while keeping
the three same-table pairs similar. Overall pair discrimination is roughly
unchanged. Retain this as a candidate additional feature, not a proven repair or
replacement for the existing features. No production code changed. R-WS-13 stays
open; R-WS-17 and R-WS-18 stay proposed.

## What the saved features actually describe

The source sends the padded, unmasked crop to both DINO and SigLIP. Only the second
DINO feature is masked. In the deployed appearance score, semantic similarity has
weight 15.276 versus 5.760 for plain DINO and 2.959 for masked DINO. That establishes
how evidence is combined, not which channel caused a mistake.

There is also a provenance gap. Appearance masks extend over a crop padded by
2% of the frame on every side. The saved outline is clipped to the detection box
and reduced to half resolution. Thus a contact sheet drawn from the outline is
not an exact rendering of the masked appearance input. A synthetic exercise of
the actual preprocessing retains 164 selected pixels outside a box which its
saved outline omits. This reproduces the preprocessing distinction, not a real
attachment fault.

For table observations 64581, 64592 and 64735, the stored selected-pixel shares of
the padded crop are 0.266, 0.171 and 0.244. Reconstructed clipped outlines cover
approximately 0.168, 0.101 and 0.160 of those same padded areas. The difference is
substantial, but rounded boxes and sampled masks prevent exact attribution to
padding. Both chairs lack measured range; the three table observations have
outline-derived ranges. All five carry full DINO, masked DINO and SigLIP vectors
and the `tensorrt` backend marker. Missing vectors or fallback to plain appearance
do not explain this particular fault. The backend marker does not record exact
model artifact hashes.

## Removing channels, with refitting rather than arbitrary weight changes

Two ablations were specified before scoring: remove semantic similarity, or use
masked DINO only. Both retain the joint-repair algorithm and thresholds from the
[previous trial](2026-10-04-bounded-identity-repair.md). Weights and intercept are
refitted on the older development labels, preserving its physical-object folds,
disputed-label exclusion and one-person policy. Fresh labels are never used for
fitting. A freshly fitted three-channel control separates channel removal from
the older deployed weights and their original labels.

| Longer recording, repair plus reader grouping | Correct pairs | Wrong pairs | Main observations waiting | New wrong pairs versus original records |
|---|---:|---:|---:|---:|
| Original records | 3,716 | 710 | 0 | 0 |
| Three-channel control | 3,658 | 282 | 9 | 0 |
| Plain and masked DINO | 3,585 | 253 | 8 | 0 |
| Masked DINO alone | 3,319 | 252 | 18 | 33 |

The full-feature fold models remain numerically identical to the previous trial.
Each evaluation pair is counted once. Cross-fold pairs compare a held-out object
with a training-fold object; this is not both-object holdout for every pair.

| Fresh recording, repair plus reader grouping | Clear correct pairs | Clear wrong pairs | Clear waiting | Tentative wrong pairs | Six confirmed chair/table mistakes remaining |
|---|---:|---:|---:|---:|---:|
| Original records | 284 | 3 | 39 | 34 | 0 |
| Refit three-channel control | 475 | 0 | 42 | 78 | 6 |
| Plain and masked DINO | 478 | 0 | 41 | 59 | 6 |
| Masked DINO alone | 431 | 10 | 41 | 55 | 2 |

The six target errors were introduced by repair, so the original-record row has
zero of those particular pairs, despite its other faults. Masked-only grouping
newly mixes a snowy-trees painting observation with ten observations of a different
building painting. Removing semantic evidence is therefore not sufficient, and
masked-only identity is not an acceptable substitute. The fresh clear score uses
146 object drafts; the sensitivity uses all 201 tentative object drafts. Neither
is independent acceptance.

## Re-encoding the actual photographs locally

The workstation ran the same DINOv2-small and SigLIP2 vision fp16 ONNX exports
named by the rover installer, using ONNX Runtime 1.30.0 on the CPU. Model hashes,
source hashes and vectors are preserved in the measurement artifacts. Each of the
232 fresh observations was encoded twice: once with the existing plain crop, and
once with pixels outside the reconstructed box-clipped outline set to the existing
grey value. The plain run is the fidelity check; the clipped run is the intervention.

For the five target observations, plain-feature agreement with the stored vectors
exceeds 0.9999 for DINO and 0.999998 for SigLIP. Across all 232, median agreement is
0.999910 and 0.999999 respectively. The minima are 0.9871 and 0.9692; three DINO
and twelve SigLIP observations fall below 0.99. Rounded stored boxes, reconstructed
masks and CPU execution limit exact replay, even though the target plain crops
reproduce closely.

Clipped DINO decreases similarity for three wrong chair/table pairs but increases
it for three. Clear-label pair AUC changes from 0.95031 to 0.94993; tentative-label
AUC from 0.96639 to 0.96607. This does not support a blanket clipping fix.

Masked SigLIP decreases similarity for all six wrong pairs, from 0.830–0.908 to
0.757–0.794. The three known same-table pairs remain at 0.874–0.943, compared with
0.857–0.959 originally. Clear-label AUC changes from 0.94099 to 0.94159; tentative
AUC from 0.95031 to 0.95301. Those small aggregate differences establish neither
significance nor a useful identity threshold. Correlated observation pairs are not
independent trials. No existing identity weights were reused with the new channel,
and no fresh labels were used to fit new ones.
The locally re-encoded plain SigLIP control has AUC 0.94128 on the clear drafts,
so part of the already-small difference from the stored baseline comes from the
re-encoding conditions rather than masking.

## Decision, remaining doubts and the next step

Keep masked semantic evidence as a candidate additional channel. Recover the
older photos, compute it for the older development recording, and fit and test it
under the same physical-object folds before another full-store repair. Preserve
plain context as a separate measurement so the experiment can test whether masked
semantics adds information, rather than assuming it should replace context.

Only 48 of the older recording's 516 frames are available locally, covering 49 of
411 labelled observations. A read-only SSH request for a historical frame timed
out during banner exchange; the direct SSH address also failed to connect. No authenticated rover session, deployment, restart
or movement occurred. The next full comparison needs those historical photographs
from the rover or an archive; it does not need another drive. Do not fit on the
fresh diagnostic recording to get around the missing training pictures.

Review should focus on three unresolved questions: whether full-resolution,
box-clipped masks behave like these reconstructed masks; whether an additional
semantic channel preserves partial-view matches under object holdout; and whether
uncertain/mixed detections should abstain rather than acquire single-object identity.
The current diagnostic does not settle any of them. Future acceptance still needs
independent labels and a hardware observation of a passing change.

The [measurement summary](2026-10-04-appearance-provenance.json) contains per-fold
models, errors, vector comparisons and artifact hashes. The
[experiment README](../../experiments/entity_association/README.md#appearance-provenance-and-channel-tests)
contains reproduction instructions. All 22 experiment tests passed, original
three-channel fold models were checked unchanged, and documentation links and
identifiers resolved. Only experiments and documentation changed.
