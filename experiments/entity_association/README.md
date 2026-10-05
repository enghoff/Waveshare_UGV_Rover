# Reviewing entity association

Workstation experiments for R-WS-13, R-WS-17 and R-WS-18. These scripts are outside
the deployment manifest. They change temporary replay stores only; they do not
change the running rover, the historical recording, or its original labels.

The recording is map session 67 in
`captures/2026-10-04-association-likelihood/world.db`, with its saved `map.json`.
The development labels are
`captures/2026-10-03-merge-review/look_labels.json`. They are mostly the previous
coding agent's judgments; the owner reviewed only 19 uncertain observations.
Neither object-level cross-validation nor running a recording again makes it
an independent acceptance drive.

## Evaluation

`audit.py` counts each known relation between observations once. Duplicate records
of one physical object share a fold. Unassigned observations are never counted as
one identity. Individual dining chairs and an odd observation's actual identity
remain unknown; no scoring rule invents those labels. Mixing, splitting, waiting,
micro pair scores and per-object recall are reported separately.

`validate_model.py` evaluates appearance ranking with **both** physical objects in
each scored pair excluded from training. It reports the difficult comparison with
wrong observations already inside an entity separately from comparisons between
different labelled objects. AUC is a ranking measurement, not a calibration test.

`replay_experiment.py` uses a frozen committed resolver. `none` reproduces its
memberships; `live` merges the resolver's own entities every five minutes of
recorded time; `groups` merges disposable copies every five minutes and presents
those group aliases to readers only. Five equally spaced **frame-count** checkpoints
score both the scheduled reader view and a freshly recomputed view. All modes
resolve every recorded frame, including the 37 regions with invalid wall-clock
timestamps (they carry no bearings). No elapsed-time interpretation is claimed
for those rows. Group memberships are recomputed from scratch, not accumulated.

The grouping experiment imposes a squared ellipse-distance limit of 13.8155 before
ranking, corresponding to a nominal 99.9% two-dimensional Gaussian gate. The rover's
ellipse errors are not independently calibrated to that confidence; this is a
conservative experimental threshold, not proof of 99.9% correctness. Shared-picture
and perception-backend checks remain intact.

The replay models exclude all objects of the test fold from training. Cross-fold
evaluation pairs, which compare a held-out object with a known object, are assigned
to one fold by their canonical anchor. This is distinct from the stricter
leave-both-objects-out appearance test. Neither substitutes for a fresh session.

## Reproduce

Run from the repository root with Python, NumPy, SciPy, scikit-learn and Pillow.
Keep raw replay outputs under a new `.cache/` directory; source recordings stay
under `captures/` and are never committed.

```powershell
python experiments/entity_association/freeze_source.py --output .cache/entity-audit-new/source
python experiments/entity_association/audit.py --output .cache/entity-audit-new/audit.json
python experiments/entity_association/validate_model.py --output .cache/entity-audit-new/model.json
python experiments/entity_association/replay_experiment.py --source .cache/entity-audit-new/source --output .cache/entity-audit-new/baseline.json --fold 0 --mode none
python experiments/entity_association/replay_experiment.py --source .cache/entity-audit-new/source --output .cache/entity-audit-new/live-0.json --fold 0 --mode live --ungated
python experiments/entity_association/replay_experiment.py --source .cache/entity-audit-new/source --output .cache/entity-audit-new/live-1.json --fold 1 --mode live --ungated
python experiments/entity_association/replay_experiment.py --source .cache/entity-audit-new/source --output .cache/entity-audit-new/groups-0.json --fold 0 --mode groups
python experiments/entity_association/replay_experiment.py --source .cache/entity-audit-new/source --output .cache/entity-audit-new/groups-1.json --fold 1 --mode groups
python -m unittest discover -s experiments/entity_association -p test_audit.py
```

Repeat the audit, appearance validation and grouping replays with
`--exclude-observation 61656` for the sensitivity check in `label_review.json`.
Its original full frame contradicts the label that it is a duplicate detection
of observation 61654. The exclusion is an analyst judgment awaiting owner
confirmation, and does not overwrite the historical label or manufacture a new
identity. Excluding it also removes that supposed duplicate-box counterexample.

Repeat grouping and appearance validation with both `--exclude-observation 61656`
and `--same-person` to test the interpretation that the owner's head and seated
body are views of one person. The old test labels them as different identities;
all 44 apparent additional wrong pairs at checkpoint 413 are between those two
records. This policy needs the owner's confirmation. Both records are excluded
together from the new replay model's training when that person is held out, with
other objects' folds preserved. Simply relabelling the old output would not fix
the original training leakage.

Use `--trace` on the reviewed `groups --fold 0` and `live --fold 1 --ungated` runs
to record the green-painting and cow-painting gate diagnostics. Tracing is optional
and its overhead is substantial; elapsed time here is not a production latency
claim. `summarize.py --directory .cache/entity-audit --output <result.json>` checks
exact baseline memberships and grouping conservation before writing compact results.

With a source checkout containing `world_state/reader_groups.py`, add
`--reader-module` to `groups` runs to exercise the diagnostic implementation
instead of the earlier bench-only grouping function. The held-out appearance
weights are fitted as before. This flag validates the implementation against
the development recording; it does not validate the daemon's default weights
on independent data.

`prepare_review.py --output .cache/entity-audit-new/owner-review` produces full
frames with numbered regions, a blank CSV and a manifest for a blinded **pilot**
review. It samples frames independently of existing entities and reports missing
frames. Its source is still the development drive, so it cannot close acceptance.
Use that labeling format on a fresh recording, retain an untouched test session,
and agree the proposed R-WS-17/R-WS-18 tolerances before judging deployment.

For a separate recording, supply `prepare_review.py --database <snapshot.db>
--frames-dir <saved-frames> --after-observation <last-id-before-drive>
--frames <number> --output <new-review-directory>`. It opens the snapshot read-only,
includes both assigned and pending regions, hides entity assignments and existing
labels, and reports missing images and regions without bearings. The resulting
CSV stays blank for an independent reviewer. This mode omits the development
recording's disputed-label and head/body examples. A fresh but unlabelled recording
is still marked as unaccepted.

An analyst first pass stays separate from that blank sheet. Store its judgments,
confidence, review groups and explicit unconfirmed provenance in a draft JSON,
then run `review_first_pass.py --draft <draft.json> --questions <questions.json>
--database <snapshot.db> --frames-dir <saved-frames> --output <review.html>`.
It checks the snapshot hash and every source observation/frame/box, and shows
raw crops beside saved-outline selections for grouped reviewer questions.
It never reads existing entity assignments, edits labels or scores candidates.
Reviewers can reply by question number or download their free-text answers.
Unreviewed coding-agent labels do not become independent truth merely because
the uncertain cases received owner review.

When the owner delegates these judgments, proceed with analyst evaluation rather
than waiting for owner answers. Keep the original draft frozen, exclude uncertain
identities from the primary score, and report them separately as a sensitivity
check. `assess_recording.py --database <snapshot.db> --draft <draft.json>
--after-observation <last-id-before-drive> --output <assessment.json>` runs the
unchanged grouping preview on a temporary copy using its default weights. It
counts each fresh/fresh pair once, keeps waiting observations separate, and checks
the source snapshot hash and every label's provenance. No fitting or tuning takes
place. The preview still uses all prior history in the snapshot; this is not a
clean-room replay or an independent acceptance result. Mixed regions, surfaces,
unknown identities and missed objects do not become correct pairs by exclusion.

## Replaying an attachment from before/after snapshots

`replay_attachment.py` starts from the consistent before-snapshot and adds the
later snapshot's measured observations with their original IDs and vectors. It
captures the state immediately before a chosen observation is offered to placed
entities, its candidate gates and decision, then saves full results and a
disposable end store. `--recorded-schedule` honours which inferences actually
settled identity. `--background-after` adds explicitly listed resolver passes;
these are reconstructed events, not a claim that exact background timing was
logged. A supplied occupancy map remains a frozen sensitivity. Check original
memberships, exemplars and placements against the end snapshot before trusting
the replay. Use a new output directory each time.

For the fresh 27-frame recording, the surviving placement-update timestamps
support background passes after frames 7, 13, 16, 22, 25 and 27. With the earlier
saved map, that reconstruction reproduces all 4,026 assignments and every final
exemplar and placement; two diagnostic notes differ. Resolving every frame
immediately does not reproduce it. This distinction matters to attachment tests.

```powershell
python experiments/entity_association/replay_attachment.py --before captures/2026-10-04-reader-validation/before.db --after captures/2026-10-04-reader-validation/after.db --map captures/2026-10-04-association-likelihood/map.json --recorded-schedule --background-after 7,13,16,22,25,27 --labels captures/2026-10-04-reader-validation/owner-review/first-pass.json --output .cache/attachment-replay-new/baseline
```

Repeat with `--candidate evidence`, `masked-floor` or `trusted-updates`, each in
a different output directory. These are temporary resolver hooks, outside the
deployment manifest. `evidence` requires the existing merge appearance score to
be positive over observation pairs with the entity's kept exemplars. Missing
masked/semantic vectors, backend mismatch or conflicting exemplar provenance
abstain. It uses the existing weights and zero threshold without new fitting.
`masked-floor` reuses the plain gate's 0.55 for masked resemblance; `trusted-updates`
reuses 0.70 to restrict exemplar updates. None is a production fix.

For the longer development recording, add `--attachment-gate` to
`replay_experiment.py --mode none` in each fold, with `--exclude-observation 61656
--same-person`. Its appearance weights are fitted only on objects outside the
test fold, as in the existing experiment. This tests the scoring rule under
object holdout, not the default deployed weights or independent acceptance.
Report newly added wrong pairs and lost correct pairs separately; net counts
alone can hide replacement mistakes.

## Bounded joint repair

`repair_groups.py` partitions all placed records into deterministic disjoint spatial
tiles, reconstructs observation clusters, and optionally runs the existing reader
preview over the proposed records in an empty temporary store. It reads the source
database in read-only mode and checks its hash afterward. Pending observations stay
pending; missing appearance stays unchanged; singletons become waiting proposals.
No live identities, persistent IDs or resolver rejection records are written.

```powershell
python experiments/entity_association/repair_groups.py --database captures/2026-10-04-association-likelihood/world.db --map captures/2026-10-04-association-likelihood/map.json --fold 0 --second-stage --output .cache/new-repair/fold-0.json
python experiments/entity_association/repair_groups.py --database captures/2026-10-04-association-likelihood/world.db --map captures/2026-10-04-association-likelihood/map.json --fold 1 --second-stage --output .cache/new-repair/fold-1.json
python experiments/entity_association/repair_groups.py --database captures/2026-10-04-reader-validation/after.db --map captures/2026-10-04-association-likelihood/map.json --labels captures/2026-10-04-reader-validation/owner-review/first-pass.json --second-stage --output .cache/new-repair/fresh.json
```

Choose new output paths; existing results are refused. Folds use the audited
physical-object exclusions and one-person policy. Without `--fold`, the deployed
default weights are used without fitting. Fresh scores report both clear drafts
and all tentative object drafts. Labels determine fitting/scoring, never tiling or
membership. The fresh store still includes earlier history.
Repeat `--trace-observation <id>` to record accepted joins involving selected
observations, their member IDs, appearance distribution and geometry misses.
Tracing observes the same decisions; it does not alter membership.

The fixed spatial radius is 0.75 m, with uncertain neighbours considered out to
3 m. Tiles target at most 512 observations; an oversized source record is kept
whole and reported. Sum linkage uses threshold 1.0, backend and same-picture
cannot-links, and a median geometry veto at 2.5 on the smaller joining side.
Unlike the old selected-neighbourhood bench, no-fit unions are refused. Missing
masked vectors fall back to plain vectors. The second stage refits proposed
records and can group across tile boundaries; it temporarily installs the fold's
appearance weights and always restores them. Run this standalone experiment
outside any serving process.

The [first measurement](../../docs/progress/2026-10-04-bounded-identity-repair.md)
shows substantial development improvement but confirmed fresh chair/table mistakes
excluded from the clear-draft score. The prototype is not a production repair.

## Appearance provenance and channel tests

`repair_groups.py --appearance-columns 0 1`, `1`, or `0 1 2` refits the selected
channels (plain DINO, masked DINO, SigLIP) on the original development labels.
Omitted coefficients are zero. Use `--fold 0` and `--fold 1` for the longer
recording; omit `--fold` to fit all development objects before evaluating the fresh
recording. Fresh labels never enter fitting. An explicit three-channel refit is
the control for the older default deployed coefficients. Geometry, linkage and
thresholds are unchanged. Supply a new output path for every run.

`probe_appearance.py` checks saved feature provenance and demonstrates the
difference between the actual padded appearance mask and the clipped saved outline:

```powershell
python experiments/entity_association/probe_appearance.py --database captures/2026-10-04-reader-validation/after.db --frames captures/2026-10-04-reader-validation/frames --observations 64520 64692 64581 64592 64735 --output .cache/new-appearance/provenance.json
```

`probe_clipped_appearance.py` uses the model exports named in
[the installer](../../world_state/install_perception.sh), plus a local ONNX Runtime
installation. It first re-encodes each plain crop for comparison with its recorded
vector, then blanks pixels outside the reconstructed saved outline and re-encodes
that crop. It preserves a JSON fidelity report and NPZ vectors in a new directory:

```powershell
python experiments/entity_association/probe_clipped_appearance.py --database captures/2026-10-04-reader-validation/after.db --frames captures/2026-10-04-reader-validation/frames --model .cache/appearance-probe/dinov2-small-fp16.onnx --runtime .cache/appearance-probe/runtime --observations 64520 64692 64581 64592 64735 --output .cache/new-appearance/dino
python experiments/entity_association/probe_clipped_appearance.py --database captures/2026-10-04-reader-validation/after.db --frames captures/2026-10-04-reader-validation/frames --model .cache/appearance-probe/siglip2-vision-fp16.onnx --runtime .cache/appearance-probe/runtime --kind semantic --observations 64520 64692 64581 64592 64735 --output .cache/new-appearance/semantic
```

The runtime can be installed in that private directory with `python -m pip install
--no-deps --target .cache/appearance-probe/runtime onnxruntime==1.30.0` when its
dependencies are already available. It is a workstation dependency, not a rover
runtime change. Both probes check that the source database remains byte-identical.

After encoding all observation IDs in the frozen draft, `assess_clipped_appearance.py
--probe <output-directory> --draft <first-pass.json> --output <new-score.json>`
reports pair-ranking AUC for clear and tentative drafts without fitting or choosing
a threshold. It verifies snapshot, frame and observation-ID correspondence, and
reports the re-encoded plain feature as a control alongside the stored reference.
It refuses a partial set of draft observations.

`stored_masked` in the DINO NPZ is the original masked DINO feature;
`stored_semantic` in the semantic NPZ is the original **unmasked** semantic feature.
The semantic mask channel is new and was not present in the original observations.
Do not mix the diagnostic vectors into live histories or reuse existing fitted
weights with a changed channel. Saved outlines have half resolution, rounded boxes
and no information about masked pixels outside the detection box. CPU/plain-crop
agreement measures part of that uncertainty; it does not remove it.

The [measurement](../../docs/progress/2026-10-04-appearance-provenance.md) retains
masked semantic evidence for a further development experiment. It does not approve
any channel replacement or identity change on the rover.

## Fitted masked-semantic repair

New semantic probe outputs contain per-observation fingerprints of the frame,
box, mask, original semantic vector and backend. `semantic_features.py` checks
those fingerprints before a bank can be used with either saved database. It
normalizes the new vectors and reports absent observations explicitly. The older
diagnostic outputs without fingerprints cannot be substituted silently.

Pass `--semantic-probe <probe-directory>` to `repair_groups.py` to fit and test
the fourth channel. Column 3 is the new masked semantic similarity. A matched
control uses `--appearance-columns 0 1 2`; the candidate uses `0 1 2 3`.
Both use the bank's coverage and the same original labels. Fit each development
fold with `--fold 0` or `--fold 1`; omit `--fold` for a full development fit followed
by fresh evaluation, never a fit to the fresh labels.

```powershell
python experiments/entity_association/repair_groups.py --database captures/2026-10-04-association-likelihood/world.db --map captures/2026-10-04-association-likelihood/map.json --fold 0 --second-stage --semantic-probe .cache/masked-semantic/full-probe --appearance-columns 0 1 2 3 --output .cache/new-semantic/candidate-0.json
```

Missing-feature observations keep their original owners. Reader grouping refuses
comparisons lacking the additional sampled features, in both arms. The temporary
reader hook preserves the original sampling, backend comparisons and three-channel
arithmetic, adds the fitted fourth term, and restores the original functions even
after failure. It supports negative fitted coefficients as well as positive ones.
Run the experiment standalone, never inside a serving process.

The probe's `--threads` controls workstation CPU threads. Frame retrieval and
encoding artifacts for the full trial are preserved privately under
`captures/2026-10-04-reader-validation/semantic-development/`. The
[full comparison](../../docs/progress/2026-10-05-masked-semantic-repair.md) rejects
this fitted candidate for deployment; the smaller favourable diagnostic is not
an acceptance result.

## Split-only and release-only proposals

`repair_groups.py --split-only` analyses each original placed record separately,
retaining the existing fit and linkage rules. `--release-only` uses the same
analysis but only sends singleton observations back to pending; all other original
memberships remain. Both forbid `--second-stage`. A checked invariant prevents
cross-record joins, changes to originally pending observations, and incomplete
coverage. This guarantee applies to the proposal, not subsequent resolver passes.

Use `--fold 0` and `--fold 1` on the older snapshot. For the fresh snapshot use
`--appearance-columns 0 1 2 --labels <first-pass.json>` to fit the older development
labels before scoring the frozen fresh draft. Use a new output path each time.

`replay_release.py` backs up the source into a temporary store, releases the
proposal's singleton IDs through the existing experimental detach path, and runs
three ordinary idle resolver passes. It supports `baseline`, `released` and
`guarded`; the last refuses each released observation's original parent using a
temporary in-memory hook. It neither applies multi-observation splits nor implements
durable rejection memory. Never use this as a live repair command.

```powershell
python experiments/entity_association/repair_groups.py --database captures/2026-10-04-reader-validation/after.db --map captures/2026-10-04-association-likelihood/map.json --labels captures/2026-10-04-reader-validation/owner-review/first-pass.json --appearance-columns 0 1 2 --release-only --output .cache/new-release/proposal.json
python experiments/entity_association/replay_release.py --database captures/2026-10-04-reader-validation/after.db --proposal .cache/new-release/proposal.json --map captures/2026-10-04-association-likelihood/map.json --draft captures/2026-10-04-reader-validation/owner-review/first-pass.json --mode guarded --output .cache/new-release/guarded.json
```

The lifecycle output's `returned_to_original` lists released-candidate IDs currently
in their original parent; at `original` and in the untouched baseline they have
never left, so those are not reattachment counts. Scores retain the frozen labels.
The [measurement](../../docs/progress/2026-10-05-release-only-repair.md) preserves a
post-score visual dispute separately and retains release-only as a candidate for
review, not an approved automatic repair.

## Reviewing the release proposals by eye

`release_audit.py sheets` draws every release of a proposal beside up to eight members
its original record keeps: the release's full frame, selected pixels and plain box,
one block per record. `release_audit.py summarize` checks a review covers every
release once, against the proposal's own parents, and counts it. The
[review of the fresh proposal](release_audit.json) was frozen before any label was
consulted; its verdicts are the coding agent's and the owner has not reviewed them.
Never edit it to agree with a later label; record disagreements in a progress entry.

```powershell
python experiments/entity_association/release_audit.py sheets --proposal captures/2026-10-04-reader-validation/split-only/release-fresh.json --database captures/2026-10-04-reader-validation/after.db --frames captures/2026-10-04-reader-validation/semantic-development/frames captures/2026-10-04-reader-validation/frames --output .cache/new-release-audit
python experiments/entity_association/release_audit.py summarize --review experiments/entity_association/release_audit.json --proposal captures/2026-10-04-reader-validation/split-only/release-fresh.json
```

The [result](../../docs/progress/2026-10-05-release-audit.md): wrong releases are about
as common as right ones, so release-only stays a candidate list, not a repair.

`release_position.py` measures each release against a placement refitted from its
record's largest kept cluster and gates on the repair's existing 2.5 veto. Its
`PREDICATE` was committed before any score was computed; scores are against the
frozen review and, for both older folds, the older look labels.

```powershell
python experiments/entity_association/release_position.py --fresh captures/2026-10-04-reader-validation/split-only/release-fresh.json --older captures/2026-10-04-reader-validation/split-only/release-0.json captures/2026-10-04-reader-validation/split-only/release-1.json --database captures/2026-10-04-reader-validation/after.db --older-database captures/2026-10-04-association-likelihood/world.db --map captures/2026-10-04-association-likelihood/map.json --review experiments/entity_association/release_audit.json --labels captures/2026-10-03-merge-review/look_labels.json --output .cache/new-release-position.json
```

It [fails](../../docs/progress/2026-10-05-release-position.md) on all three.

`release_category.py` scores each observation's stored SigLIP2 vector against
`PHRASES`, keeps a release whose best phrase matches its record's, and is judged on
the same terms. `request` writes the phrase list; the vectors must come from the
rover's perception sidecar, the backend every stored vector came from:

```powershell
python experiments/entity_association/release_category.py request --output phrases-request.json
ssh orin "curl -s -X POST -H 'Content-Type: application/json' --data-binary @- 127.0.0.1:8776/embed" < phrases-request.json > phrases-response.json
```

Store the response with its phrases as the `--phrases` file for `test`, which takes
the same arguments as `release_position.py` with `--phrases` for `--map`. It
[fails](../../docs/progress/2026-10-05-release-category.md) on all three.
