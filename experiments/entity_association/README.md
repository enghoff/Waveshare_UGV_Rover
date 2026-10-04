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
