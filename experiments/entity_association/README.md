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
