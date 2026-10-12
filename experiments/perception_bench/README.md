# Perception evidence benchmark

This is a frozen catalogue of **340 cases**, with ground truth, historical
baseline measurements and raw-evidence locations. It supports R-WS-10,
R-WS-13, R-WS-17 and R-WS-18. It does **not** claim a fresh replay of the current
deployed perception stack. Candidate models must first reproduce that baseline
on the same evidence before their results can justify a rover change.

The [M5 identity audit](shakedown_audit.md) is already measured: the red bean bag
split across its move, and record 838 holds both armchairs even after refitting.
The individual visual labels and resolver notes are in
[shakedown_audit.json](shakedown_audit.json). The two failures are explicit
regression cases in [regression_cases.json](regression_cases.json), with the
required outcomes and bad-pose exclusions. Their live memberships are measured;
a faithful baseline replay is still required before testing a remedy.

## What is included

| Cases | Count | Ground truth and scope |
|---|---:|---|
| Find by words | 171 | 120 frozen phrases: 60 real, 60 absent; 40 positive anchors from the record review; 11 taped M4 targets with known positions and frozen matching record lists |
| Where things are | 17 | 11 owner-taped October 9 targets and six October 3 development targets; 35 scored M4 attempts with before/after placement errors, plus the original October 3 placements |
| One object, several records | 13 | Ten M4 multi-record target lists and three per-look duplicate groups from the merge review; lists are scoped to their original label snapshot |
| Per-look identity | 25 | The October 3 review's 25 records, with main, odd and unsure observation IDs; only the 19 formerly uncertain looks were reviewed by the owner |
| Merge-pair review | 18 | Fourteen analyst judgments of one subject in sampled crops, and four ambiguous/mixed pairs, transcribed from the original October 4 sheets |
| What changed | 10 | Five shakedown change events and five scene-1 cases; scene 1 retains its corrected staging interval and unresolved positions |
| Unchanged controls | 7 | The October 11 recorded runs, with closed-door exceptions, the control-3 false alarm and control 6 marked too short for acceptance |
| Record quality review | 79 | 40 object, 18 mixed, 13 non-object, four person and four unclear records |

[manifest.json](manifest.json) lists each case, its source files and hashes,
ground truth, observation IDs, raw frame references, recorded pose/range coverage,
retained depth coverage, ROS bag locations, and scoring limits.
[frame_index.json](frame_index.json) resolves **4,349 snapshot-scoped frame
references**, representing **4,062 distinct pictures**. All their current-world
JPEG paths existed on the rover when checked; existence is not a decode test.
Stored metadata says 640 by 480 for every referenced picture. These legacy
labelled frames predate full-resolution retention and **none has a `.full.jpg`**.
Since the deployment at approximately 12:05 IST on October 12, still looks retain
a separate 2592 by 1944 view with `full_width`, `full_height`, `full_bytes` and
`full_taken_at` (R-WS-19). Nine such files were verified in a later read-only
inventory; they are newer unlabelled evidence, outside this frozen catalogue.
Each frame and item now reports full-image availability separately from the
640 by 480 perception frame. A paired resolution benchmark still needs labels
for those new pictures.

The frozen truth files under `ground_truth/` keep the ignored local measurement
sources from becoming the only copy of their labels. The manifest also retains
the original sources and hashes. Source databases and photographs remain in
ignored `captures/` or on the rover; this commit does not duplicate large images
or databases into Git.

## Evidence and limitations

The live store was copied **through SQLite backup**, then read without changing
the original. The full consistent world backup is locally available at
`tmp/codex-m5-audit/world.db` and on the rover at
`/tmp/codex-m5-audit-world.db`. Selected photographs and metadata are beside the
local backup. The full episode-store transfer failed; its complete consistent
backup remains at `/tmp/codex-m5-audit-episodes.db`, and all selected audit-window
events were successfully exported to `tmp/codex-m5-audit/events.json.gz`.

Every identifier is qualified by a snapshot. **Even the same generation token
does not establish that an entity ID survived a rebuild.** Use labelled
observation membership to match rebuilt results, rather than assuming that a
record number still names the same physical object. October 3's development
targets are separate from October 9's held-out acceptance targets.

The 60 real benchmark phrases mostly establish presence, rather than an exhaustive
set of correct object records. Report presence/refusal on that set; do not call it
top-1 identity accuracy. Positive record anchors also tolerate stray looks and
are not pure per-look labels. A result returning a different clean duplicate may
be correct. Identical dining chairs have kind labels, not individual identities.

Observation **61656** remains disputed and is excluded from duplicate scoring;
the original label is preserved. October 4's merge judgments describe sampled
crops, not the purity of every look in a record. Owner confirmation is not inferred
from authorization to do this work.

Raw depth exists for only some pictures; a stored numeric range is not a retained
depth frame. Each item says which are present. Missing depth cannot be recreated
from the JPEG. One selected historical file, `20261010-135451-7f696d.jpg`, is empty
and fails image decoding; it is listed in the manifest, not counted as usable
visual evidence. All other locally copied audit pictures passed decoding checks.

Scene 1's first move occurred after 13:17 and before 14:13, with an unrecorded gap.
The replay's 13:45 move is an assumption, not the measured event time. Some scene
positions and individual-chair correspondence remain unknown. In the shakedown,
08:00:58 until the approximately 08:11 refit is a bad-pose window. Preserve those
pictures as fault evidence; exclude their poses from normal placement scoring.

The current deployment revisions are recorded in the manifest. Historical M4
before/after scores describe the recorded trials and their scoring replay; they
are not a fresh rerun of those observations through today's deployed stack.
Most raw images are referenced rather than locally bundled, so full end-to-end
offline re-perception requires materializing the listed images/depth and freezing
the matching model/backend. The catalogue itself can be inspected offline now.

## Rebuilding and checking

The builders operate only on copied data and local files:

```powershell
python experiments/perception_bench/audit_shakedown.py tmp/codex-m5-audit/metadata.json.gz --output experiments/perception_bench/shakedown_audit.json
python experiments/perception_bench/build_manifest.py --metadata tmp/codex-m5-audit/metadata.json.gz --inventory tmp/codex-m5-audit/remote-inventory.json --full-inventory tmp/codex-m5-audit/full-image-inventory.json
```

`build_manifest.py` also needs the two frozen local databases under
`captures/2026-10-03-targets/` and `captures/2026-10-03-merge-review/`, NumPy for
the one-time tape-frame extraction, and Pillow for local picture checks. The
committed tape coordinates have already been extracted; the builder itself needs
Pillow, not NumPy. It makes no network calls or rover writes.

Validation checked every referenced ground-truth observation against its scoped
database, every local source path and hash, every M4 episode snapshot reference,
all locally copied JPEGs, and the repository's requirement identifiers and links.
The next measurement is a faithful replay of the deployed baseline against these
cases, with candidate and baseline using identical inputs and identical exclusions.
