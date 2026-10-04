# A short fresh drive is ready for independent identity labels

The owner authorised driving after the grouping preview was deployed. The rover
made a short trial, returned near its starting point and was explicitly stopped.
It added 232 regions in 27 full frames: 204 have bearings and 82 have ranges.
None has been used for training, tuning or candidate scoring. R-WS-13 remains
open; R-WS-17 and R-WS-18 remain proposed. Independent labels and agreed
tolerances are still needed for acceptance.

## Movement and limitations

The camera and map showed clear floor ahead. The latest local position check was
trusted within 4.5 cm and 0.5 degrees of the saved pose; the older global drift
check remained ambiguous. No map reset or pose refit was performed.

The rover reported 0.704 m forward, then 0.606 m after a left turn. The requested
45-degree turn reported 64.6 degrees, so the outward trial was shortened. A
planned return to the original measured map point reported another 1.085 m.
The final position was reported 8.5 cm from the start, at heading 117.2 degrees
against 102.9 initially. These are rover estimates, not taped measurements.
The final camera image showed the original room view. Explicit stop was followed
by `driving: false` and zero speed. The requested heading was not restored
exactly; no navigation fix is claimed without a controlled reproduction.

The first stationary sweep tried pans beyond the calibrated 20-degree envelope;
subsequent sweeps used -15, 0 and +15 degrees. All 28 regions without bearings
remain in the recording and review pack. All new regions use TensorRT and have
valid wall-clock timestamps. No unavailable evidence was silently dropped.

## Frozen evidence

`captures/2026-10-04-reader-validation/` holds consistent `before.db` and
`after.db` snapshots, all 27 frames, `calls.jsonl`, final navigation status and
camera image. Transfers were checked against remote SHA-256 digests. The
[compact record](2026-10-04-reader-validation-drive.json) fixes those hashes,
the observation cutoff, counts and deployed commit.

`navigation.json` contains 1,800 pose samples and 359 costmaps over 180 seconds.
It ended before the return and contains no plan or `cmd_vel_nav` samples: the
selected topics did not capture the direct drive/spin commands. It is partial
navigation evidence, not a validated controller replay.

`owner-review/review.html` shows numbered regions and `owner-review/labels.csv`
has 232 blank rows. Current entity assignments, old labels and candidate results
are hidden. Pending, mixed and uncalibrated regions remain present. Unclear
chair identities and surfaces must stay explicit rather than acquire invented
physical identities. Head/body views of one person remain the working policy.

The preparation tool now accepts a separate read-only snapshot, saved frames
and an observation cutoff. Nine evaluation/review tests pass, including a
blinding check that covers pending regions and missing images. Documentation
checks pass. Only workstation tooling and documents changed; no additional
component deployment is required. Changes are on `entity-reader-groups` and
the earlier main-checkout edits remain intact.

Next: an independent reviewer fixes physical-object identities before scoring.
Establish whether this shortened recording covers the planned minimum of 20
distinguishable objects, then compare baseline and grouping with weights frozen
before the drive. Report missing geometry and pending regions separately.
Further coverage and controlled reproduction of the turn may need another run.
