# Independent visibility recording

R-WS-13 remains open; R-WS-17 and R-WS-18 remain proposed. Keep the proposed 5%
contamination target. Evaluate the narrower `replay_visibility_rival_veto.py
--new-visibility-only` candidate with its existing thresholds. Preserve the wider
failures and old mixed-crop identity-score failure in the
[October 6 measurement](../progress/2026-10-06-side-painting-recording.md).

## Handover and execution

Request the rover only when ready to run. Obtain explicit confirmation that the
charger is disconnected and this session controls the rover. No chassis movement
is authorized by this plan. Keep production perception/navigation unchanged.

Use the committed diagnostic helpers described in the
[experiment README](../../experiments/entity_association/README.md), staged together
outside the deploy tree. The prepared launcher defaults to stationary verification;
only `--execute --motion-authorized` enables motion. Use a fresh session name, for
example `visibility-independent-20261006-1`; existing outputs and recording markers
must be refused. Do not reuse the old per-command scratch caller.

```bash
bash /tmp/visibility-trial-COMMIT/run_prepared_trial.sh \
  --session visibility-independent-20261006-1 --execute --motion-authorized
```

Replace COMMIT with the staged and hash-verified source commit. Before moving,
the launcher must verify live navigation, stationary motor output, at least 60%
charge, changing fresh IMU feedback, trusted fresh scan agreement of at least
90% within 0.25 m/10 degrees, active navigation/board recording and an active
world-call recording following a fresh stationary depth look. Never reset or
force-fit maps to make preparation pass.

Use `visibility_trial_card.json` only while map ID `7da19bef3888` still applies.
Save the actual current pose as HOME. The two candidate observation positions
are B (-19.050, -14.850) and C (-20.000, -14.200), with view headings 82.67 and
67.12 degrees. Recheck HOME-B, B-C, C-B and B-HOME against current live planner
paths and costmaps. Later starts are hypothetical and must face along travel;
the executor separately checks its actual turn and route at each leg. Reject
unknown/occupied body space or a route longer than direct distance plus 0.5 m.
The apparently obvious farther-west extension (-20.150, -14.850) is not an
alternative: it intersects the saved obstacle map. Do not improvise another route.

Collection and return run serially in one process, without conversational waits.
Reserve 15 seconds for return preparation. A translation has a 16-second STOP
watchdog and a turn an 8-second watchdog; admit each only with its allowance
remaining. Begin checked return as soon as collection ends or no further action
fits. If the first return motor command has not started by 60 seconds, STOP for
recovery instead of making a late unverified move. A fresh check failure, partial
move, external STOP or watchdog ends automatic execution; do not treat an
unreached waypoint as reached. At 35% charge, end collection and return; at 15%,
STOP for manual recovery. These are operational limits, not battery calibration.

Return through successfully reached points in reverse order to HOME. Restore
neither old map coordinates nor an assumed heading. Verify STOP and zero motor
output, then acknowledge recorder closure and preserve its data. Tell the owner
when movement is finished so charging can resume. Report map-estimated return
error and heading difference plainly; do not claim measured docking precision.

## Evidence and decision

Copy and hash-verify the complete navigation/board recordings, world-call before
and after stores, event/map files, photographs and retained raw depth before
saying it is safe to power off. Confirm the world manifest and navigation
checkpoint both acknowledge closure. A periodic open checkpoint protects some
data after a fault but does not prove the final STOP was recorded.

Obtain two clear views of one distinguishable background painting, at least
0.4 m apart with 12 degrees measured bearing separation, plus an occluded view.
The prepared B/C positions predict about 15.5 degrees; actual images and bearings
decide whether coverage passed. Retain foreground chair regions separately, all
missed detections, valid parts, mixed masks and missing depth. Do not lower the
coverage threshold if a time-bounded run supplies less than intended.

Review full photographs and stored outlined pixels with assignments/candidate
results hidden. Freeze physical labels and the final candidate source hash before
scoring. Distinguish pure object, valid part, mixed, non-object and unresolved
individual identity. Previous development refinements and corrected historical
labels cannot become independent acceptance truth by being scored again.

First reproduce every actual original checkpoint and ordered map query, and all
available retained depth answers. The candidate extends mapped reach to valid
stored depth only during existing-record matching; discovery/refitting and other
gates stay unchanged. In frame assignment, apply the existing 0.15 global rival
veto additionally only where the original unextended ray forbids that candidate.
Reject candidate queries whose answers differ across saved pass grids, including
failures swallowed by production fallback handling.

Retain at least 98% of existing useful connections, create no reviewed clean
cross-object connection, and gain a genuine clear/occluded painting connection.
Report all unreviewed changes, failed coverage and missing evidence. Do not call
a small successful subset deployment acceptance or merge old records on its
strength. Preserve all earlier frozen scores, including mixed-crop failures.

Pure-table/glass observations with independently measured distance, partial/whole
armchair representation and duplicate historical placements remain separate owed
controls. A frozen-rotation-feedback guard also remains unimplemented; it needs
reproduction against failed and healthy moving/stationary recordings before a
control change. A successful power cycle does not establish that fault is gone.
