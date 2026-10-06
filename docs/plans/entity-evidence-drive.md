# Independent visibility recording

R-WS-13 remains open; R-WS-17 and R-WS-18 remain proposed. Keep the proposed 5%
contamination target and the narrower `--new-visibility-only` candidate's existing
thresholds. Preserve all earlier failures, including the mixed-crop score.
The [targeted-look audit](../progress/2026-10-06-targeted-look-provenance.md)
adds a real painting-to-duplicate example but is not independent acceptance truth.

## Blocker before handover

Stationary readiness found 17.5 degrees disagreement between reported heading
and a trusted fresh scan fit. The unchanged limit is 10 degrees. No motion was
requested. Do not ask for an untethered rover until trustworthy starting
localization can pass this check. Do not force-fit or relax the check to get a
pass. Independent confirmation/correction of the starting pose is still needed;
software preparation and route clearance do not establish it.

## Prepared execution

The committed diagnostic bundle is `/tmp/visibility-trial-9de3802` on Orin,
hash-verified against source. It is outside the deploy tree and may disappear on
reboot; restage the nine files listed in the experiment README if necessary.
Source and recorder checks are complete; successful motion remains unproven.

After localization is resolved and a fresh charger-disconnected handover, use:

```bash
bash /tmp/visibility-trial-9de3802/run_prepared_trial.sh \
  --session visibility-independent-20261006-2 --execute --motion-authorized
```

A unique session and both flags are required. Obtain handover only when ready to
execute immediately. Recheck actual HOME, map ID `7da19bef3888`, at least 60%
charge, fresh changing IMU feedback, trusted scan agreement of at least 90%
within 0.25 m/10 degrees, and complete active world/navigation/board recording.

B is (-19.050, -14.850), C is (-20.000, -14.200). Both arrival headings are
55 degrees. Predicted painting bearings are 82.67 and 67.12 degrees: offsets of
27.67 and 12.12 degrees remain within the nominal forward depth field, but actual
outlined-region depth and visibility must decide coverage. The gimbal must remain
centred as in the recorded preparation; no ad hoc chassis-facing correction is
allowed. The B/C baseline still predicts about 15.5 degrees parallax.

Use navigation directly, including its normal turning, rather than a separate
preliminary `turn_in_place`. Validate HOME-B, B-C, B-HOME and C-HOME against actual
start/arrival headings. Reject occupied/unknown body space or paths longer than
direct distance plus 0.5 m. Check current paths and costmaps again before each leg.
The return is directly to HOME from either completed viewpoint, not through B.
The old C-B return fails the route limit; the new direct returns passed stationary
planning. A changed HOME may invalidate those proofs, so handover repeats them.

Before and after motion, require measured speed/rotation/PWM zero continuously
for 0.4 seconds, with healthy board/fresh transforms, within three seconds.
Command completion alone is insufficient. Check fresh localization at arrival.
If the requested observation heading is missed by more than five degrees, stop;
do not issue an extra trial turn. An incomplete move or failed fresh check stops
for recovery instead of pretending the goal was reached.

The serial run reserves 15 seconds for return preparation, admits translations
with a 16-second watchdog and retained-depth looks with 12 seconds available.
Looks record without synchronous identity settling; the normal world-state
clock handles settling, and the call recorder retains the actual passes.
Return starts as soon as collection ends or no next action fits. If no return
motor command begins by 60 seconds, STOP for recovery. At 35% charge return;
at 15% stop for manual recovery. External STOP ends this session's control.

Verify physical rest, close the recorders and tell the owner movement is finished
so charging can resume. Report map-estimated return error and heading difference;
this is not docking precision. Copy and hash-verify evidence before saying the
rover may be powered off. No production perception/navigation change is part of
this trial.

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
controls. The newer rotation-feedback guard is documented in
[the October 6 measurement](../progress/2026-10-06-a-rover-that-cannot-feel-itself-turn.md);
respect it during handover. A successful power cycle alone does not establish
that fault is gone.
