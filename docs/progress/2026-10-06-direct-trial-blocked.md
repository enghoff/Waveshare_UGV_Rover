# Direct routes are prepared; starting heading prevents execution

The next entity trial is blocked before motion by a trusted scan check finding
17.5 degrees heading disagreement, beyond the unchanged 10-degree limit.
No movement was requested and the owner was not asked to disconnect the charger.
R-WS-13 stays open; R-WS-17 and R-WS-18 stay proposed. No production component
was changed or restarted.

## Route preparation

The separate initial turn in the
[failed trial](2026-10-06-trial-turn-stop.md) exposed a known post-turn heading
discrepancy before navigation could act. The revised experiment asks navigation
to handle each complete travel leg. Checks at admission and arrival remain.
It does not claim to repair the underlying heading fault.

Read-only live planning reproduced a remaining route failure: C-to-B from the
old viewing heading produces 3.407 m for a 1.151 m displacement. Direct C-to-HOME
passes, but direct B-to-HOME initially fails. Choosing 55-degree arrival headings
at both views admits both direct returns without lowering the route limit.

| Route | Planned length |
|---|---:|
| Actual HOME (-17.125, -15.824, 142.5 degrees) to B | 2.538 m |
| B to C | 1.157 m |
| C directly to HOME | 3.763 m |
| B directly to HOME | 2.524 m |

All four paths passed complete footprint and direct-distance-plus-0.5-m checks
against the captured costmap. The return starts at the actual planned observation
heading, not a hypothetical already-return-facing pose. The predicted painting
offsets from the new headings are 27.67 and 12.12 degrees; actual depth/outlined
coverage remains unproven. These are planning results, not executed wheel paths.

The serial policy returns directly from either completed viewpoint, including
early collection termination. A failed/partial leg still stops for recovery.
Fresh arrival localization remains mandatory. Missing the observation heading
by more than five degrees causes refusal, not another trial turn.

## Stationary host checks

Source b9454a9 was staged and hash-verified as nine diagnostic files outside the
deploy tree. The complete stationary preparation activated both recorders and
made a fresh retained-depth observation: 12 stored regions, six ranges, zero
reported motion. The look took 8.64 seconds including waiting, longer than the
trial's old eight-second allowance. Final source 9de3802 reserves 12 seconds
and uses recording-only looks, leaving settling to the normal world-state clock.

The fresh localization admission failed at 17.5 degrees. Preparation closed both
recorders normally, retaining 152 poses, 29 costmaps and zero velocity commands.
Seven world/archive files and the support bundle were copied and hash-verified
under `captures/visibility-direct-readiness-b9454a9/`. The
[measurement](2026-10-06-direct-trial-blocked.json) retains hashes and exact results.

Twenty-five local checks pass, including recorded route rejection, both direct
returns, early collection end, no extra chassis turn, post-arrival admission,
the observed look-duration allowance and existing recorder/watchdog checks.
All nine final files at `/tmp/visibility-trial-9de3802` were hash-verified against
their commit. Calling its inspection entry point on Orin refused at the same
fresh localization guard before a look. Successful post-motion execution and
independent entity acceptance remain unproven. The successful 12-second
recording-only world call was the stationary warmup; no changed service needed
deployment or restart.

## Blocker

The navigator reports 142.5 degrees; the scan fits near 160 degrees with score
0.988, versus 0.603 at the reported pose. The 6 cm position difference passes
the distance limit. Software preparation cannot establish which heading is
physically correct. Starting localization needs independent confirmation or an
operator correction before a fresh readiness check and immediate untethered run.
Do not force-fit the map or loosen the limit merely to obtain a pass. The
[plan](../plans/entity-evidence-drive.md) contains the prepared command, admission,
recording, return and failure policy; no further motion was requested here.
