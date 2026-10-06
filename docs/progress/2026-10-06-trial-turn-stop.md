# The independent entity trial stopped after its first turn

R-WS-13 remains open; R-WS-17 and R-WS-18 remain proposed. Session
`visibility-independent-20261006-1` failed coverage before translation, so it
provides no acceptance score for the entity candidate. Production perception
and navigation were unchanged. The 5% contamination target remains proposed.

The trial, source 6c363d9, requested -166.23 degrees. After about 1.75 seconds
the turn replied `arrived`, reporting -171.4 degrees. Driving was false and PWM
zero, but measured odometry still reported -112.6 degrees/second. The runner
mistakenly treated command completion as rest. Its fresh scan check exceeded
the unchanged 10-degree heading limit and stopped execution. No translation
command was issued and no waypoint was reached.

The immediate STOP verification also saw residual rotation (-116.5 degrees/second)
and did not claim success. A later STOP/status check confirmed zero speed,
rotation and PWM. Map position changed about 8 mm to (-17.645, -15.610), while
heading changed from -42.3 to 131.8 degrees. The owner was released to reconnect
the charger at the starting location, facing the other way. This is a map estimate,
not independent docking measurement.

Both recorders closed. Navigation retained 232 poses, 45 costmaps and four plans;
the passive board record has 1,157 rows. Zero Nav2 velocity commands does not mean
zero movement: the direct turn is in the RPC and board logs. Six support files
and eight world/archive files were copied and hash-verified under
`captures/visibility-independent-20261006-1/`.

Source 48228bd reproduces the old admission against recorded replies, then
requires measured linear/angular velocity and PWM to remain zero for 0.4 seconds,
with a healthy board and fresh transforms, within three seconds. It applies
before localization, after motion and after STOP. Twenty checks passed. Test
rest windows appended to the recorded moving replies are synthetic, not a
measured stop-duration trace. These are trial limits, not stopping calibration.

All nine diagnostic sources staged at `/tmp/visibility-trial-48228bd` were
hash-verified. The new wait accepted real stationary TCP 8769 replies after
0.428 seconds. This verifies stationary operation; post-motion operation remains
unproven. No service restart was required for these non-deployed helpers.

A distinct heading disagreement remained after physical rest: a later read-only
scan fit found 12.5 degrees and 10.2 cm difference, score 0.989 versus 0.557 at
the reported pose. Waiting cannot fix this. It resembles the independently
measured [October 1 fault](2026-10-01-heading-after-turning.md), but supplies no
independent angular truth or root-cause diagnosis. A subsequent stationary
health check passed at a different reported pose and control sequence; that is
not evidence that this change repaired heading. The owner did not recall
intervening movement; its cause remains unresolved and is not an entity result.

Do not repeat this trial unchanged or raise its heading limit. Review the newer
autonomy recordings for targeted-look evidence before spending another battery
on collection. If more movement remains necessary, finish the actual-heading
route and return preparation first. The previous
[in-move heading correction was reverted](2026-10-01-heading-check-rolled-back.md)
because it blocked commands; do not revive it as a trial workaround.
