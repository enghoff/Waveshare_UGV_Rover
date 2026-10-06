# The trial is prepared; recording closure is proved on the stationary rover

R-WS-13 remains open; R-WS-17 and R-WS-18 remain proposed. This work prepares
the independent recording required by the narrower visibility candidate; it
does not deploy that candidate or change navigation/perception services.

## Reproduced failures and local checks

Executing the original `nav_record.py` main loop with an interrupted spin and a
real saved navigation episode produces no output file, reproducing the previous
loss on interruption. The new diagnostic recorder retains that same episode on
interruption and explicit close, checkpoints atomically before collection and
every five seconds, and labels error/unfinished checkpoints as open. A forced
exit cannot claim final-STOP coverage merely because a file exists.

The previous real command log starts return at 78.5 seconds. A serial policy
using its actual command durations proceeds directly into return, without the
inter-command conversational gaps. That model deliberately excludes the time of
live checks and is not a hardware timing prediction. The actual runner reserves
15 seconds for return preparation, admits bounded actions serially and requests
STOP if no return motor command begins by 60 seconds. It never issues a late
unchecked return. Turns have an 8-second STOP watchdog, translations 16 seconds.
A partial/failed move stops for recovery instead of assuming its goal was reached.
At 35% charge collection ends; 15% is the manual-recovery floor. External STOP
changes control ownership and prevents further commands from this session.

Fifteen local tests pass, including the reproduced loss, explicit/interrupted
closure, error checkpoints, timed return, partial movement, exhausted budget,
watchdog refusal, battery transitions and recorded-grid path checks. These tests
do not substitute for an untethered execution of the runner.

## Stationary hardware verification

Committed diagnostic sources were staged outside `~/ugv`, in temporary directories
named for their commits. No deployed source was edited and no service restarted.
The final nine files at `/tmp/visibility-trial-6c363d9` were hash-verified against
commit 6c363d9. The default launcher performs preflight only; both explicit motion
flags are required after an owner handover.

Three stationary checks were retained. The first proves explicit recorder close.
The second closes the recorder with SIGINT, reproducing the real interruption
mechanism on Orin: it exits successfully and preserves 194 poses, 37 costmaps and
four plans. The final source's normal close preserves 192 poses, 37 costmaps and
four plans. Both keep zero navigation velocity commands, report no board recording
error, and acknowledge closure. The recorder implementation is identical in those
last two source bundles. The last two checks also force a fresh stationary depth
look, verify the world recorder becomes active, then close its complete manifest.
The affected world function answered over TCP 8769 with zero movement/rotation.

The final preflight verifies trusted fresh localization, changing raw IMU samples,
fresh recording, idle motors and 85% charge. HOME is (-17.641, -15.616). All four
live hypothetical travel-facing paths pass the direct-distance-plus-0.5-m limit
and complete footprint checks: HOME-B/B-HOME about 1.604 m and B-C/C-B about
1.151 m. Actual body turns and navigation remain for the untethered run; hypothetical
path checks are not proof that those motions have happened.

The new C (-20.000, -14.200), paired with B (-19.050, -14.850), predicts about
15.5 degrees of painting bearing separation. The saved-grid check rejects the
obvious farther-west alternative (-20.150, -14.850) because its body intersects
occupied floor. Actual photographs, outlined pixels and measured bearings must
still establish the required clear/occluded coverage; predictions do not waive
the frozen 12-degree minimum.

Support files from all three checks are copied and hash-verified locally under
`captures/visibility-readiness-COMMIT/`. The corresponding world recordings remain
on the rover; they are stationary readiness evidence, not the independent moving
acceptance recording. The [measurement](2026-10-06-serial-trial-readiness.json)
retains source/evidence hashes, exact counts and route lengths.

## Decision

Preparation is complete and the next action needs the rover untethered. Obtain a
fresh charger-disconnected handover, then launch the committed serial runner with
a fresh session. It repeats current preflight before any movement and returns
through completed waypoints without waiting for conversation. All perception
criteria and prior failed scores remain unchanged. Runtime motion timing and
independent entity acceptance are explicitly unproven until that run. The
[plan](../plans/entity-evidence-drive.md) describes the remaining work.
