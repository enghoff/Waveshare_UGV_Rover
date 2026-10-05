# Resolve visibility admission with a clear and occluded painting recording

R-WS-13 remains open; R-WS-17 and R-WS-18 remain proposed. Keep the proposed 5%
contamination target. Development evidence does not settle acceptance.

The [completed subject diagnosis](../progress/2026-10-05-subject-fragmentation.md)
finds a real background-painting visibility rejection. Broad measured visibility
recovers a connection but loses reviewed floor-container connections. Limiting it
to existing-record matching removes that regression and retains 99.2% of the older
October 1 proxy links, but a new scored cross-target connection involves a mixed
painting/toolbox/chair crop. Preserve both failed/provisional outcomes. Do not
activate a blanket range gate, extend production visibility, merge old records or
relax the fixed criteria on the strength of corrected analyst labels.

## Next short recording

Use the unchanged production resolver and verified optional call recorder. Obtain
one distinguishable background painting from two clear positions with at least
0.4 m baseline and 12 degrees measured bearing parallax, plus an intermediate
occluded view. Keep the foreground chair/toolbox as a separately recorded region.
Retain all failed detections, valid parts, mixed masks and missing depth. The
second short drive lacked an unoccluded side painting region; another photograph
of the same occlusion does not fulfil that gap.

The [night attempt](../progress/2026-10-05-night-visibility-run.md) reached the first
three waypoints but not the proposed side extension. Do not repeat that extension
with the same heading request merely because the global grid shows clearance.
The saved costmap/lattice diagnostic suggests facing along the travel segment
before requesting the side goal, then framing the painting after arrival. It
predicts a shorter path but does not reproduce the live planner exactly. The next
live precheck must verify the actual route, localization and current clearance.
Any refusal or uncertain localization ends the attempt; do not reset or refit maps
to make a route pass, or improvise a longer route during the evidence trial.

Use the local `drive_trial_guard.py` admission checks and a new capture directory.
Wait for every movement command to finish before a fresh inspection. Reserve the
motion deadline or inspection allowance within the 60-second window and begin
return when no further action fits. Explicit return legs are exempt from the
outbound cutoff. Keep a recording window long enough for the return and release
recorders only after STOP; the preceding support recording missed the final leg.

Save the exact current starting pose, raw board feedback and controller tick
recording before movement. Capture fresh raw depth and actual matching calls
at the first waypoint, intermediate position and side position, with camera
framing directed at the painting. Do not spend the battery collecting unrelated
armchair views. Start return by 60 seconds after first movement or when reported
battery reaches 35%, whichever comes first; inspect only after prechecks and
recording are ready. Start only with at least 60% reported charge and fresh normal
rotation feedback. These are operational limits, not calibrated battery capacity.
The last drive's reported charge fell from 45% to 10% in about two minutes.

Motion, including chassis turns, requires a fresh prompt and owner permission;
the charger may be attached. Request the rover only when these checks, recording
and return sequence are prepared for immediate use. End at the saved charging
start, STOP, verify motor output, then copy the complete recording. Do not claim
tape-measured docking precision from map estimates.

## Morning run card

Preparation must finish with the rover off. The prepared local caller is
`.cache/visibility_morning_call.py`, targeting the unused
`captures/visibility-morning-prepared-1` directory. Use matching world session
`visibility-morning-prepared-1`. Confirm neither exists before first use; if
already used, choose a fresh suffix and update the caller before any RPC. Retain
its motion lock, explicit permission token and return-leg flag. This card is preparation,
not permission to move. Obtain a fresh handover with the charger disconnected.

The immediate objective is the **missing clear side view**, not a claim that all
acceptance coverage fits into one minute. The previous route used almost the whole
budget before reaching it. Limit this attempt to two outbound translations and
one framing turn; any initial chassis turn consumes that same allowance. A full
two-clear-view/parallax trial remains owed if this short run cannot supply it.
Do not weaken the coverage or scoring criteria to call the short run successful.

Use these candidate map points only while the map ID remains `7da19bef3888` and
fresh localization agrees. Save the actual starting pose as HOME; never substitute
last night's start for it. Recheck floor/obstacles in the current camera and grid.

| Point | Map x, y (m) | Purpose |
|---|---|---|
| HOME | Fresh measured start | Return destination and starting photograph |
| A | -18.239, -15.030 | Previously reached approach; previously occluded painting |
| B | -19.050, -14.850 | Proposed side view; visibility and direct approach still unproven |

Before moving, ask the live planner for HOME to A, A to B, B to A and A to HOME.
Use travel bearings for goal headings, not the painting-view heading. Explicit
start poses for later legs are hypothetical; retain that distinction in the
recording. Check the complete paths against the fresh map and camera: no unknown
floor, occupied/inflated-body collision, unexpected loop or unexplained pivot.
For this short trial, reject a leg longer than its direct distance plus 0.5 m.
This is a predeclared trial route limit, not a general navigation acceptance test.
If any leg fails, stop preparation and report the route blockage; do not add P1,
P3 or a longer detour during the run. The exact route can still change after the
precheck; watch the actual motion and retain the 16-second STOP watchdog.

`experiments/entity_association/plan_visibility_route.py` asks only Nav2's
`ComputePathToPose`; it does not navigate, publish velocity or change maps. At
handover, copy it to `/tmp/visibility-plan.py` on Orin, then invoke it from Bash
after sourcing `~/ugv/ros_nav/env.sh` and `~/ugv/ros_nav/dds.sh`. For example, the
hypothetical A-to-B check is:

```bash
python3 /tmp/visibility-plan.py --start -18.239 -15.030 131 --goal -19.050 -14.850 167.486
```

Replace the example start heading with the planned arrival heading at A. Omit
`--start` for a check from the actual current pose. Save every JSON path/error.
The helper is prepared and syntax-checked offline; ROS availability and live
paths must be verified at handover. Planning requests have bounded waits, and
unavailable or timed-out planning means no movement.

Before first movement, complete battery/status/camera, fresh read-only `measure`
and changing raw IMU checks. Require at least 60% charge, trusted localization,
a fresh fit of at least 90% agreeing within 0.25 m and 10 degrees, live lidar and
Nav2, and no active motion. Start navigation and passive board recorders for
**300 seconds**, then enable the unique world-call recording session. Verify they
are producing data before beginning; if setup has consumed a minute of their
window, restart the support recordings before driving. Never wait for a recorder
to expire merely to release the rover after returning.

Run commands serially and wait for each final response, including any yielded
tool session. The schedule is a maximum allowance, not a prediction:

| Time from first motion | Permitted work |
|---|---|
| 0-32 s | At most two translations, HOME to A to B; each has a 16 s watchdog |
| By 48 s | At most one completed framing turn, only if necessary and budget permits |
| By 56 s | One stationary fresh depth inspection; admit only with 8 s remaining |
| By 60 s | Begin return; no more outbound moves or inspections |

At each arrival, check command completion and battery before admitting another
action. A camera preview can determine whether to stop at A or proceed to B, but
its elapsed time counts too. Only inspect while stationary, with the intended
painting in view. The 8-second inspection allowance is not a hard timeout: if a
look runs longer, do not launch competing motion; finish/cancel it as supported,
return immediately and record the timing failure. A failed movement, watchdog
STOP, 35% battery or uncertain position ends evidence collection immediately.

Return through the reached, checked points in reverse order: B to A to HOME, or
A to HOME if B was never reached. After a partial/failed move, STOP and obtain a
fresh position check before choosing a return segment; if that check fails, leave
the rover stopped and request manual recovery. Do not mark an unreached point as
a completed leg. Return commands carry `return_leg` and retain the per-command
watchdog. Restore the starting heading only if a checked turn is appropriate;
otherwise report the difference rather than spending extra battery docking.

At HOME, send STOP and verify zero commanded speed and zero motor PWM, save the
final camera, pose and battery, and tell the owner movement is finished and the
charger can be connected. Close the world recorder after the final stationary
settle; retain support recording through that STOP. Copy and hash-check evidence
before saying it is safe to power off. Report map-estimated return error plainly.

## Freeze and judge the result

Review the physical subject and selected pixels with assignments and candidate
results hidden. Distinguish pure object, valid part, mixed, non-object and unresolved
individual identity. Preserve explicit corrections alongside original labels.
A mixed historical crop cannot become independent acceptance truth by analyst
relabeling after scoring.

Reproduce every actual control checkpoint, ordered map query and retained depth
answer before any candidate replay. Use the matching-only measured-visibility
candidate frozen in a61f1c4: extend mapped reach only to valid stored depth during
existing-record matching, with original discovery/refitting and all other gates.
Reject any candidate query with different answers across the recorded pass grids,
including failures caught by the production map callback.

Retain the 98% existing useful-link minimum and zero new reviewed clean cross-object
connections. Require a genuine clear/occluded painting connection gained. Report
all unreviewed changes, failed coverage and missing measurements; a mixed crop
needs separate treatment, not a forced single-object label. Do not count a small
successful subset as deployment acceptance or claim complete old-record repair.

Pure-table/glass views and independently measured transparent-object distance remain
separate owed controls. The partial/whole armchair representation issue and duplicate
historical placements need separate work. A guard against frozen rotation feedback
also remains unimplemented: reproduce failed feedback and healthy moving/stationary
recordings before proposing control changes. Recovery after one power cycle does
not establish that the sensor fault cannot recur.
