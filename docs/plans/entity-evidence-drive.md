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
