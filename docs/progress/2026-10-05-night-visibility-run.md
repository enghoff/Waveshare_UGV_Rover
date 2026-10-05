# Night recording returns safely but misses the required side view

The visibility trial did not obtain its required clear side view. Indoor lighting
was sufficient for readable stationary photographs, but both useful looks still
showed the painting behind chairs. The side route did not complete and its
attempted inspection overlapped motion. No candidate replay or policy acceptance
is claimed. R-WS-13 remains open; R-WS-17 and R-WS-18 remain proposed; R-WS-16
remains settled. The rover returned to its starting area and stopped; no
navigation or matching fix was deployed.

## Preconditions and what happened

The owner authorized driving and warned that it was dark. A stationary camera
check showed the room lights on and visible paintings. Battery reported 80% at
11.96 V. Status contained an earlier failed map check; a fresh read-only scan
measurement instead scored 96.6%, within 4 cm and 1.5 degrees of the believed
pose. Raw board IMU tuples changed normally and gyro bias was nonzero. No map
reset, refit or control change was made.

The live global-grid precheck passed every prepared segment, including at least
0.743 m clearance on the proposed extension. Three outbound waypoint moves
arrived. The extension to (-19.050, -14.850) nevertheless chose a 2.72 m route for
a goal about 0.7 m away. Its independent 16-second command deadline sent STOP;
the command reported stopped after 0.220 m and -131.4 degrees. Global-grid
clearance did not prove that navigation would take the anticipated short route.
The archived controller inputs and outputs cover this failed extension.

I made two execution mistakes. I started the final inspection while the movement
tool was still running, rather than waiting for its completion. Its image is
motion-blurred, the rover turned 26.8 degrees during capture, and no regions were
retained. I also missed the 60-second return cutoff: the return began at 109.7
seconds after the initial turn. These are operational failures, not exceptions
to the frozen plan. The 150-second controller/board recordings ended before the
final home leg; they must not be described as covering the entire return.

A fresh stationary scan after stopping scored 98.2%, with a reported correction
of 14 cm and 5.5 degrees. The three return moves along the reached route arrived.
Final map position was 0.182 m from the saved start and heading differed by
29.5 degrees. STOP was acknowledged, commanded speed was zero and motor PWM was
[0, 0]. The final photograph showed the starting armchair/cabinet scene. These
are navigation estimates and camera evidence, not tape-measured docking. Battery
still reported 75% at 11.86 V. The owner was told the charger could be reconnected.

## Recording proof and limits

The completed `visibility-20261005-night-1` call recording was copied locally.
All 23 files in the initial recording transfer matched remote SHA-256 values;
the empty inspection's photograph and raw depth were also recovered separately.
Ten photographs and seven raw depth maps are retained, including that failed
capture. The local archive is `captures/2026-10-05-visibility-drive/`.

The unchanged matching control reproduces all 15 actual resolver passes,
61 identity checkpoint checks and 18,947 ordered reach queries. Every saved
grid reproduces its logged answer. All 35 supplied ranges and all 54 available
sampler answers reproduce exactly. Two projection frames lack raw depth,
affecting 16 unranged observations; one other frame has no projection event,
affecting ten unranged observations. Complete sampler proof remains false.

There are 80 new observations in nine nonempty frames, all with stored bearings.
The two requested stationary inspections kept eight and seven regions; the third
kept none. Four distances meet the existing minority-band flag, but no abstention
or visibility policy trial was run on this failed-coverage recording. Reproduced
distance values are not independent distance truth.

The recording includes intervening production perception/search changes made by
the other agent. Its manifest records those source hashes; matching source is
unchanged. This is another reason not to attribute differences from the earlier
daylight recording solely to lighting. The exact call replay remains valid
because it reuses the actual retained measurements and reproduces every state.

## Next step

The saved global costmap's direct centreline to the side goal has zero cost in
all 101 sampled cells. That snapshot predates the live plan, so it does not prove
the live costmap was identical. With the recorded start/goal headings, the existing
Python lattice model produces a 2.014 m path; with both headings along the segment
it produces 0.463 m. The live path was 2.720 m. Heading constraints are therefore
a route-preparation hypothesis, not an exact reproduced navigation fault or a
justified controller fix. Keep movement heading separate from the later
camera-view heading when checking a future route.

The local diagnostic caller now uses `drive_trial_guard.py` to refuse a fresh
inspection while its movement call is active and reserve a 16-second motion
deadline or eight-second inspection allowance within the 60-second outbound window. Replaying the actual call timeline
refuses the outbound moves at 59.9 and 62.7 seconds and the overlapping inspection
at 72.8 seconds, while allowing the prior actions and designated return legs.
The inspection allowance is not an enforced perception deadline. The check sends
no motor commands itself and does not guarantee an automatic return;
return control and localization checks remain necessary. This is a local test
runner correction, not a deployed navigation guard. A failed motion RPC leaves
the runner's lock for explicit recovery instead of assuming the movement ended.

Keep the matching-only visibility candidate undeployed. Before another drive,
use the captured failed-extension plans/costmaps to choose a route that can supply
the missing view. Use the new outbound admission and motion-completion checks;
model-side timing reminders did not suffice. Recording duration must cover the return, or a missing final leg must
again be reported. Do not repeat the same extension on global-grid clearance
alone. Further driving requires a fresh prompt once preparation is complete.

The [measurement](2026-10-05-night-visibility-run.json) records all motion outcomes,
timing failures, source/input hashes and coverage. These changes affect only
documents and offline experiments, with no service restart required. The hardware is no longer needed for
checking this saved run.
