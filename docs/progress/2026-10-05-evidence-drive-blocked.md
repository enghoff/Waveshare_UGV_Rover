# The evidence drive stops at a route refusal and failed localization

The planned independent viewpoints were not reached. After the owner authorized
the prepared short drive, its first move travelled 0.298 m and ended blocked.
A fresh stationary inspection then could not confirm the rover's position:
the best nearby scan fit scored 65%, against the existing 90% requirement.
The trial was stopped without retrying the route. An automatic return was not
attempted with uncertain localization; the owner was asked to place the rover
at its charging position. That manual return has not been verified. R-WS-13
remains open; R-WS-17 and R-WS-18 remain proposed. R-WS-16 remains settled: the
failed looks retained their evidence without supplying unconfirmed bearings.

The navigation API had reported a trusted position, matching parked scan,
live lidar and ready planner before the move. Current occupancy-grid prechecks
showed clearance along the short route, whose positions were reached earlier
that day. Live navigation instead reported a 4.3 m route to the 0.3 m goal and
that the rover stood inside a costmap obstacle. This shows why that precheck
cannot establish a currently usable navigation route. It does not establish
the cause of the obstruction or localization failure. No navigation fix, map
refit, map reset or experimental identity policy was applied.

The completed optional recording, `depth-drive-20261005-1`, is archived under
`captures/2026-10-05-depth-hardware/`. Before/after stores, actual matching calls,
saved grids, four photographs and three retained raw depth maps were copied
before releasing the rover. A three-second navigation recording after STOP
contains five costmaps, 30 poses, zero commands and no motion. It describes the
stopped state, not the controller's missing earlier ticks; it cannot by itself
reproduce or justify a fix for this drive failure.

The unchanged resolver reproduces all 14 actual passes, 43 identity checkpoint
checks and 11,936 ordered reach queries. Archived grids reproduce every live
answer. Twenty new observations were stored: six with bearings before movement
and fourteen without bearings after the scan check failed. All fourteen remain
unassigned. Every one of the eight supplied ranges reproduces exactly from the
retained raw depth, as do all 19 sampler answers whose depth was retained. One
ordinary background frame lacks raw depth; its single observation had no range.
The partial-depth report names that omission and explicitly denies complete
sampler proof. No measurement was reconstructed from a photograph or guessed.

None of the eight checked ranges triggers the fixed minority-band rule. No
range counterfactual was run and no threshold was changed. This failed drive
therefore supplies no new evidence that the rule improves associations, no
independent same-subject links, and none of the missing glass, pure-table or
glare-armchair controls. The preceding isolated painting correction and older
cabinet losses remain the evidence for and against blanket use.

The [measurement](2026-10-05-evidence-drive-blocked.json) records the real control
calls, source/input hashes, coverage and limitations. The next step is to confirm
localization at the charging position and reproduce the navigation fault with
controller recording before proposing a fix. Prepare all work offline before
requesting the rover again; further movement requires a new driving prompt.
