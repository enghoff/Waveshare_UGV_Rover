# Test whether ambiguous depth can safely abstain

R-WS-13 remains open; R-WS-17 and R-WS-18 remain proposed. This is an offline
development test, with criteria fixed before computing the new measurements.
No identity or range policy is authorised by a pass on reused development data.

## Fixed candidate and controls

Replay the production outline sampler from saved depth. Its nearest band supplies
the ordinary answer. Abstain only when that band contains less than half of valid
projected samples **and** the overall median exceeds its median by more than 0.5 m.
The purpose is to reject a precise answer about a minority foreground surface,
not to replace it with the farther surface. Missing outlines, failed projections,
box fallbacks and missing ranges remain unchanged. No trimming or mask erosion.
Use the existing two projection iterations and minimum-pixel rules.

The known painting view 68640 is a reproduction check, not independent evaluation.
Apply the same rule to the older recorded drives and their frozen taped-target
mapping. Regenerate unavailable outlines with the exact cached region-model export;
require bounding-box overlap of at least 0.5 and report every mismatch. Archive the
model, source, frame/depth and input hashes. Compare regenerated control readings
with the earlier untrimmed-outline replay before interpreting candidate results;
differences must remain visible, not be replaced with the cached answers.

## Fixed decision criteria

The candidate must abstain on 68640, retain at least 90% of control readings within
0.25 m of their taped distance across the older drives, and remove at least half of
control errors of 1 m or more. Report counts per physical target and drive, and all
lost correct readings and unresolved cases. A failure on any criterion rejects this
candidate; no threshold scan or retuning in this experiment.

Also review glass/narrow-part examples from older recordings without candidate
results or assignments exposed. A nearest minority surface can be the intended
object. Such examples test the interpretation of the rule; where no taped truth is
available they cannot be counted as independent range correctness. Keep this limit
visible rather than treating the overall median as truth. This pilot measures range
abstention only; it does not establish retention of identity or placement support.

## What follows

If the fixed abstention loses useful measurements, keep surface ambiguity as a
reported property and investigate its role in association separately. A failed
abstention does not justify choosing the far surface, inflating every geometric
claim, or relaxing the identity target. Do not drive the rover during this work.


## Supplementary check fixed before computing it

Apply the same unchanged rule to the separately recorded 2026-10-03 taped-target
looks using their stored outlines and lens headers. Use the original target
mapping and wall frame; identify stationary looks from the original leg windows.
Report moving and still results separately. Require all stationary reconstructed
ranges to agree with stored ones within 2 mm before interpreting those results,
and retain at least 90% of stationary readings within 0.25 m of the tape. Report
gross-error removal, but fewer than two gross control errors supplies insufficient
evidence of general error removal. Do not reinterpret a lack of flags as a failed
rule, and do not claim an identity-policy pass from a range test.
