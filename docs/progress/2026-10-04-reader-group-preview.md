# Reader groups can be inspected without changing observation assignments

R-WS-13 remains open; R-WS-17 and R-WS-18 remain proposed. The reader design now
has a diagnostic implementation: `world_state_groups {}` recomputes groups on a
disposable copy and changes no live identities. The console, voice and executive
still read original entities. This is preparation for independent acceptance,
not acceptance or automatic physical merging.

## Scope and policy

One person across head/body views is the working identity policy. The disputed
painting observation 61656 remains excluded in sensitivity scoring, without
changing its historical label. These assumptions remain explicit; no owner label
confirmation is inferred from permission to continue work.

The preview uses the existing development appearance weights by default. Replay
fits held-out weights as in the preceding audit. Candidate member IDs have no
persistent forwarding semantics or navigation position. A new preview starts
from original records and can withdraw an old group. Generation, map session,
evidence revision, staleness and convergence accompany the result.

The nominal squared ellipse limit 13.8155 now rejects incompatible pairs before
ranking (R-WS-8); it is not a calibrated probability guarantee. Backend and
shared-picture checks remain intact through successive grouping rounds. The
development recording's first 17 original proposals were already inside this
limit, so adding it is a required guard rather than a claim to have reproduced
an existing out-of-ellipse failure.

## Verification

The world-state suite passed 1,050 checks, the daemon suite passed 1,098 checks,
and the updated daemon store/control-call test passed 39 checks. Six focused
preview scenarios passed 16 checks: every source row unchanged, withdrawal after
new shared-picture evidence, strong appearance rejected by geometry, concurrent
preview refusal, stale evidence during calculation, shared-picture constraints
through a chain, and older schema migration restricted to the disposable copy.

On the real 3,540-observation recorded store, the diagnostic converged to 18
candidate groups spanning 41 original records in 3.81 seconds on the workstation.
The source recording's SHA-256 stayed unchanged. These candidate counts have
not been independently judged.

Both complete implementation replays reproduced every original resolver
membership and every earlier bench score at all five checkpoints. Same-object
pairs together were 124, 394, 1,165, 2,218 and 3,932; baseline had 124, 394, 1,105,
2,174 and 3,716. Wrong pairs were unchanged at 22, 120, 247, 472 and 710. The
[compact results](2026-10-04-reader-group-preview.json) retain these checks and
the policy. The final schema-migration guard was verified separately on an older
store; the replay stores already had the current schema. Frozen runtime source
is in `.cache/entity-audit/reader-source` for reproduction.

## Deployment proof

Commit `50858c172d7bc78d832f7062a52969c57ba9e2db` on branch
`entity-reader-groups` was pushed and deployed to Orin using the registered
`world_state` and `rover_daemon` components. Perception and daemon restarts passed
readiness checks; the on-host suites passed 1,052 and 1,100 checks respectively.
The new call was then made over TCP 8769, after both components completed.

The live preview converged to 20 candidates spanning 44 original records from
195 entities and 3,784 observations, in 10.78 seconds. It was current, not stale.
Read-only database digests before and after matched for every entity row, every
observation's ID/entity/note membership record, and all three merge-journal tables.
The journals remained empty. An apply argument was refused. Raw proof is in
`.cache/entity-audit/live-module-proof.json`; these candidates remain unjudged.

The earlier session's uncommitted automatic-merge/autonomy work remains intact
in the main checkout. Deployment used the clean separate checkout at
`.cache/entity-reader-groups`, and carried none of those edits. Main still points
to the preceding audit commit; the deployed implementation is on the pushed
`entity-reader-groups` branch. Branch integration must preserve that pending work.

The rover was reachable over TCP 8769 during this work. Its current state included
recent observations with no bearings and monotonic values in wall-clock fields.
Those looks do not constitute a new independent acceptance drive. Fresh physical
object labels and usable recording provenance remain the next acceptance work.
