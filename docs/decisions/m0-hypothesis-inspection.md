# M0 permits investigation before persistent identity is trusted

Status: agreed 2026-09-10; acceptance contract revised, runtime permission unchanged.

The owner agreed to relax the identity prerequisite for bounded exploratory
inspection. M0a will permit a safe move to test an explicitly uncertain hypothesis;
M0b will retain independent identity validation before actions rely on that identity.
Both gates remain unpassed. This decision supports R-AUT-12 and revises the scope
of R-WS-13; it does not waive R-SAFE-3, R-SAFE-4, R-SAFE-12 or R-WS-16.

## Why change the gate

The previous M0 required zero known incorrect associations among all associations
eligible to influence movement. That also excluded movement intended to discover
that an association was wrong. A painting incorrectly placed in the room should
not be treated as a fact, but may be a worthwhile question to investigate from a
separately validated viewpoint. Its estimated location cannot certify traversable
floor or supply movement authority.

This is a relaxation for exploration, not a stronger identity guarantee. The
additional requirements concern explicit hypotheses, independent viewpoint checks,
bounded effort, evidence-backed outcomes and refusing identity-dependent follow-on
actions. Their implementation and hardware proof are still owed.

## What the alternatives showed

Keeping the original gate remains appropriate for identity-dependent actions.
It is too broad as the only entry gate for gathering evidence about uncertain
objects. Tightening geometric uncertainty did not isolate bad associations while
keeping useful coverage: see the
[September 8 measurement](../progress/2026-09-08-uncertainty-cannot-gate.md).

The [September 10 benchmark](../progress/2026-09-10-bounded-entity-fitting.md)
reproduced the baseline and tested bounded candidate search, association revision
and conservative depth contradictions. The full prototype separated four of eight
reviewed mix-ups but retained only 50.7% of known same-object pairs; depth
contradictions made no correction on the three recordings. It did not measure
inspection outcomes or establish which resolver is closest to revised M0.
The experimental resolver is not accepted by this decision.

Simply allowing some wrong trusted identities, or calling every existing goal an
inspection, would leave the original failure intact. Neither is the chosen option.

## Acceptance and consequences

The [plan](../plans/autonomous-curiosity.md#milestone-m0-semantic-state-is-safe-enough-to-influence-goal-selection)
owns the two gates and the [runbook](../runbooks/m0-acceptance-drive.md) owns the
trial manifest. All attempts count. Physical target coverage, correct verification,
false promotion, unresolved outcomes and unsuccessful travel replace raw entity
counts as the usefulness measures. Independent physical measurements check range
and placement; agreement between two estimators is supporting evidence only.

The initial M0a usefulness floor is at least half of all inspection attempts
correctly answering their question, with at least twenty attempts across three
fresh runs. This is a modest
entry requirement, not a result fitted to the existing recordings or a reliability
claim. Later M4 trials must still demonstrate active-perception gain. The same
physical case repeated does not create additional target coverage.

Existing recordings remain development evidence. No previous failure becomes a
pass, no requirement is marked settled, and no service or movement permission is
changed. Implementation must first reproduce the incorrect-association case and
show bounded verification/refusal in replay, followed by supervised hardware proof
under the existing control safeguards.

## What would reopen this decision

Revisit the relaxation if uncertainty cannot be kept out of identity-dependent
actions, if false hypotheses escape their effort limits or affect route safety,
or if fresh trials cannot demonstrate useful verification within the budget.
Disable the inspection path and retain its episodes when those boundaries fail.
