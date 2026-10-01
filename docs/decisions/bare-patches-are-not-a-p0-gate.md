# Bare floor and background patches do not hold up P0

Status: agreed 2026-10-01 by the owner; no filter built, R-WS-12 stays open.

P0 no longer requires that a bare patch "cannot" become an object-inspection
goal. [R-WS-12](../requirements/world-state.md#r-ws-12) stays open as a
requirement on the world state, but neither M0a nor M0b waits on it. This
revises shared prerequisite 5 of
[M0](../plans/autonomous-curiosity.md#milestone-m0-semantic-state-is-safe-enough-to-influence-goal-selection)
and nothing else; it does not touch R-AUT-12's limits on an attempt, R-WS-16, or
any safety requirement.

## Why

The rule as written cannot be met by anything the rover can run, and the cost
of a bare patch slipping through is already bounded. Under M0a a patch that
becomes a hypothesis costs at most one attempt within its frozen travel, time
and attempt limits, and should end unresolved or contradicted. That is a waste
of a few metres, not a safety or correctness fault. The owner's judgement is
that real objects will be far more salient and plentiful than bare patches once
the rover is choosing where to look, so the waste may never matter.

## What was tried

On 2026-10-01 bare patches reproduced on the
[morning's drive](../progress/2026-10-01-the-room-as-it-stands.md): five things
in 69 reviewed held no object. Rules were designed on the 2026-09-08 labels
(seven bare patches in 76) and then scored once on the 2026-10-01 drive.

| Rule | Development: caught | Held out: caught | Held out: real things excluded |
|---|---|---|---|
| glare or low featureless floor | 5 of 7 | 3 of 5 | 1 |
| SigLIP zero-shot, surface prompt beats object prompt by 0.02 | 5 of 7 | 3 of 5 | 3 |
| SigLIP, half or more of looks favour a surface prompt | 7 of 7 | 5 of 5 | 19 of 66, mostly paintings |
| either of the first two | 5 of 7 | 4 of 5 | 4 of 64 |

No rule reached zero misses without also excluding real things. The misses were a
floor strip with a rug edge in it, a painted stripe on a bare wall, and glare;
the wrong exclusions were partly seen or blurred real objects and a blown-out
table. The scripts and tables are kept outside Git with the drive, in
`captures/m0-2026-10-01/eligibility/`.

The alternatives were to adopt the union rule and accept the lost objects, to
require strong positive evidence of an object (which loses the paintings), or to
keep "cannot" and leave P0 unclosable. The owner chose none of them for now.

## What would reopen this

Bare patches turning up among M0a's chosen hypotheses often enough to cost
attempts that real objects would otherwise have had, or a patch whose attempt
does not end within its limits. Either is visible in the M0a run reports, which
record every attempt. A filter built then should start from the union rule above
and be scored on a fresh drive, not on these two.
