# How sure of identity an action must be is decided for that action

Status: agreed 2026-10-01 by the owner. M0b is retired as a gate of P0. Nothing
was built.

There is no general rule for how sure the rover must be about which object is
which. Each action that would rely on identity is evaluated when it is proposed.
The evaluation weighs what a wrong identity would cost that action and decides
what evidence is enough. One action might act on what the world state remembers,
another might need a fresh look first, and another might need more than that.
Until an action's case has been decided, no autonomous run may take it.

So M0b, one bar for every identity-dependent action, is no longer a gate of P0.
P0 closes on M0a and the shared prerequisites.
[R-WS-13](../requirements/world-state.md#r-ws-13) stays open and now states what
each evaluation owes. This revises
[M0b](../plans/autonomous-curiosity.md#m0b-retired-on-2026-10-01) and R-WS-13's
bar. It does not change the M0a half of the
[September 10 decision](m0-hypothesis-inspection.md), R-AUT-12, R-WS-16 or any
safety requirement.

## Why

**Nothing the rover does on its own depends on identity yet.** The executive has
three goals: reach a frontier, improve a thing's geometry, and check whether
something stands where a thing was placed. After a check it only reads. The
daemon lets an autonomous run drive to a point, look and stop, and nothing else.
The voice tool that drives to a named thing is a person's request, and an
autonomous run cannot call it. The replay showed that an inspection's answer
leads to no identity-dependent follow-on
([the replay](../progress/2026-10-01-hypothesis-inspection-replay.md)).

**What a mix-up costs varies enormously between actions, so one bar fits none of
them.** A revisit that looks at the wrong chair wastes a few metres. Reporting
that something has moved when it has not misleads the person who asked. M0b
asked the same thing of both: fifty reviewed decisions with none wrong.

**The store cannot meet that bar today, and holding P0 on it would hold P0 on an
open research problem with no date.**

- On the 2026-09-08 drive, 17 of 76 things held two physical objects.
- On 2026-10-01, 3 of 71 things held more than one object, and no target came
  out as exactly one clean thing.
- The bounded-revision prototype separated four of eight mix-ups but kept only
  half of the true pairs.
- The room has six identical dining chairs.

## What was considered

- **Keep M0b in P0 as written.** P0 waits on the resolver.
- **A rule that every identity-dependent action first confirms with a fresh
  look.** This was proposed earlier the same day. The owner turned it down
  because it is still one rule for every case. Its measurements remain evidence
  that an evaluation can use (below).
- **Confirm by place alone, using M0a's check.** It says that something is
  there, not which thing; a swapped object passes.

What one fresh look can carry, measured on the 2026-09-08 labels. The "wrong"
column is a look at a different object put where the claimed thing was:

| Rule | Right object confirmed | Wrong object confirmed |
|---|---|---|
| likeness 0.70 or better | 43% | 2.3% of 21,983 pairings |
| likeness 0.85 or better | 5% | none of 21,983 |
| resembles the claimed thing by 0.10 more than any other | 13% | 2 of 528 |

## What would reopen this

Either of these would reopen the decision:

- An identity-dependent action reaches an autonomous run without its case having
  been decided. The plan already treats this as a stop condition.
- An action whose case was decided acts on the wrong object more often than its
  evaluation allowed.
