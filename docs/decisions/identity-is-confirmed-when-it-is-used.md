# An action that relies on identity confirms it with a fresh look first

Status: proposed 2026-10-01 for the owner to decide; nothing built or changed yet.

**The proposal.** No action may rely on which thing the world state remembers a
thing to be. A remembered identity can only nominate. An action whose correctness
depends on identity must first take a fresh look of its own at the place, and
must refuse if that look does not confirm the thing. M0b would then judge that
confirmation, not the store's associations.

**What it does to P0.** P0 would keep the half of M0b that keeps the rover safe:
the deployed path refuses every identity-dependent action. The half that makes
identity useful moves to M5, the first milestone that has such an action: at
least fifty reviewed decisions with none wrong, and coverage of real targets.
M5's criterion 4 already asks for this on moved objects. P0 would then close on
M0a and the shared prerequisites.
[R-WS-13](../requirements/world-state.md#r-ws-13) stays open, with its bar
applied to the confirmation. This revises
[M0b](../plans/autonomous-curiosity.md#m0b-actions-that-rely-on-persistent-identity)
and nothing else. M0a, R-AUT-12's limits, R-WS-16 and every safety requirement
are untouched.

## Why

**Nothing the rover does on its own depends on identity yet, so the safety half
can be proven now.** The executive has three goals: reach a frontier, improve a
thing's geometry, and check whether something stands where a thing was placed.
After a check it only reads. The daemon lets an autonomous run drive to a point,
look, and stop, and nothing else. The voice tool that drives to a named thing is
a person's request, and an autonomous run cannot call it. The replay showed that
an inspection's answer leads to no identity-dependent follow-on
([the replay](../progress/2026-10-01-hypothesis-inspection-replay.md)).

**The store cannot meet the bar, and nothing tried comes close.**
- On the 2026-09-08 drive, 17 of 76 things held two physical objects.
- On 2026-10-01, 3 of 71 things held more than one object, and no target came
  out as exactly one clean thing.
- The bounded-revision prototype separated four of eight mix-ups but kept only
  half of the true pairs.
- Placement uncertainty cannot gate.
- The room has six identical dining chairs.

Holding P0 until the store passes means holding it on an open research problem
with no date, and the owner has ruled out a bar that just fails P0.

**A look taken at the moment of acting can be made as strict as it needs to be.
A stored association cannot.** Suppose a different object is put where the
claimed thing was. A single look at it scored 0.85 or better against the claimed
thing in none of 21,983 pairings. But a look at the right object also cleared
0.85 only 5% of the time. One strict look is safe but almost never says yes.
Raising that rate is the work this decision leaves for M5. Candidates are
several looks, M0a's place check combined with appearance, and the masked crop
that already separates a chair from the painting behind it. M5's 80% bar for
moved objects stays.

## What else was considered

| Option | What it would mean |
|---|---|
| Keep M0b in P0 as written | P0 waits on the resolver, with no date |
| Confirm by place alone, using M0a's check | Says something is there, not which thing; a swapped object passes |
| Confirm with one look at 0.70 | Confirms 43% of true matches, but also 2.3% of swapped objects |
| Require the look to favour the claimed thing by 0.10 | Confirms 13% of true matches, and passed 2 of 528 wrong ones |

## What would reopen this

Any identity-dependent action proposed for an autonomous run before M5, such as
a revisit chosen because of what a thing is. Also, a confirmation that passes M5
and then acts on a wrong object in the field.
