# Plans: work that is not finished

A plan is how a body of unfinished work is meant to reach the point where
requirements can be called settled. Plans have a lifespan: they are written,
worked through, and eventually retired once what they describe is either running
or abandoned.

| Plan | Status | About |
|---|---|---|
| [semantic-world-state.md](semantic-world-state.md) | mostly delivered; acceptance work open | remembering what the rover has seen, and searching it by description |
| [autonomous-curiosity.md](autonomous-curiosity.md) | Phase 0 in progress | the rover choosing for itself what to investigate, and learning skills from experience |
| [autonomous-curiosity-design.md](autonomous-curiosity-design.md) | proposed | the architecture the plan above implements |

## What a plan owes

**Name the requirements it will settle.** A plan that cannot say which
[requirements](../requirements/README.md) it moves, and to what, is a wish list.
Reference them by identifier so the connection survives both documents being
rewritten.

**Say what would count as done, before doing it.** Predeclare the success
predicate, the minimum improvement and the number of trials. A criterion written
after the results are in is not a criterion.

**Distinguish what is agreed from what is only proposed.** Later phases of a long
plan are guesses about work that has not started. Say so, so that nobody treats
a phase heading as a commitment.

**Carry a status line at the top.** One or two sentences: which phase is live,
what is blocking, and where the current evidence is. It is the first thing read
and the thing most likely to go stale, so it is worth keeping short enough to
update.

## What a plan does not own

**How the thing currently works.** That belongs in the component's README. A plan
describing the running system is a second, diverging copy of the truth, and the
one in the plan is always the one that rots. When a phase lands, move what it
built into the component README and leave the plan describing only what is still
ahead.

**Measurements.** Results go in [../progress/](../progress/README.md) as dated
entries. A plan may summarise where things stand and link to the evidence, but a
plan that accumulates its own findings becomes unreadable and quietly loses the
ones that were inconvenient.

**Decisions taken along the way.** When a plan rules something out on evidence,
that becomes a record in [../decisions/](../decisions/README.md) so it can be
found by somebody who never reads the plan.

## Retiring one

A plan is retired when everything it describes either runs or has been
abandoned. Move whatever is still true into the component README and the
requirements, then delete the plan — its history is in Git and its findings are
in the progress log. A plan kept as a trophy is read as current work by the next
person.

The two plans here are at opposite ends of that life.
[semantic-world-state.md](semantic-world-state.md) has already been reduced to
its remaining constraints and acceptance work, with operation moved into
[world_state/README.md](../../world_state/README.md);
[autonomous-curiosity.md](autonomous-curiosity.md) is almost entirely ahead of
itself.
