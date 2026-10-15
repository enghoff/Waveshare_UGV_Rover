# Plans: work that is not finished

A plan is how a body of unfinished work is meant to reach the point where
requirements can be called settled. Plans have a lifespan: they are written,
worked through, and eventually retired once what they describe is either running
or abandoned.

| Plan | Status | About |
|---|---|---|
| [semantic-world-state.md](semantic-world-state.md) | mostly delivered; one thing per object (R-WS-17, R-WS-18) in investigation | remembering what the rover has seen, and searching it by description |
| [entity-evidence-drive.md](entity-evidence-drive.md) | recorded and scored; re-scored once the world state rebuilds the painting's eight mixed records (R-WS-13, R-WS-17, R-WS-18) | connect verified views across viewpoint changes and separate foreground from background |
| [autonomous-curiosity.md](autonomous-curiosity.md) | Phase 0 closed; M3 and M4 passed 2026-10-10 | the rover choosing for itself what to investigate, and learning skills from experience |
| [sessions-that-serve-several-milestones.md](sessions-that-serve-several-milestones.md) | proposed 2026-10-11 | one supervised session per charge serving M5, the multi-day benchmark and later milestones together, around a fixed survey loop, with everything else on recordings |
| [autonomous-curiosity-design.md](autonomous-curiosity-design.md) | proposed | the architecture the plan above implements |
| [perception-that-keeps-what-it-learns.md](perception-that-keeps-what-it-learns.md) | proposed 2026-10-12; full-resolution evidence landed the same day (R-WS-19); the rest not built (R-WS-11, R-WS-12, R-WS-17, R-WS-18) | keeping what the cameras see at full resolution in a dense 3D field, and what models and the owner understand about it as claims with evidence |
| [what-where-streams.md](what-where-streams.md) | proposed 2026-10-14; step 1 run offline on the M5 shakedown, met its frozen bar narrowly; nothing on the rover (R-WS-8, R-WS-16, R-WS-17, R-WS-18) | comparing expectation with observation in a frozen appearance space ("what") and on the map ("where") per thing, not as pictures or depth: new, out of place and gone, calibrated on unchanged revisits, with a latest-state memory of things and learned predictors to follow |
| [perceiving-by-prediction.md](perceiving-by-prediction.md) | proposed 2026-10-12; nothing built; its house model replaced by what-where-streams on 2026-10-14, its rules and gates kept (R-WS-2, R-WS-8, R-NAV-3, R-NAV-9) | the specific architecture for relying on prediction: a body model and a house model of stored RGB-D looks, unknown kept apart from measured, changes and identities pending until a later look settles them, errors explained at the highest level that accounts for them (a moved chair as one placement change, a damaged cushion as a content change inside it), and consumers switched on one gate at a time, starting with where to point the camera |
| [perception-from-first-principles.md](perception-from-first-principles.md) | proposed 2026-10-12, a design with hardware set aside; nothing built; five tests on recordings declared (R-NAV-9, R-NAV-11, R-WS-17, R-WS-18) | what brains and video world models suggest: predicting rather than describing, a fovea aimed where unsure, anchoring on what does not move, objects as paths, two memories joined by nightly replay |
| [a-world-model-kept-per-visit.md](a-world-model-kept-per-visit.md) | proposed 2026-10-15 at the owner's direction; step 1 (benchmark and baseline) live offline on branch `visit-model`; the resolver and the move step paused (R-WS-17, R-WS-18) | objects tracked through each visit, matched against memory once per visit, and marked gone when their place is seen empty, scored against the deployed store on the looks since 2026-10-03 |
| [battery-charge-curve.md](battery-charge-curve.md) | recording the pack; nothing fitted yet (R-CTL-11) | making the battery percentage mean the share of a charge left |

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
