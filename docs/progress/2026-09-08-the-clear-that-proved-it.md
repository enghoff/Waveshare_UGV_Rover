# The world state was emptied for real, and 210 episodes went quiet about what they named

**The generation token did on hardware what it had only ever done against a
fake.** Another agent cleared the semantic world state this afternoon to begin
the M0 acceptance drive. The 210 episodes recorded that morning name things in a
store that no longer exists, and asked to resolve those names against the store
that is live now, the record refuses every one of them — while still saying
exactly what the rover did.

Taking one episode that named nine things: **nought of nine resolvable**, each
refused with *belongs to a world state that has since been cleared; the
identifier has been reissued*. The episode itself reads as it always did — a look
that found twelve regions, ten of them attached to things the rover already knew,
none ranged, closed `succeeded`, one picture kept. That separation is the whole
design, and until this afternoon it had only been shown against a test.

The store went from `f49e9206997fbe42` to `4d2fda72621ef169`, taking 1594
observations and 128 things with it. The map was not touched.

## Why this is the dangerous case rather than a tidy one

A cleared store is not a store with holes in it. The identifier counters restart,
so `object:42` is handed to a different object as soon as the rover has looked at
enough of the room. At the moment of checking, the new store held eleven
observations and no things at all, so `object:42` resolved to nothing — but that
is a property of the drive not having started, not of the record being safe.
Twenty minutes later it will be a real object, a different one, and a record that
resolved the stored name would have produced a confident, wrong account with
nothing about it looking broken.

## An episode now says which build produced it

The other agent pointed out something the morning's shadow run had hidden. The
deploy that restarted the daemon mid-run also changed the resolver's association
rules — a look is now refused if another thing in the room resembles it too
closely, and an exemplar is only learnt from a strong match. So the looks
recorded before about 09:43 and the ones after it were **decided by different
rules, and nothing in the record said so**. Anyone comparing the two halves of
that run would have been comparing two experiments while believing they had one.

Every episode now carries the commits `world_state`, `rover_daemon` and `ros_nav`
were deployed at — the three whose build changes what a look means — read from
the deployer's own state file, which is the same file it refuses to advance until
a component's checks have passed. A deploy landing mid-run is noticed, marked and
reported at the end of the run, and so is the semantic world being emptied, for
the same reason: the things named either side of it are not the same things.

This is the *map and calibration versions* half of the plan's durable evidence
contract, which had simply been left out.

**It cannot be applied backwards.** This morning's 210 episodes are silent about
which rules made them. The boundary is around 09:43 and is known only from the
other agent's account of when their deploy landed, which is exactly the situation
the stamping exists to prevent and exactly what it cannot repair.

## What the record did and did not observe

The recorder was not running when the clear happened, so it recorded the new
generation as the first world it had seen rather than watching the change. The
change is still plain in the record — 210 episodes carry the old generation and
everything since carries the new — but the mark reads as a first sighting, not as
a transition. A recorder running across the next one will mark it properly; the
offline suite covers that path.

## Requirements

- [R-AUT-2](../requirements/autonomy.md#r-aut-2) held, and its evidence
  strengthened from an offline suite to a real clear on the rover. It was already
  `settled`; what changes is what it is settled on.
- [R-AUT-1](../requirements/autonomy.md#r-aut-1) and
  [R-AUT-3](../requirements/autonomy.md#r-aut-3) untouched.
- Nothing about M0 moved. The clear was the other agent's, for their acceptance
  drive, and this entry only records what the episode record did about it.
- Counts: autonomy 334 passed, 0 failed.

## Next

The driven half of M1's criterion 4 is still open, and a recorder is watching for
it: no episode has ever recorded a move, because nothing has driven while
anything was watching.
