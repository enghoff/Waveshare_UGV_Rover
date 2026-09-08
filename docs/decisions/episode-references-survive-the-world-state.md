# An episode's references have to outlive the world state they name

Status: implemented 2026-09-08, for
[Phase 1](../plans/autonomous-curiosity.md) of the curiosity plan. Supports
[R-AUT-1](../requirements/autonomy.md#r-aut-1),
[R-AUT-2](../requirements/autonomy.md#r-aut-2) and
[R-AUT-3](../requirements/autonomy.md#r-aut-3); it retires nothing.

An episode is a record of the rover deciding something, and everything
interesting it can say points outwards: *this* thing was the goal, *that* picture
is why. The semantic world state those names come from does not hold still
underneath them, and the ways it moves are not equally obvious.

**The dangerous one is the quiet one.** Clearing the semantic world empties the
`counters` table with everything else, so identifiers start again at one — which
is exactly what a repeatable experiment wants and is why it was built that way.
The consequence is that `object:8` recorded last week and `object:8` today are
different objects wearing one name. A stored reference resolved against the live
store therefore does not dangle; it comes back with a stranger, and nothing about
the answer looks wrong. This is not hypothetical: the acceptance drive of
2026-09-08 ran in a store cleared at 07:20 that morning, so its `object:8` is
already a different object from the previous day's.

Three further movements, each loud and each real. A merge deletes the losing
entity outright, so a reference to it resolves to nothing. A thing that turns out
to be two objects gets taken apart — three of the twenty-one most-looked-at
things on that drive were two objects each — so a reference then names part of
what it named. And the owner may delete evidence, or retention may expire it.

## What was decided

**Names carry the generation of the store that minted them**, so a stale one
fails closed. A world reference is `ws/<generation>/object:8`; the generation is
sixteen hex characters minted by `WorldStore` when its database is created and
minted again by `clear`. A reference whose generation is not the live one may not
be looked up at all. The token is reported in the world state's summary, so
anything holding a name holds the means to tell whether it still means anything.

**A decision keeps a copy of the world it was made from**, and a replay reads
that copy rather than the live store. This is the part that makes a
reconstruction independent of time: whatever happens to identity afterwards, the
episode still says what the rover saw and chose. Recording a decision against a
live dictionary is refused where it is written rather than discovered at replay.

**Evidence is named by its own bytes.** A frame is `sha256:...`, copied out of
the world state's frames directory before a clear can empty it. Content
addressing gives global uniqueness with no namespace to agree on, survives every
clear, and deduplicates the common case of many episodes looking at one thing.

**What happened to identity afterwards is recorded beside the episode, never
inside it.** A merge is an alias row; a thing taken apart into three is three
rows sharing a source, because "what is it now" genuinely has three answers. A
reader is shown what the episode said and what that thing is called today.

## The alternatives, and how they fail

**Store the bare local identifier.** What an episode would naturally record, and
the reason this document exists: after a clear it silently names something else.
Rejected outright — it is the only option here that can produce a confident wrong
answer rather than a visible gap.

**Scope names by map session.** The world state already stamps rows with one, and
reaching for it is tempting because it is there. It answers a different question:
a map session is about the coordinates a placement was measured in, and it
deliberately does not move when the semantic world is cleared. Two stores either
side of a clear share a map session and share identifiers, which is precisely the
case that must be distinguishable.

**Have the world state stop reusing identifiers.** A monotonic counter that
survives a clear would make bare names unique for ever, and it is a smaller
change than this one. It was rejected on the experiment's terms rather than the
record's: starting a fresh recording at `object:1` is what makes two acceptance
drives comparable, and the world state's own store documents that as the point of
counting in a table. It also fixes only the clear, leaving merges, splits and
deleted evidence untouched.

**Resolve references lazily and mark what has gone.** Keep bare names, and have
the replay ask the world state what still exists. It cannot work, for the reason
above: after a clear the lookup succeeds and returns the wrong thing, so there is
nothing for the replay to notice.

**Reference frames by path or row id.** Both are reused after a clear, and both
name a file the world state may delete. Content addressing costs a hash per frame
and removes the question.

## What would reopen it

Evidence that copying frames into the episode record is what fills the rover's
disk. The measurement to make is a shadow run's evidence in megabytes per hour
against what the world state itself writes; if copying is the dominant cost, the
alternative to weigh is a reference that pins a frame in place rather than
duplicating it, which trades disk for a coupling this decision deliberately
avoided.

Evidence that the generation token is not enough — most likely a way for the
world state to reissue identifiers without minting a new generation. Any such
path is a bug in `WorldStore` rather than a reason to change this, but it would
be worth knowing which.

## What this does not settle

Nothing tells the episode record about a merge or a split. The alias table exists
and is tested; the world state does not call it. Until something does, identity
changes are recorded only when a person or a later pass puts them there, and a
reader is told what an episode said rather than what became of it.
