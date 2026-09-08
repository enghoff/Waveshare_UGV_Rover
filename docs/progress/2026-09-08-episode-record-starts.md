# Phase 1 starts: a record of what the rover decides, with names that survive a clear

**The rover can now keep an account of a decision that will still be true next
month, and nothing writes one yet.** `autonomy/` is on the Orin and passes its
168 checks there; the semantic world state now mints and reports a token saying
which filling of it a name belongs to. What is missing is the caller — no episode
is recorded during ordinary running, so M1's thirty-minute shadow run has nothing
to record.

This entry is a build and a deployment rather than a measurement of the rover's
behaviour. It is here because it settles three requirements and restarted the
rover's services, and both are things the next person needs to find.

## Why it needed doing before anything decides where to drive

An episode is only worth keeping if a stored name still means what it meant. It
does not, today, and the way it fails is silent. Clearing the semantic world
empties the identifier counters with everything else — which is deliberate, and
is what makes two acceptance drives comparable — so `object:8` before a clear and
`object:8` after it are different objects wearing one name. A stored reference
resolved against the live store does not come back empty; it comes back with a
stranger, and nothing about the answer looks wrong.

That is not hypothetical here. [The acceptance drive of this
morning](2026-09-08-acceptance-drive-two.md) ran in a store cleared at 07:20, so
its `object:8` is already a different object from the previous day's.

## What was built

The reasoning is in [a decision
record](../decisions/episode-references-survive-the-world-state.md) and the
component in [its own README](../../autonomy/README.md). In short: a reference
carries the generation of the store that minted it and may not be looked up
against any other, so a stale name fails closed; a decision keeps a copy of the
world it was made from, and a replay reads that copy rather than the live store;
evidence is copied out of the world state and named by the hash of its own bytes,
which survives every clear and deduplicates.

Nothing already written is ever changed. There is no `UPDATE`, `DELETE` or
`REPLACE` in the store, and the check is not a reading of the source: a whole
episode is recorded with SQLite reporting every statement it issues, and none of
them changes a row.

`world_state` gained the generation itself — sixteen hex characters minted when
the database is created and again by `clear`, reported as `world_generation` in
its summary. The map session deliberately does not move with it: that one is
about the coordinates a placement was measured in and survives a semantic clear,
and conflating the two is the mistake this avoids.

## What was deployed, and what proved it

Both components went to the Orin at `2d492a5` from a detached worktree, because
another agent had `locate.py` and `resolve.py` dirty; nothing uncommitted went
with them. Perception and the daemon were restarted.

- `world_state`: 806 passed, 0 failed on the rover. The daemon reports
  `world_generation` `f49e9206997fbe42` beside map session 67 and 1462
  observations.
- `autonomy`: 168 passed, 0 failed on the rover.
- One episode end to end through the deployed pair, on the Orin: it read the live
  generation off the daemon, recorded a decision about `object:105` and
  reconstructed it; asked to resolve that same reference against a different
  generation it refused, saying the identifier had been reissued, while still
  reporting the episode itself as complete and readable. That refusal is the
  whole point of the exercise.

**The rover's existing store had no generation, so one was minted when the daemon
first opened it after the deploy.** It separates everything from today onwards
and cannot separate what came before, including the 07:20 clear. Nothing can,
retroactively; names from before today are permanently in the unresolvable
bucket, which is the honest place for them.

## Requirements

- [R-AUT-1](../requirements/autonomy.md#r-aut-1),
  [R-AUT-2](../requirements/autonomy.md#r-aut-2) and
  [R-AUT-3](../requirements/autonomy.md#r-aut-3) are new and `settled` on the
  offline suite — all three are rules about how the record behaves rather than
  about hardware.
- [R-AUT-4](../requirements/autonomy.md#r-aut-4),
  [R-AUT-5](../requirements/autonomy.md#r-aut-5) and
  [R-AUT-6](../requirements/autonomy.md#r-aut-6) are new and `open`. Replay has
  no path to the rover in this build and that has not been shown on a rover with
  a daemon beside it; the component has no movement-capable call and no shadow
  run has demonstrated it; and retention is neither implemented nor measured.
- M1 criteria 2, 3, 7 and 8 are met by the suite. Criterion 1 is met except for
  "through at least one migration", which cannot exist until the schema changes.
  Criteria 4, 5 and 6 are untouched.
- No world-state requirement moved. Nothing here measures identity, placement or
  bearings.

## Next

1. Give the component a caller: record episodes during ordinary running, with no
   authority, which is what criteria 4 and 5 wait on.
2. Measure what a shadow run costs in disk per hour before writing a retention
   policy, because the number decides whether copying frames is affordable at
   all.
3. Have the world state tell the record when it merges or splits a thing. The
   alias table exists and is tested; nothing calls it.
