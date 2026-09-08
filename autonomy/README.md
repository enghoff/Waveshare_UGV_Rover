# Episodic memory

This component records what the rover decided, what it did about it, and what it
was looking at when it decided. It is the evidence trail that has to exist before
anything is allowed to choose where the rover drives, and it is
[Phase 1](../docs/plans/autonomous-curiosity.md) of the curiosity plan.

**It has no authority over anything.** There is no path from here to the driver
board, the gimbal or the navigator. Nothing on the rover writes to it yet either:
the executive that will is Phase 2 work, and until then this is a library with a
test suite.

## What an episode is

One occasion of the rover deciding something. It opens with a trigger, collects
the goals that were considered, the choice and why, the calls that were made and
what came back, what changed in the world state, and closes with an outcome. A
shadow run — deciding and having no authority to act — closes `abandoned`, which
is a result and not a failure.

```text
episode:1 -- opened 2026-09-08 14:31:02, triggered by nothing_to_do
  world state 9f2a1c04ffab3d21, map session 7
  considered 2 goals, chose look_at(object:8): one look is a bearing and not a position
  asked alibaba/qwen-omni-realtime for phrasing what it was about to do
  called look_at(entity=object:8, pan_deg=-20.0) -- ok
  closed: abandoned -- shadow mode: no movement authority
  2 pieces of evidence kept, 51 bytes
```

## The one hard problem: a name that still means something next month

The world state underneath an episode does not hold still. Clearing it resets the
identifier counters, so the next `object:8` is a different object; a merge deletes
the losing entity outright; a thing that turns out to be two objects gets taken
apart. An episode that resolved a stored name against today's store would produce
a confident, wrong account of what the rover did — which is worse than no account,
because nothing about it looks broken.

Three mechanisms, and [`refs.py`](refs.py) is where they are written down.

**A name carries the store that minted it.** A world reference is
`ws/<generation>/object:8`, where the generation is a token the world state mints
when its database is created and mints again when it is cleared. A reference
whose generation is not the live one cannot be looked up at all — it fails closed
rather than pointing at a stranger. The token lives in the world state's `meta`
table; see `WorldStore.generation`, and `world_generation` in its summary.

**A decision keeps a copy of what it was made from.** Every decision records a
snapshot of the world it saw, and a replay reads that snapshot and never the live
store. This is what makes a reconstruction independent of everything that happens
afterwards, and it is why recording a decision against a live dictionary is
refused rather than allowed.

**Evidence is named by what is in it.** A frame is `sha256:` of its own bytes,
copied out of the world state's frames directory before a clear can empty it.
Globally unique with no namespace to negotiate, survives every clear, and twenty
episodes that looked at the same picture hold one copy between them.

Alongside those, what the world state did to identity afterwards is recorded
beside the episode rather than inside it: a merge is one alias row, a thing taken
apart into three is three rows, and a reader is told both what the episode said
and what that thing is called now.

## Nothing already written is ever changed

There is no UPDATE, DELETE or REPLACE in [`store.py`](store.py) — a test records a
whole episode with SQLite reporting every statement and checks that not one of
them changes a row. Two consequences worth knowing:

- **An episode does not carry its own outcome.** Closing appends a `closed`
  event, and the outcome is read back off it, so the row written when the episode
  opened still reads exactly as it did.
- **Deleting evidence does not touch the evidence row.** The bytes go and a row
  goes into `deletions` saying when and why, so a replay says *the owner deleted
  this on the twelfth* rather than finding nothing.

A correction is a later event naming the earlier one it corrects; both are kept,
and a reader is shown both. An annotation added after the episode closed is
marked as such, so hindsight cannot be mistaken for what was known at the time.

## Runtime

Nothing runs this on the rover yet. When something does, its data belongs outside
the deploy tree, in the place the store already defaults to:

```text
~/.ugv/autonomy/episodes.db
~/.ugv/autonomy/evidence/<first two characters>/<digest>
```

`UGV_AUTONOMY_DIR` overrides it, and the only thing that ever overrides it is the
test suite.

## The modules

| File | What it holds |
|---|---|
| [`refs.py`](refs.py) | the names: minting, parsing, and whether one may be looked up |
| [`schema.py`](schema.py) | the tables, and why there is no mutable one |
| [`store.py`](store.py) | episodes, events, snapshots, evidence, deletions and aliases |
| [`events.py`](events.py) | what an episode may say, and the fields each kind carries |
| [`replay.py`](replay.py) | rebuilding an episode from the record, and nothing else |
| [`summary.py`](summary.py) | the few lines a person or a model reads |
| [`selftest.py`](selftest.py) | the checks; `python autonomy/selftest.py` |

## What is not built

Named because a plan that quietly absorbs a description of the thing it built
leaves two accounts of the running system with one of them maintained. Ahead of
this, from [Milestone M1](../docs/plans/autonomous-curiosity.md):

- **Nothing records an episode on the rover.** Criterion 4 wants a thirty-minute
  shadow run recording real navigation and world-state events, and that needs a
  caller this component does not have.
- **Retention and disk limits.** Neither implemented nor documented, and evidence
  copied per look is the thing that will fill a rover's disk. Criterion 6.
- **A real migration.** The schema has shipped once, so there is nothing to
  migrate through yet; the suite exercises the mechanism against a database built
  to be one column short, which is the closest honest thing to criterion 1's
  "through at least one migration".
- **Nothing tells this component about a merge.** `store.alias` exists and is
  tested, and the world state does not call it, so identity changes are recorded
  only when something puts them there.
