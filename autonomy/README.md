# Episodic memory

This component records what the rover decided, what it did about it, and what it
was looking at when it decided. It is the evidence trail that has to exist before
anything is allowed to choose where the rover drives, and it is
[Phase 1](../docs/plans/autonomous-curiosity.md) of the curiosity plan.

**It has no authority over anything, and that is structural rather than
careful.** Everything here reaches the rover through `client.ReadOnly`, which
holds a list of the reads it may make and raises on anything else — so a call
that would move the rover is not merely unused, it is unavailable. The test that
proves it does so by trying every one.

Nothing *decides* anything yet. The executive that will is Phase 2 work, so the
episodes recorded today contain no decision and no call, and a replay reports
them as "decided nothing", which is the honest reading.

## What an episode is

One occasion of the rover doing something. It opens with a trigger, collects
whatever the occasion produced — the goals considered, the choice and why, the
calls made and what came back, what changed in the world state — and closes with
an outcome. `abandoned` is a result and not a failure: it is what a look that
attached to nothing closes with, and what a decision taken with no authority to
act will close with.

Today two things trigger one: the rover taking a look, and the rover moving.
Both are recorded by the shadow run below, which watches and decides nothing.

One the rover really recorded, read back off the Orin:

```text
episode:210 -- opened 2026-09-08 09:59:52, triggered by the rover looked
  world state f49e9206997fbe42, map session 67
  recorded by a shadow run; the rover decided nothing here
  decided nothing
  called nothing
  a look found 11 regions, 0 of them attached to a thing the rover already knows, 0 with a measured distance
  closed: abandoned -- every region in this look is still unattached, which is the ordinary state until two bearings cross
  1 piece of evidence kept, 31 kB
```

When there is an executive, the same shape carries the goals it considered, the
choice and why, and the calls it made — and `abandoned` will then also mean a
decision taken with no authority to act on it.

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

## The shadow run

    ssh orin 'cd ~/ugv/autonomy && python3 recorder.py --seconds 1800'

`recorder.py` watches the rover work and writes down what happened. It is not a
service: nothing starts it at boot, and it is run by hand for as long as somebody
wants a recording.

**Looks come from the world state's numbered history, not from watching for
events.** The daemon has no event stream, so something has to poll, and polling
for *state* would drop whatever happened between two polls — a recording with
silent holes in it reads exactly like a rover that did nothing. So the recorder
remembers the last observation it recorded and walks back from the newest until
it meets it. Nothing between polls can be missed, and a recorder that was stopped
for an hour catches up when it starts again.

**Moves come from the driving loop's own running commentary.** There is no
history of moves to walk, but the loop keeps the last thirty-two sentences it
said and hands back everything said since a sequence number the caller names — so
the recorder names the last one it recorded and gets the ones in between. That
matters because a replan lasts about a fifth of a second and is the one phase of
a move worth knowing about; nothing polling could catch it, and this does. One
move is one episode with a step per sentence, closed on the sentence saying how
it ended, using the navigator's own word for it. Only a gap long enough to
overrun those thirty-two sentences loses anything, and what it lost is counted
and reported rather than left as an absence a reader would take for a rover
sitting still.

`--no-frames` records the looks without copying their pictures, which is how the
cost of keeping them gets measured.

## Runtime

Data lives outside the deploy tree, so that a deploy replaces the code and never
the recording:

```text
~/.ugv/autonomy/episodes.db
~/.ugv/autonomy/evidence/<first two characters>/<digest>
```

`UGV_AUTONOMY_DIR` overrides it, and the only things that ever override it are
the test suite and `recorder.py --dir`.

## Keeping it off the disk

A look costs a copied frame and a rover left switched on looks all day, so
`retention.py` removes evidence in three passes, in this order:

1. **A pinned episode's evidence is never touched.** An acceptance recording is
   what this is for; a full disk with a loud reason is a better outcome than a
   quietly deleted recording somebody was arguing from. `store.pin` and
   `store.unpin` are both rows, so "pinned in September, released in October" is
   answerable.
2. **Anything past the age limit goes**, so that a rover switched off for a month
   does not come back and delete a week of recent looks because the total is
   over.
3. **Then oldest-first until the store is under its size limit.**

Every removal goes through `store.delete_evidence`, so an episode whose pictures
have gone reports itself as no longer fully replayable, with the reason, instead
of being summarised as though it could still be checked. `retention.would_fill`
measures the growth rate from what is actually in the store rather than from an
assumed frame size. The policy's numbers live in `retention.DEFAULT` and nowhere
else.

## The modules

| File | What it holds |
|---|---|
| [`refs.py`](refs.py) | the names: minting, parsing, and whether one may be looked up |
| [`client.py`](client.py) | the only way to the rover, and the calls it refuses |
| [`recorder.py`](recorder.py) | the shadow run: watch, write down, decide nothing |
| [`retention.py`](retention.py) | what is removed when the record grows, and what never is |
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
