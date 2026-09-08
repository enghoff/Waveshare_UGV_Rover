# Episodic memory, what the rover would do next, and the loop that does it

This component records what the rover did, works out what would be worth doing
next, writes down what it would have chosen and why it may not, and — under a
permission the daemon issues and can take back — goes and does it. It is
[Phases 1 to 3](../docs/plans/autonomous-curiosity.md) of the curiosity plan.

**Nothing here can move the rover on its own, and that is structural rather than
careful.** There are two doors and neither of them is a movement call. The
recorder and `decide.py` hold `client.ReadOnly`, which is every read and nothing
else. The executive holds `client.Acting`, which adds exactly three: ask the
daemon for permission, hand it back, and do one admitted thing *under* it.
`drive_to` is on neither list, so an autonomous drive is not merely unused here
— it is unavailable, and the test that proves it does so by trying every one.

The authority itself lives in the daemon, as a bounded run a person opens and a
fifteen-second lease this component keeps renewing. Stop the rover and the lease
is gone and cannot be got back from here, because the one call that clears a stop
is a call no client here may make. Kill this process and the daemon takes the
wheels back by itself within the lease. See
[rover_daemon/permission.py](../rover_daemon/permission.py), which is one file
deployed into both components so that what this expects and what the rover
enforces cannot come apart.

So: run `decide.py` and the rover says what it would investigate and moves
nothing. Have somebody open a run, start `executive.py`, and it goes and does it
— one goal at a time, with every metre attributable to one episode.

## What an episode is

One occasion of the rover doing something. It opens with a trigger, collects
whatever the occasion produced — the goals considered, the choice and why, the
calls made and what came back, what changed in the world state — and closes with
an outcome. `abandoned` is a result and not a failure: it is what a look that
attached to nothing closes with, and what a decision taken with no authority to
act will close with.

Today three things trigger one. Two are the rover's own activity — it took a
look, it moved — and those carry no decision, because nobody here decided to do
them. The third is this component deciding: what it would go and do next, what
that would cost, and what it refused. All three come out of the shadow run
below; only the third contains a choice, and even that one closes without
acting.

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

A deliberation uses the same shape and fills in the parts this one leaves
empty: the goals considered with their scores, the choice and why, and, when
the executive carried it out, the calls it made and what came back. A
deliberation that closes `abandoned` is one that chose nothing or was not
allowed to act; one that closes `interrupted` was stopped part-way, and says
by what.

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
cost of keeping them gets measured. `--decide-every 0` turns off the deliberating
below and leaves a run that only watches.

## What it would do next

    ssh orin 'cd ~/ugv/autonomy && python3 decide.py'

Once a minute during a shadow run — or once, on demand, with the command above —
the rover works out what would be worth doing and writes down the answer. On the
Orin against the real house that is **about 0.6 s for twenty-five candidates**,
and it moves nothing.

**Two kinds of goal, which is what the plan enables at this phase.**

`explore_frontier` is somewhere to stand where the scanner would see ground
nobody has mapped. Which gap is worth driving to is not decided here: it is
asked of `ros_nav/frontier.py`, the same module the rover's own `explore` ranks
with, deployed into this component from the one file in the repository so that
the two cannot come to different views of a map. What this component adds is the
estimate of how much new floor standing there would actually reveal — the
frontier's width times how far the scanner is worth believing, capped by the
unmapped ground really within reach of the goal.

`improve_geometry` is somewhere to stand that would make a thing's position come
out better. **A bearing pins a thing across the line of sight and says nothing
along it**, so two looks from nearly the same place leave the same long thin
uncertainty however many times they are taken, and the useful viewpoint is the
one whose ray crosses the error ellipse's long axis. That is the whole of the
arithmetic in `goals._from_viewpoint`, and it is why a look from where the rover
is already standing is often correctly worth nothing. A thing whose distance has
never been measured is the other case: there the gain is not from crossing but
from a range landing on it, which the depth camera can only do from inside the
band the geometry was accepted in.

Nothing is predicted better than **0.10 m**, whatever the arithmetic says. The
model knows about bearings and not about the rover's own pose, the width of the
thing, or the box drawn round it moving between frames — and a tenth of a metre
is the best this rover has ever managed on a tape-measured pair.

**The score, with every term written down:**

    utility = purpose_relevance * gain
              - w_time * time - w_travel * travel - w_energy * energy
              - switching_cost

Each term is dimensionless against a declared scale: 20 m² of new floor and
0.30 m off a placement are each worth half a unit, a minute and ten metres are
the scale of one errand. The gain saturates rather than capping, so two badly
placed things do not come out identical, and idle is worth zero — a candidate
that costs more than it gains loses to standing still. The energy term is kept
and weighted at nothing, because this chassis has no current sense and the
estimate is time in different units.

**Refusals are not terms.** A *gate* is about the rover — a flat or unreadable
battery, a latched stop, a pose it does not trust, a map that has not settled, a
perception loop that has stopped, and the standing fact that nothing here may
move anything. A *veto* is about one candidate — nowhere to walk to, a viewpoint
outside 0.5 to 2.5 m, a goal outside a configured safe area, or a thing that is
cooling off. Both run before the scoring, so a purpose weight of a thousand
still buys nothing.

**A thing that keeps taking looks and coming out no better is put aside.**
[`cooling.py`](cooling.py) notices that between two deliberations, and the entry
lapses either after fifteen minutes or the moment the placement really improves.
It is decided from the two readings rather than remembered, so it survives the
recorder being stopped and started, and it is part of the inputs a decision is
snapshotted with.

## Going and doing it

    ssh orin 'cd ~/ugv/autonomy && python3 executive.py'

The loop that carries out what the deliberation chose. It attaches to a run
somebody has already opened, and if there is none it says so and exits — it
cannot open one, which is the whole architecture in one sentence.

One turn of it is one episode, and the deliberation above is that episode's
first half:

```text
IDLE -> SELECT -> PLAN -> EXECUTE -> EVALUATE -> IDLE
                            |          |
                            +-> ABORT <-+
```

**IDLE** renews the permission and reads the rover. **SELECT** is the same
`scoring.consider` a shadow run records, differing only in that the answer can
now be acted on. **PLAN** turns the goal into a short list of admitted
operations — a frontier goal is a drive, a geometry goal is a drive and then a
look — and checks every one of them against the daemon's own list before the
first is dispatched. **EXECUTE** sends them one at a time and waits for the ones
that are not over when the call returns, renewing the permission as it polls.
**EVALUATE** reads the rover again and records what actually changed rather than
what was hoped for. **ABORT** is any of that going wrong, and closes the episode
`interrupted` with the reason.

**The rover's own frontier `explore` is deliberately not used**, even for a
frontier goal. Two reasons, and the second is the one that would matter anyway:
a run that chooses its own next goal produces movement this episode cannot
attribute, and
[R-NAV-6](../docs/requirements/navigation.md#r-nav-6) is failing — a rover
ringed by unmapped floor retires the whole rim on arriving without having moved.
So the executive drives to one frontier viewpoint at a time and decides again
when it gets there.

**There is no model in this loop**, and that is not an omission. Nothing here
asks anything to be curious on its behalf, so a model outage cannot start a
physical action, cannot stop one and cannot change what is chosen. The check for
that is written against the record rather than the code: an episode may only say
a model answered by carrying a `model` event, and a completed autonomous turn
has none.

**What ends a run.** Its budget — minutes, metres, actions, failures in a row,
the battery floor — or a person stopping the rover, or the daemon noticing that
nothing has renewed the lease. The first two end it from outside this loop
entirely; the third is what happens if this process is killed mid-drive.

## How a decision reads

```text
would improve_geometry:object:19@-17.57,-18.55: object:19 is placed to 2.79 m,
seen 5 times; a look from 2.5 m crossing at 21 degrees would bring that to
about 0.28 m
  expects a look from (-17.57, -18.55), 2.5 m from the thing, crossing the
    present uncertainty at 21 degrees
  costs 1.3 m and about 14 s; gain 0.8932 (2.51 m taken off where the thing is,
    counted against the 0.30 m tolerance the acceptance run declared)
  score 0.7856 = 1.0*0.8932 - time 0.2286 - travel 0.13 - switching 0.0
  improve_geometry:object:13@nowhere scored higher at 0.8668 and was refused --
    unreachable: there is no route to it over floor the map calls free
  cannot act: no movement authority -- nothing in this component can move the
    rover: every call it may make is a read
  25 candidates, 6 vetoed, in 0.61 s
```

The refused line is the half worth reading. Everybody can see what the rover
wanted; the question a week later is why it did not do the obvious thing, and
that is only answerable because every candidate is recorded with its score and
its refusals, whether it won or lost.

## What the choosing is judged against

    python autonomy/scenarios.py

Running the scorer on the rover shows what it does and cannot show whether that
was right, because the rover has no opinion. So there are **48 curated
scenarios** in [`scenarios/`](scenarios), each a room drawn as a picture with a
few things in it and the expected outcome written beside it:

```text
##################          #  wall      .  floor
#................#          ?  unseen    R  the rover
#.......R........#
############.....#
```

They are scored as if the rover had authority, because that is the only way to
test the refusals — today's gate is always shut, and every case would come out
the same. The milestone asks for the expected ordering in at least 95% of them.

**When the code and a scenario disagree, one of them is wrong and it is not
always the code.** Two of the first run's disagreements were the generator's
fault, and both were real: viewpoints were being cut to the sixty nearest the
rover before the certified band was applied, which left a thing across the room
with no usable viewpoint at all; and the goal wanted last time was recognised by
an identifier that changes whenever the map is redrawn, which lost the
hysteresis exactly when it was needed.

## The weights, and the purpose nobody has declared

Every number in the score is configuration, not code: `scoring.DEFAULTS` holds
the built-in set with the reason for each, and `~/.ugv/autonomy/scoring.json`
overrides any of them on a particular rover. The whole configuration is written
into each decision rather than named by version, because the file lives on the
rover where nothing versions it.

**`purpose_relevance` is 1.0 for both goal types, and that is a stated position
rather than a missing feature.** The design's purpose term is the owner's
policy — "learn where household objects tend to be", "understand every doorway
and route" — and inventing one on their behalf would put a preference nobody
holds into every decision the rover records. Equal weight says the rover has
been told nothing about what it is for. Telling it is a line in that file.

## Runtime

Data lives outside the deploy tree, so that a deploy replaces the code and never
the recording:

```text
~/.ugv/autonomy/episodes.db
~/.ugv/autonomy/evidence/<first two characters>/<digest>
```

`UGV_AUTONOMY_DIR` overrides it, and the only things that ever override it are
the test suite and `recorder.py --dir`.

**Stopping it disturbs nothing.** The record is downstream of everything and
upstream of nothing: killing the recorder outright, deleting its database, or
removing the component leaves the world state, the map and the rover's own place
in it exactly as they were. That is what makes it safe to run beside work that
matters, and safe to throw away a recording that has gone wrong. Killed with
`SIGKILL` mid-poll on the Orin, every world-state and navigation field read
identical either side and the record reopened with its place in the history
unmoved.

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
| [`client.py`](client.py) | the two ways to the rover, and everything both of them refuse |
| [`recorder.py`](recorder.py) | the shadow run: watch, write down, decide, act on nothing |
| [`situation.py`](situation.py) | one reading of the rover: everything a decision may look at |
| [`mapgrid.py`](mapgrid.py) | the occupancy map, and the rover's own frontier chooser reading it |
| [`goals.py`](goals.py) | what could usefully be done next, and what each would cost |
| [`scoring.py`](scoring.py) | what each is worth, what refuses it, and which one wins |
| [`cooling.py`](cooling.py) | what is not worth looking at again just now, and when that lapses |
| [`decide.py`](decide.py) | one deliberation, recorded; `python3 decide.py` says what it would do |
| [`executive.py`](executive.py) | the loop that carries one out, under a permit the daemon can take back |
| [`scenarios.py`](scenarios.py) | rooms drawn on paper with the expected answer beside each |
| [`retention.py`](retention.py) | what is removed when the record grows, and what never is |
| [`schema.py`](schema.py) | the tables, and why there is no mutable one |
| [`store.py`](store.py) | episodes, events, snapshots, evidence, deletions and aliases |
| [`events.py`](events.py) | what an episode may say, and the fields each kind carries |
| [`replay.py`](replay.py) | rebuilding an episode from the record, and nothing else |
| [`summary.py`](summary.py) | the few lines a person or a model reads |
| [`selftest.py`](selftest.py) | the checks; `python autonomy/selftest.py` |

## What is not built

Named because a plan that quietly absorbs a description of the thing it built
leaves two accounts of the running system with one of them maintained.

- **Nothing has driven yet.** The executive is built and every way it can go
  wrong is checked against a fake rover holding the real permission rules, and
  it has still never moved this rover: M3 asks for twenty supervised sessions in
  a pre-cleared room and none of them has happened. Until they do, what is
  proven is the logic and not the rover.
- **The executive is not a service either.** A person opens a run and starts it;
  nothing starts it at boot, and a run cannot outlive the person who opened it by
  more than its budget.
- **The recorder is not a service.** It is run by hand for as long as somebody
  wants a recording. Nothing starts it at boot, so a rover left alone records
  nothing and decides nothing, and retention is not run on a schedule either.
- **Four of the six goal types do not exist.** Inspecting a semantic gap,
  revisiting something stale, investigating a change and searching for something
  missing all need the temporal and claim semantics of M4 and M5. What is here
  is the geometric half.
- **A thing seen once and never placed is never proposed.** One bearing is a
  direction and not a position, so there is nowhere to plan a viewpoint around;
  what such a thing needs is for the rover to be somewhere else, which is what
  exploring does. It means the pool of unplaced sightings — a third of this
  rover's observations — is invisible to the choosing.
- **Line of sight is only as good as the lidar's plane.** A viewpoint is refused
  when the map has a wall between it and the thing, and the map is built at
  20 cm off the floor: a table top, a sofa back or anything else the scanner
  passes under hides nothing here and everything in the picture. So a viewpoint
  that gets past this check is one worth trying, not one shown to work.
- **The unmapped floor a frontier would reveal is an upper bound**, with no
  visibility test: ground behind the wall the rover would be standing against is
  counted. What keeps it honest is that it is used to cap the frontier's own
  width-times-depth estimate rather than on its own.
- **A real migration.** The schema has shipped once, so there is nothing to
  migrate through yet; the suite exercises the mechanism against a database
  built to be one column short.
- **Nothing tells this component about a merge.** `store.alias` exists and is
  tested, and the world state does not call it, so identity changes are
  recorded only when something puts them there.
- **A look that found nothing leaves no episode.** An inspection that found no
  region writes no observation, so "the rover looked and saw nothing" — which is
  itself worth knowing — is invisible to the recorder. Closing that needs the
  daemon to expose its inspection log, which it does not today.
