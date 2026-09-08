<!-- requirement-area: AUT -->

# Autonomy and the record of it

What must be true of the rover's own account of what it decided, before anything
it decides is allowed to move it. The conventions for these records are in
[README.md](README.md).

These exist because of the order the
[curiosity plan](../plans/autonomous-curiosity.md) is built in: the evidence
trail comes before the executive that writes to it, so that no autonomous action
is ever taken that cannot afterwards be reconstructed.

<a id="r-aut-1"></a>
### R-AUT-1 — A record of what the rover decided is never changed after it is written

- **State:** settled
- **Evidence:** [autonomy/README.md](../../autonomy/README.md);
  `autonomy/selftest.py` records a whole episode with SQLite reporting every
  statement it issues, and checks that not one changes or removes a row

Corrections and annotations are later records naming the earlier one they
correct, and both are kept. What would make this false is any path that fills a
field in afterwards — an outcome written back onto the episode row, a flag set on
a piece of evidence — because then the record of what was believed at the time
would be gone and nothing would say so.

<a id="r-aut-2"></a>
### R-AUT-2 — A name recorded from the world state cannot be resolved against a different filling of it

- **State:** settled
- **Evidence:** [autonomy/README.md](../../autonomy/README.md) and
  [the decision behind it](../decisions/episode-references-survive-the-world-state.md);
  `autonomy/selftest.py` and `world_state/selftest.py`; and on the rover in
  [2026-09-08](../progress/2026-09-08-the-clear-that-proved-it.md), where the
  semantic world was really cleared and an episode naming nine things resolved
  none of them

Clearing the semantic world restarts the identifier counters, so `object:8`
before a clear and `object:8` after it are different objects wearing one name.
Every reference an episode stores carries the generation token of the store that
minted it, and a reference from any other generation may not be looked up at all.
What would make this false is a stored name that resolves without the generation
being compared — the failure is silent, because the lookup succeeds and returns
a stranger.

<a id="r-aut-3"></a>
### R-AUT-3 — Evidence that has gone is reported as gone, and never replaced

- **State:** settled
- **Evidence:** [autonomy/README.md](../../autonomy/README.md);
  `autonomy/selftest.py`

Evidence the owner deleted reads as deleted, with the reason they gave and when;
evidence that was never recorded reads as absent; evidence recorded and since
missing from disk reads as absent and says the file went missing. An episode
whose evidence has gone is reported as no longer fully replayable rather than
summarised as though it could still be checked. What would make this false is
anything that substitutes a newer record sharing a local identifier.

<a id="r-aut-4"></a>
### R-AUT-4 — Replaying a recorded episode issues no command to the rover

- **State:** settled
- **Evidence:** [autonomy/README.md](../../autonomy/README.md);
  `autonomy/selftest.py` reads the replay module and checks that it imports
  nothing but the record and the standard library; episodes reconstructed on the
  Orin on 2026-09-08 with the daemon running beside them

The reconstruction reads the episode record and the snapshot the decision was
made from, and there is no client, socket or daemon call anywhere in that path.
What would make it false is an import that reaches the rover, added for one
convenient lookup — which is why the check is on the imports rather than on the
behaviour of a particular replay.

<a id="r-aut-5"></a>
### R-AUT-5 — The component that records episodes has no path that can move the rover

- **State:** settled
- **Evidence:** [autonomy/README.md](../../autonomy/README.md); every call this
  component may make is named in `client.ALLOWED` and every other is refused
  before a socket is opened, checked by trying each one a recorder might reach
  for; the thirty-minute shadow run on the Orin of
  [2026-09-08](../progress/2026-09-08-shadow-run.md), and
  [the driven run](../progress/2026-09-08-the-driven-run.md) the same evening,
  through which the rover was driven round the whole property while the component
  watched and could not have moved it

Distinct from [R-AUT-4](#r-aut-4), which is about replay alone. This is about the
component as a whole: it observes and must have no movement-capable call
available to it. The refusal is a property of the client rather than of the
caller's restraint, so what would make this false is a call being added to the
allow-list — and the list is short enough to read. Movement authority arrives at
M3 and is gated separately in [safety](safety.md).

<a id="r-aut-6"></a>
### R-AUT-6 — Recording episodes cannot fill the rover's disk

- **State:** settled
- **Evidence:** [autonomy/README.md](../../autonomy/README.md); the policy and
  its numbers are in `retention.DEFAULT`; measured and demonstrated on the rover
  in [2026-09-08](../progress/2026-09-08-shadow-run.md), where a limit applied to
  the real record removed the oldest evidence, spared every pinned episode, and
  left the pruned episodes reporting themselves as no longer fully replayable

Each look an episode keeps means a copy of a frame, and a rover left running is
the case that matters. The record is bounded by age and by size, oldest removed
first, and a pinned episode is never touched even by a store far over its limit.

**The recorder applies the policy itself**, as it records, which is what makes
this a property of the component rather than of somebody remembering to run a
second program: the record only grows while something is recording. What would
make it false is evidence kept by a path that does not go through the recorder,
or a pinned set large enough to exceed the limit on its own — which is reported
loudly rather than resolved by deleting the pins.

<a id="r-aut-7"></a>
### R-AUT-7 — The episode record can be stopped or lost without disturbing the rover

- **State:** settled
- **Evidence:** [autonomy/README.md](../../autonomy/README.md); on the Orin on
  2026-09-08 a running recorder was killed with `SIGKILL` mid-poll, and every
  field of the world state and of navigation — generation, map, map session,
  settled, kept, position trusted, and the observation and entity counts — was
  identical either side, while the record itself reopened intact with its place
  in the history unmoved

The record is downstream of everything and upstream of nothing. Its process
dying, its database being deleted, or the whole component being removed must
leave the rover exactly as it was — which is what makes it safe to run beside
work that matters, and what makes it safe to throw away a recording that has
gone wrong. What would make this false is any write to a component other than
its own store, and the only path out of here is a client that can nothing but
read.

<a id="r-aut-8"></a>
### R-AUT-8 — A decision can be re-ranked from the record alone, and comes out the same

- **State:** settled
- **Evidence:** [autonomy/README.md](../../autonomy/README.md);
  `autonomy/selftest.py` re-reads the snapshot an episode names, runs the scorer
  over it again with the weights that episode recorded, and checks that the same
  candidates come back in the same order with the same numbers

Everything a choice is made from is read once, written into the record whole,
and never asked for again: the things with their placements, the occupancy map,
the pose, the battery, what was cooling and what was wanted last time. The
weights go in too, in full rather than as a version string, because the file
they came from lives on the rover where nothing versions it. What would make
this false is a generator or a scorer that reads anything outside the situation
it was handed — the rover, the clock, or a configuration file — and the failure
is silent, because the decision still looks reproducible until somebody tries.

<a id="r-aut-9"></a>
### R-AUT-9 — A refusal cannot be outscored

- **State:** settled
- **Evidence:** [autonomy/README.md](../../autonomy/README.md);
  `autonomy/selftest.py` scores a refused candidate with the purpose weight set
  to a thousand, and checks that it is still refused and still not chosen; the
  curated scenarios carry the same case

Refusals are of two kinds and both run before the score rather than inside it. A
**gate** is about the rover — a flat or unreadable battery, a latched stop, a
pose it does not trust, a map that has not settled, a perception loop that has
stopped, and the standing fact that nothing in this component may move anything.
A **veto** is about one candidate — nowhere to walk to, a viewpoint outside the
0.5 to 2.5 m band the geometry was accepted in, a goal outside a configured safe
area, or a thing put aside for getting nowhere. What would make this false is a
constraint expressed as a large negative term in the utility, because a term can
always be outweighed and nobody notices which weight did it.

<a id="r-aut-10"></a>
### R-AUT-10 — Why the rover did not do the better-looking thing is in the record

- **State:** settled
- **Evidence:** [autonomy/README.md](../../autonomy/README.md);
  `autonomy/selftest.py`; and on the rover in
  [2026-09-08](../progress/2026-09-08-shadow-decisions.md), where six of
  twenty-five candidates were refused and each refusal is named beside the goal
  it refused

Every candidate is recorded whether it won or lost, with its full score
decomposition, its estimated travel and time, and the refusals against it. A
decision names what it would have cost and lists the candidates that scored
better than the one chosen with the reason each was refused. What would make
this false is a record of the winner alone: "why did it not go and look at the
thing in the hall" is the question a shadow run exists to answer, and a rover
that only writes down what it wanted cannot answer it.
