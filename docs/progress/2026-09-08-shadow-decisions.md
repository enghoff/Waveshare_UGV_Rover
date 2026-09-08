# The rover can say what it would go and look at, and why it may not: M2 passes

**Twenty-five decisions in twenty-five minutes, against the real house, and not
one call to the rover.** A shadow run on the evening of 2026-09-08 worked out
what the rover would do next once a minute while it stood still: 25
deliberations, 25 candidates weighed in each, 625 candidates written down with
their scores, and zero calls of any kind — the component that produced them can
only read. Every criterion of Milestone M2 is met.

It wanted the same thing all twenty-five times, which is the honest result for a
parked rover in an unchanging room, and it is worth reading:

```text
would improve_geometry:object:19@-17.57,-18.55: object:19 is placed to 2.79 m,
seen 5 times; a look from 2.5 m crossing at 21 degrees would bring that to
about 0.28 m
  costs 1.3 m and about 14 s; gain 0.8932
  score 0.7856 = 1.0*0.8932 - time 0.2286 - travel 0.13 - switching 0.0
  2 scored higher and were refused for the same reason -- unreachable: there is
    no route to it over floor the map calls free
    they are object:13, object:97
  cannot act: no movement authority -- nothing in this component can move the
    rover: every call it may make is a read
  25 candidates, 5 vetoed, in 0.63 s
```

## What the rover's own map said that no drawn room could

**Five of the twelve worst-placed things it holds have nowhere to look at them
from.** Of the 25 candidates in each deliberation, 8 were places where the map
stops and 17 were things worth a second look — and five of those seventeen came
back saying every place within the certified band is either unreachable or has a
wall in the way, sixty places tried and sixty blocked. Those five things are
placed in ground the mapper has never confirmed is floor: behind furniture, in a
doorway it has only seen through, or simply wrong.

That is a finding about the world state and not about the choosing, and it would
have stayed invisible if the generator had quietly skipped them. It does not:
each comes back as a refused candidate with the reason, which is what
[R-AUT-10](../requirements/autonomy.md#r-aut-10) asks for.

**A deliberation costs 0.63 s** on the Orin — 0.60 to 0.65 across the run —
which reads the whole entity listing and the occupancy map, walks the map once
for reachability, and ranks 25 candidates. At once a minute that is a 1% duty
cycle beside a rover that looks every second.

## Every criterion

| M2 asks | How it stands |
|---|---|
| 1. deterministic under replay | the snapshot an episode names is re-read, re-scored with the weights it recorded, and produces the same order and the same numbers; checked in the suite |
| 2. full score decomposition per candidate | every one of the 625 carries gain, purpose relevance, time, travel, energy, switching, the total and the weights version |
| 3. gain weights cannot override a veto | a refused candidate scored at a purpose weight of a thousand is still refused and still not chosen |
| 4. 40+ curated scenarios over the listed cases | 49, in four files: what it explores, what it goes back to, what refuses it, and how it chooses |
| 5. expected ordering right in 95%+ | 49 of 49, with two first-run disagreements resolved against the code — see below |
| 6. a rover shadow run emitting decisions, zero movement calls | 25 deliberations over 25 minutes, 0 calls; the clock was removed from this criterion by the owner on 2026-09-08 |
| 7. one concise record: what, why, cost, and why not the better option | the block above, printed live by the run and read back out of the record afterwards |

## The two the drawn rooms got wrong, and the code was wrong both times

The acceptance set exists to be argued with, and the rule is that a disagreement
is reviewed rather than edited away. Two of the first run's disagreements were
real defects:

**Viewpoints were being cut before the band was applied.** The ring of places to
stand comes back nearest-walk-first and was truncated to sixty; for a thing
across the room, all sixty nearest places are three and four metres away, which
is outside the 0.5-to-2.5 m band the geometry was accepted in — so every
viewpoint that would have worked was thrown away before anything looked at it,
and a badly placed thing four metres off came back as "nowhere to stand". The
band is applied first now.

**The goal wanted last time was recognised by a name that moves.** Hysteresis
matched the previous goal by identifier, and an identifier carries the viewpoint
the generator picked — which changes every time the map is redrawn by a
centimetre. So the rover paid a switching cost to carry on with what it was
already doing. It matches by what the goal is *about* now: the same thing
wherever the rover stands for it, and for a frontier, the same place within a
metre.

A third came from the rover rather than the scenarios: the first threshold for
"the perception loop has stopped" was three minutes, and a parked rover falls
back to looking every five, so the gate condemned a perfectly healthy rover four
minutes after it stopped. Ten minutes now.

## What it cost to record, and the mistake in the middle of it

**A situation snapshot came out at 97 kB, and 96 kB of it was a listing of 123
things and a map that had not changed.** Content addressing was supposed to make
an unchanged world free; it did not, because the battery moves by a hundredth of
a volt and the clock moves at all, so every reading hashed differently. Twenty
five minutes of deliberating wrote 2.5 MB of near-identical rows — and nothing
prunes snapshots, because there is no DELETE anywhere in this store. A rover
left deliberating for a day would have added a quarter of a gigabyte, which is
precisely what [R-AUT-6](../requirements/autonomy.md#r-aut-6) says the record
may not do.

Stored in two rows now: the things and the map in one, everything volatile in
another that names it. Five deliberations over an unchanged world write six rows
rather than five copies.

## What this does not give

- **Nothing acts.** Every deliberation closes `abandoned` for the same reason:
  there is no executive and no authority. M3 is the movement permission, the
  stop latch and the supervised trials, and M3 is still gated on M0, which has
  not passed.
- **A parked rover decides the same thing every minute**, correctly. This run
  shows the choosing works against the real world state and the real map; it
  does not show it choosing *differently* as the room changes, because the room
  did not change. A run taken while somebody drives would.
- **Four of the six goal types do not exist**, and they are the semantic ones.
  What is here is the geometric half: where the map stops, and where to stand to
  place a thing better.
- **The purpose is undeclared.** Every goal type is weighted 1.0, which says the
  rover has been told nothing about what it is for. It is a line in
  `~/.ugv/autonomy/scoring.json` when the owner wants to say.
- **Nothing measures whether a goal would have paid off.** Predicted gain
  against realised gain needs something to act first.

## Where it is

Deployed to the Orin at `a8a14a8c4221`. 550 offline checks and 49 of 49
scenarios pass; the run above is the hardware evidence. `python3 decide.py` on
the rover prints what it would do now, `recorder.py` decides once a minute while
it records, and `review.py --decisions` reads the decisions back.
