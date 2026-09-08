# Thirty minutes of watching the rover, and the daemon restarted in the middle of it

**The record works against a real rover, and it survived something nobody
planned.** A shadow run watched the Orin for thirty minutes on the afternoon of
2026-09-08 and recorded 210 looks as 210 episodes, 630 events and 210 pictures.
Sixteen of its 889 polls found nothing listening on the daemon's port, because
the daemon restarted underneath it — and the recording carried on from where it
was, with nothing recorded twice and nothing lost. That was not a test anybody
set up; it is the best evidence in the run.

What is *not* here: no episode contains a decision, because nothing decides
anything yet, and no episode is a move, because nobody drove.

## What was recorded

Of the 210 looks, 200 were the rover's own recorded history, picked up when the
recorder caught up on starting. The other ten arrived while it watched: five
because I turned the gimbal to provoke them, and five from the rover's own
cadence, which is one look every five minutes when nothing has moved far enough
to be worth another. That gate lives in the daemon and is a good one — a
stationary rover looking at the same wall every second would fill the store with
observations that can never be triangulated against each other.

The practical consequence is that **an idle rover is nearly silent**, and a
thirty-minute shadow run over one records almost nothing live. A run that means
anything needs the rover to be doing something.

A picture costs 30.3 kB, measured across all 210. That gives two rates worth
holding on to:

- **An idle rover: about 0.6 MB an hour**, from twenty looks an hour.
- **A driving rover, at the ceiling of one look a second: about 104 MB an hour.**
  Against the 2 GB the policy allows, that is roughly twenty hours of continuous
  driving, or weeks of sitting still. The Orin has 827 GB free, so the limit is
  set by what is reasonable to keep rather than by what fits.

## The daemon went away and the recording did not

Sixteen consecutive polls got "connection refused" at about 09:43. The daemon's
own process had been up nineteen minutes when the run ended against a supervisor
up for two and three quarter hours, so it was restarted mid-run — most likely by
the world-state deploy going out beside this work.

This is the case the design was built for and it is worth being explicit about
why it worked. The recorder keeps its place in the world state's *numbered*
history rather than watching for events, so "what happened while I was not
looking" is a query rather than a loss. When the daemon came back, the next poll
walked back from the newest observation until it met the mark, and recorded
exactly the looks in between.

## Retention, applied to the real record

The ten episodes recorded live were pinned as the acceptance recording. A limit
of 4 MB was then applied to the store, which held 6.37 MB:

- **73 pieces of evidence removed, 2.18 MB freed**, oldest first.
- **All ten pinned episodes untouched**, and all ten still fully replayable.
- A pruned episode now reads, in its own words: *the evidence was deleted on
  2026-09-08 10:05:30: retention: the record was over 4.0 MB — this episode can
  no longer be replayed in full*, while still saying what the look found.

That last line is the point of the exercise. An episode whose pictures have gone
does not go quiet and does not get summarised as though it could still be
checked; it says what it did and says it can no longer show you.

Two things were wrong when this was first run against the rover, and both were
the same kind of wrong — a number stated confidently that was not true. The
growth-rate function divided the whole store by the span between its oldest and
newest evidence, which after a catch-up is a few seconds, and reported 14 GB an
hour. It refuses now unless the span is at least six minutes, and says how to ask
properly. And the reason written onto a pruned piece of evidence formatted every
limit as gigabytes, so a 4 MB limit came back to a reader as "the record was over
0.0 GB".

## The rover was not disturbed

Before and after the run, the map is `3d9b689a1298`, still settled and still
kept, and the map session is still 67. The world state's generation is unchanged
at `f49e9206997fbe42`. Its observations went from 1462 to 1594 and its things
from 109 to 128, which is the rover's own looking and the other agent's
single-viewpoint placements landing — nothing was reduced, cleared or rewritten.
The recorder cannot do otherwise: every call it may make is named in a list and
every one of them is a read.

## Requirements

- [R-AUT-4](../requirements/autonomy.md#r-aut-4) to `settled` — replay imports
  nothing that could reach the rover, checked on the module, and episodes were
  reconstructed on the Orin with the daemon running beside them.
- [R-AUT-5](../requirements/autonomy.md#r-aut-5) to `settled` — the component
  has no movement-capable call available to it, refused at the client before a
  socket is opened, across a thirty-minute run on the rover.
- [R-AUT-6](../requirements/autonomy.md#r-aut-6) to `settled` — bounded by age
  and size, pinned episodes spared, the recorder applying the policy itself as it
  records rather than leaving it to somebody's memory.
- [R-AUT-1](../requirements/autonomy.md#r-aut-1) to
  [R-AUT-3](../requirements/autonomy.md#r-aut-3) held, now against a real
  recording rather than only a suite.
- M1 criteria 5 and 6 met; criterion 4 is **half met** — the world-state half of
  a shadow run is demonstrated and the navigation half is not, because nothing
  drove.
- No world-state or navigation requirement moved. Nothing here measures the
  rover's perception or its driving.
- Counts: autonomy 314 passed, 0 failed.

## Next

1. **A driven shadow run**, which is the only thing standing between M1's
   criterion 4 and being met. The move recording is built and tested offline
   against the navigator's own vocabulary, and it has never seen a real move.
2. **A look that found nothing leaves no episode**, because the recorder keys an
   episode to an inspection's observations and an inspection that found no region
   writes none. "The rover looked and saw nothing" is worth knowing and is
   currently invisible; it needs the daemon to expose its inspection log.
3. **Nothing starts the recorder.** It is run by hand, so a rover left alone
   records nothing at all.
