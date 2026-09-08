# The rover was driven round the property and the record kept up: M1 passes

**Fifty-nine moves and two hundred and fifteen looks, over thirty-four minutes,
with nothing lost and nothing recorded twice.** The owner drove the rover through
the whole property on the evening of 2026-09-08 while a shadow run watched. That
was the one thing Milestone M1 was still missing, and with it every criterion is
met.

The proof that the recording is complete is not a count but a numbering. The
driving loop numbers every sentence it says, and the record holds **sentences 608
to 1748, all 1141 of them, with no hole anywhere in the sequence**. The looks are
numbered independently and the record holds 426 of them, every inspection number
distinct — so nothing was recorded twice either.

That matters because the recorder was not running for the whole span. It was
stopped and restarted in the middle, and the record came back and carried on from
its mark.

## What a driven episode says

```text
episode:481 -- opened 2026-09-08 12:00:36, triggered by the rover moved
  world state c5f923250f74197f, map session 67
  recorded by a shadow run; the rover was driven by somebody else
  decided nothing
  called nothing
  drive_to(x_m=-17.96, y_m=-18.14), and it went planning x2 -> driving x19 -> ended
    planning: the goal is with Nav2
  2.68 m of route, 39 waypoints, 4 of 22 steps seen as they happened, the rest read back afterwards
  closed: succeeded -- arrived
```

`decided nothing` and `called nothing` are the honest reading: there is no
executive, so the rover was driven by a person and the record only watched.

The fifty-nine moves were 32 `drive_to`, 17 `turn_in_place`, 7 `drive` and 3
`explore`. **Thirty-five arrived cleanly**; the rest are the more interesting
half, and the navigator's own words for them are worth reading:

- Seven arrived only after the goal was moved, by between 5 cm and 40 cm, because
  the spot asked for was too close to something for the rover's body to fit.
- Two were refused outright — *there is nowhere within half a metre of that spot
  where the rover's body fits: it is inside a wall or under something.*
- One was blocked because *the rover is standing inside something the costmap
  believes in*, and another because *it could not even turn towards the one spot
  nearby where its body fits*.
- Two were stopped after Nav2 gave up on 3 and 7 recovery attempts, both on short
  goals the planner had routed 7.0 m round.
- One exploration ended with *there is nothing left on the map worth driving to,
  after 13 places tried*, with 67% of the map still unknown.

None of that is new about the rover, and none of it is this component's business
to fix. What is new is that it is now **written down against the moment it
happened**, with the pose, the battery and the map's state beside it, in a record
that will still be readable when the map it refers to is gone.

## The recorder was down and it cost nothing

Restarting the recorder to pick up a fix, I killed it with a `pkill -f` pattern
that also matched the SSH command carrying it — the exact trap
[the deploy runbook](../runbooks/deploy.md) warns about. It killed itself before
starting the replacement, and nothing recorded for several minutes.

**Nothing was lost, and the honest reason is not that the design saved it.** The
rover was standing still through that whole window: the last move before it ended
at 11:31 and the next began at 11:59. The loop said nothing, so there was nothing
to lose. Had the rover been driving, the loop's memory of thirty-two sentences
would have been overrun and the record would have counted and reported what it
could not recover. The looks would have survived regardless, because their
history is numbered and stays on disk.

## What it costs, corrected

A picture on this drive cost **24 kB**, and the run recorded 215 looks in
thirty-four minutes — about **9 MB an hour while the rover is being driven round
a house**.

That is a useful correction to [this morning's
estimate](2026-09-08-shadow-run.md), which put a driving rover at 104 MB an hour.
That figure assumed the ceiling of one look a second; the daemon's own gate makes
it far rarer, because a look is only worth taking once the rover has moved 15 cm
or turned 25 degrees. Against the 2 GB the retention policy allows, 9 MB an hour
is about nine days of continuous driving rather than twenty hours.

All 274 of tonight's episodes are fully replayable.

## Three worlds in one record

The store was cleared twice today, so the record now holds episodes from three
different fillings of the semantic world: 210 from `f49e9206997fbe42`, one from
`4d2fda72621ef169`, and 274 from `c5f923250f74197f`, which is live. Asked to
resolve the nine things one of this morning's episodes named against the store
that is live now, the record refuses all nine. The episodes still say what the
rover did; they simply will not pretend their names still point anywhere.

## Requirements

- M1 passes. Criteria 1, 2, 3, 5, 6, 7 and 8 were met
  [this morning](2026-09-08-shadow-run.md) and
  [this afternoon](2026-09-08-the-clear-that-proved-it.md); criterion 4 is met
  here, by a thirty-four minute run holding both navigation and world-state
  events with no movement-capable call available to the component.
- [R-AUT-5](../requirements/autonomy.md#r-aut-5) held across a driven run rather
  than a stationary one.
- [R-AUT-1](../requirements/autonomy.md#r-aut-1),
  [R-AUT-2](../requirements/autonomy.md#r-aut-2),
  [R-AUT-3](../requirements/autonomy.md#r-aut-3),
  [R-AUT-4](../requirements/autonomy.md#r-aut-4),
  [R-AUT-6](../requirements/autonomy.md#r-aut-6) and
  [R-AUT-7](../requirements/autonomy.md#r-aut-7) held.
- Nothing about the rover's navigation or perception moved. Every navigator
  verdict quoted above is a record of what it said, not a measurement of whether
  it was right.
- Counts: autonomy 344 passed, 0 failed.

## What M1 does not give

- **Nothing decides anything.** Every episode in the record is an occasion of the
  rover acting, never of it choosing. The executive is Phase 2 and it is the
  first thing that will write a decision into these episodes.
- **Nothing tells the record about a merge or a split.** A retained reference
  cannot be broken or redirected by one, because replay never resolves against
  the live store — but "what became of that thing" is unanswered until something
  calls `store.alias`.
- **Nothing starts the recorder.** It is run by hand, so a rover left alone
  records nothing.
- **A look that found nothing leaves no episode**, because an inspection with no
  region writes no observation for the recorder to key on.
