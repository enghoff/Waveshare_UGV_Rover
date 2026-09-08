# The second drive: the geometry is right and a thing can still be two objects

**M0 does not pass, and identity is the whole of why.** The owner drove the rover
for twenty minutes on the morning of 2026-09-08 through a cleared semantic store,
restarting navigation once during the run. What came back is the best geometry
this rover has recorded and the same identity fault it has had all along, in a
form that finally says where it comes from.

The recording is `~/.ugv/archive/world-2026-09-08-drive.db` on the rover with 272
frames and 186 depth maps beside it, and a copy is in `captures/m0-2026-09-08/`
with the contact sheets the review was read from. 1328 looks from 106 standing
places, one map session, 104 things.

## The geometry is confirmed, by a tape measure

The owner put two objects out, `object:80` and `object:99`, and the rover placed
them 2.899 m apart. **Measured in the room: 2.9 m.** That is the first
confirmation that the corrected OAK mount does real work on a driven run rather
than passing a bench fit, and it tests the whole chain at once — the bearing, the
rover's own pose, and the range that helped place them.

It also says the rover is pessimistic about itself. It claimed 0.263 m and
0.175 m of placement uncertainty on those two, so it would have accepted anything
between about 2.5 and 3.3 m, and the truth landed inside a tenth of a metre.
Over-stating uncertainty is the safe direction and it is not free: 387 of the
1328 looks attached to nothing at all.

Ranges are better again. Against the parallax ground truth — every thing seen
from three or more standing places has its position fixed by crossing bearings
alone, and each range compared against the distance from its own look's viewpoint
to that crossing — **74 of 108 ranges land within half a metre, 69%**, against
66% on [the first acceptance drive](2026-09-07-m0-acceptance-drive.md) and 48% at
[the baseline](2026-09-07-m0-semantic-world-state.md). Median error 0.34 m.

## Three things the rover placed are two objects each

Of the 21 most-looked-at things, reviewed as contact sheets and then confirmed by
putting each doubtful box back on its full frame:

| thing | is | and also | wrong look |
|---|---|---|---|
| `object:8` | a framed landscape painting, 16 looks | a dining chair | 35022 |
| `object:33` | the dark sofa, 30 looks | the person sitting at the desk | 35965 |
| `object:42` | the black cabinet, 36 looks | a small framed picture on the far wall | 35394 |

All three carry enough looks to be eligible to steer the rover, and criterion 3
allows none. The [collapse test](2026-09-07-both-remedies-deployed.md) deployed
the day before did not catch any of them.

## Why the appearance score let a chair join a painting

**Because founding a thing never asks the question that joining one does.**

A look that wants to join a thing that already exists is asked twice: does this
crop resemble the thing, and does that resemblance survive having everything but
the object's own pixels blanked out. The second question is `resolve.collapsed`,
and a drop of 0.20 or more refuses the match. Creating a new thing out of two
looks asks only the first, in `_could_be_one`, and there is nothing else in that
path.

The numbers on the pair that founded `object:8` are as stark as this gets. The
chair and the painting behind it scored **0.701 as the camera framed them and
0.154 with each object's own pixels alone** — the widest gap of any founding pair
in the run, and far outside the 0.20 the joining path already refuses. The pair
crossed at 12.7 degrees off a 0.588 m baseline, both barely over their floors,
and placed the thing to within 0.783 m.

Everything after that follows from it. Once the founding pair holds two objects,
the thing's exemplars hold two objects, and the gate that should catch the next
chair is comparing it against a chair the thing has already swallowed. The
painting is behind a chair in most of its looks, so this compounds rather than
washing out.

Across the whole run, every one of the 104 founding pairs passed the whole-crop
gate — the lowest scored 0.551 against a floor of 0.55 — while **51 of them are
below that same 0.55 once masked**, and 25 would be refused by the drop threshold
that is already deployed on the joining path.

## What the remedy does, and what it does not

Asking the same question with the same threshold at creation introduces no new
number. On this recording it refuses 25 of 104 foundings and catches two of the
three faults; `object:42`'s pair drops 0.144, inside the threshold, and no
principled number separates it without being chosen after seeing it.

**Tried on the recording, and it is not a remedy.** Four replays of the whole
drive, differing only in where the masked question is asked:

| what asks the masked question | things | chair in the painting | person in the sofa |
|---|---|---|---|
| nothing (as deployed) | 120 | merged | separate |
| the founding pair | 119 | merged | **merged** |
| the same-group join | 123 | merged | separate |
| both | 114 | **separate** | **merged** |

It takes both gates together to separate the chair from the painting, and the
same pair of gates turns the sofa into a merge that the deployed build does not
make. One fault fixed and one caused is not an improvement, so **nothing was
deployed and the change is reverted.** The suite stays at 798.

Two things are worth keeping from the attempt. The first is that a refused pair
is not a discarded look: it stays in the pending pool and can found a thing later
with a better partner, which is why the entity count moves so little. The second
is a real asymmetry in the code, found while tracing the chair and left in place
because fixing it alone changed nothing here. After a founding pair places a
thing, every other ray in the same group that points near it joins, and that loop
asks only the plain-crop question. Its own comment records that it once attached
on geometry alone and had the plain gate added afterwards; the masked one was
never added. `object:8` took three of its looks that way.

**And the replay is a weak instrument for this fault.** It reproduces one of the
three merges the rover made and produces the other two only under changes, which
is the recording and the live rover disagreeing. Where they disagree the rover is
right, so a remedy chosen on replay alone would be chosen on the wrong evidence.

A refused pair is not a discarded look. It stays in the pending pool and can
found a thing later with a better partner, which is why the entity count barely
moves.

## The other two criteria this run touched

**Coverage is now reported rather than silent.** 687 of 1328 looks — more than
half — could not be ranged because the region sat outside the depth camera's
view, with 45 more where the camera did not answer and 18 where nothing in the
box could be measured. 38 of the 104 things were never ranged at all. That is
what criterion 10 asked to be counted, and it is a large enough number to be an
operating limit rather than a footnote.

**99 looks were given no direction at all**, their pictures kept, because the
rover's place on the map was not confirmed when they were taken. That is
[R-WS-16](../requirements/world-state.md#r-ws-16)'s behaviour, observed across a
navigation restart on hardware for the first time.

## The spray can, which is not a fault

A blue spray can on the floor was detected twice and never became a thing, and
the resolver is right. The two looks are four seconds apart from standing places
0.37 m apart, with 1.1 degrees between their bearings, against floors of 0.4 m
and 12 degrees. There is no crossing there to find.

What the run does show is that **a measured distance cannot found a thing.** Both
looks carried a range, 1.077 m and 0.585 m, and a range with a bearing places an
object from a single viewpoint. The creation path only ever crosses two bearings,
so the distance is recorded, shown in the console, and then not used. A target
the rover can range but cannot place is worth naming.

## Requirements

- [R-WS-13](../requirements/world-state.md#r-ws-13) stays `open`, with three
  fresh named instances and a mechanism: the collapse test guards joining a thing
  and not founding one.
- [R-WS-10](../requirements/world-state.md#r-ws-10) stays `failing`. The 2.9 m
  separation is real evidence and it is one measurement of the whole chain, not
  the held-out bearing trials the criterion asks for.
- [R-WS-16](../requirements/world-state.md#r-ws-16) stays `open`; its hardware
  demonstration exists now and wants writing up against the restart itself.
- [R-WS-12](../requirements/world-state.md#r-ws-12) is untouched: no bare patch
  turned up in this review either.
- M0 criteria 3 and 8 fail. Criterion 2 is partly met — the ranges are good and
  the refusals are counted — and criterion 7 is not settled by one separation.
- Counts: world_state 798 passed, 0 failed.

## Next

1. **Label the merges on this recording before trying another remedy.** Three
   faults found by eye is what defeated this attempt: a change that fixes one and
   causes another cannot be judged against a sample of three. The contact sheets
   for all 104 things are already built; what is missing is a verdict written
   down against each one, which is a review rather than an experiment.
2. Let a measured range place a thing from one viewpoint. The spray can is the
   case, and 38 things that were never ranged are the scale.
3. Narrow the recorded placement uncertainty toward what the drive measured.
4. Count splits, which nothing reports the way a merge is reported.

Not worth doing: another pass at where to ask the masked question. The four
replays above cover the three places it can go, and the deciding evidence is a
labelled recording rather than a fourth arrangement of the same gates.
