# The world model kept per visit is pass/fail on honesty alone

Status: agreed 2026-10-16 by the owner. It replaces the conditions
[the plan](../plans/a-world-model-kept-per-visit.md) tested version 1 against
on 2026-10-15, from version 2's test on. No code has changed.

One thing passes or fails the new world model: **it never holds two things
that can be told apart as one.** A record that takes a labelled picture of a
second such thing fails it, by the rule of M5's criterion 4
([counted](m5-sessions-are-counted-before-they-start.md#what-counts-as-identity-kept-criterion-4)),
with every filing of the model counted. Things nothing tells apart, the dining
chairs and the two black armchairs
([agreed](the-two-armchairs-cannot-be-told-apart.md)), are left out. For them
the honest answer is "one of the two", which is
[R-WS-2](../requirements/world-state.md#r-ws-2).

Everything else is measured against the deployed store on the same pictures and
reported without a bar:

- **moved things followed**, by the strict reading: the record holding most of
  the thing afterwards is the one that held most of it before;
- **things that stayed put**: how many records hold each;
- **the share of labelled pictures filed.**

It takes the resolver's place once, run in shadow on the rover beside the
deployed store, it has made no wrong merge, follows clearly more moves than the
deployed store, and is no worse on the other two.

## Why

- **The conditions it replaces were accuracy bars.** Two agreed rules say there
  should be none: how sure identity must be is decided action by action
  ([2026-10-01](identity-is-judged-action-by-action.md)), and P0 measures what
  the hardware does, with only honesty, abstention and safety pass/fail.
- **With three to five moves a session, a bar on moves is decided by one or
  two of them.**
- **Honesty can always be met**, by declining to choose.

## What the history says

Over the five labelled sessions, counting every filing but not a record that
holds both armchairs, and counting moves of distinguishable things only:

| | Records holding two things | Moves followed, strict, of 6 | Labelled pictures filed |
|---|---|---|---|
| deployed store | 0 | 0 | 63-69% |
| version 1 | 2 | 1 | 88-95% |
| version 2 | 1 | 2 | 87-94% |

Version 2's development used all five sessions, so its row flatters it. Its
mixed record holds nine pictures of the snowy-tree painting and two of the
blue painting, on the shakedown. The deployed store stays honest by filing
about a quarter fewer pictures. Neither follows moves.

## How many sessions it takes

- **Following moves.** The deployed store follows almost none, so each move
  the model follows and it does not counts. Five such moves is a difference
  chance gives about one time in thirty. A staged session gives three moves in
  the wait, and more if things carried in before it were seen well earlier. At
  version 2's rate on its own development, one in three, that is three to five
  sessions; if it follows most, two. If it follows none of the first eight
  moves, it is rejected: a model following one in three would do that less
  than one time in twenty.
- **Honesty.** One wrong merge fails the version as it stands. Clean sessions
  prove honesty slowly: a model that mixes two things in one session in five
  passes three sessions half the time, and showing fewer than one in ten takes
  about thirty clean sessions. Those come from the shadow run, where every
  ordinary drive counts once labelled, not from staged sessions.
- **Things that stay put and pictures filed** are already clear on the history,
  and every session measures them again.

## What was considered

- **Keeping the plan's bars.** Declined for the reasons above.
- **A per-picture tolerance**, one wrong look in twenty, as
  [R-WS-18](../requirements/world-state.md#r-ws-18) proposes. Every version
  here would pass it. That tolerance is for the owner to agree with the
  requirement; until then the model is held to the rule the deployed store was
  held to in M5.

## What would reopen it

An action that relies on which thing a record is sets its own evidence under
[R-WS-13](../requirements/world-state.md#r-ws-13); that does not change this
test. A wrong merge that the rover never states, because no answer or action
reads it, would be a reason to ask whether the rule counts the right thing.

Moving [R-WS-17](../requirements/world-state.md#r-ws-17) itself still needs
labels by someone other than the code's author, as the requirement says.
