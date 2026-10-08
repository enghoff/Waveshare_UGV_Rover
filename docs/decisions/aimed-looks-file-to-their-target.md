# A look aimed at a thing files the region at the aim to that thing

Status: agreed by the owner on 2026-10-08 ("yes"), for supervised runs, M3
sessions and the first M4 trials. `executive.NAME_THE_TARGET` is the switch.

This is the case [the 2026-10-01 decision](identity-is-judged-action-by-action.md)
asks for before an action relies on identity. The action is the geometry goal's
look ([R-WS-13](../requirements/world-state.md#r-ws-13)).

## What the action does

The executive drives to a viewpoint facing a thing and takes a look. Today the
look's regions are filed like any other, and 5 of 157 recorded aimed looks
reached the thing aimed at ([2026-10-08](../progress/2026-10-08-aimed-looks.md)).
With this change the look names its target, and the world state (`aimed.py`):

1. **Files one region to the target.** It chooses the region that points at the
   target within the resolver's own allowance, with height, range and appearance
   agreeing. A near tie that appearance cannot break files nothing.
2. **Moves the target's position and claim from aimed depth.** Where aimed looks
   ranged the target, their depth gives its position and the uncertainty it
   claims, never below 0.20 m. The tolerance the resolver matches with stays the
   bearings' own.
3. **Remembers same-object suspects.** It records the other records the region
   also fitted as probably the same object. A run cools them off with the target,
   so it does not go back to the same object by way of another record of it.

What it does not do: it does not add the region to the target's appearance
examples. It does not merge records. It does not change how any other look is
filed.

## What a wrong identity costs here

- **A wrong region filed to the target.** It is one wrong look, and it pulls the
  target's placement toward whatever it shows; if it ranged, it moves the
  position directly. It is not made an example, so the target does not then
  accept more wrong looks for it. The table of aimed filings says which look did
  it, so it can be found and put back.
- **An over-confident claim.** The rover would believe a thing is better placed
  than it is, and stop trying to improve it until something changes. Nothing
  drives to a thing because of its claim alone, and no person-facing answer
  changes beyond the number shown.
- **Two different objects suspected of being one.** Both are set aside for 15
  minutes. Nothing is merged.

## The evidence

Offline, on recorded looks; analyst labels, not owner-confirmed:

- **Regions picked.** On the 157 aimed looks the targeted-look audit kept, 16 of
  the 19 picks whose target could be judged from the photographs showed it. The
  3 wrong picks all had target records that were already mixed.
- **Filing through the production code.** Run on the 2026-10-06 store, it filed
  24 regions. 19 of them had been taken by another record and 18 were ranged; 16
  claims fell by more than 2 cm.
- **Claims against the tape.** Among targets the owner taped on 2026-10-03, the
  window-side painting went from 0.47 m off to 0.06 m, claiming 0.20. The green
  landscape painting behind the dining chairs is the hard case: 3 of 6 filings
  claimed less than they were off, by up to 0.35 m. There the depth camera can
  measure the chair in front, and nothing in the reading says so.
- **Placing by depth.** Over 126 eye-matched looks of the six taped targets, this
  rule put placements a median 0.11 m from the tape, against 0.36 m from
  bearings, with the tape inside the claim 68% of the time
  ([2026-10-08](../progress/2026-10-08-depth-placement.md)).

## What is not proven

- **No live look has used it.** The judgments are the analyst's.
- **Things seen over or behind furniture.** Claims there can be over-confident.
  A further check -- whether an aimed range stops short of the wall that a thing
  hung high must be on -- is not built.
- **Same-object suspects are untested on the rover.** They also share the risk
  of any appearance-based pairing: alike objects, such as the dining chairs.

## Extended 2026-10-08: group-mates are set aside as suspects are

After a goal at a record, a run now also sets aside the records the latest
grouping joined with it -- the merge proposer's, then co-fit's
([2026-10-08](../progress/2026-10-08-merging-by-cofit.md)) -- as it does the
look's same-object suspects. Judged under this case, because the action and
its cost are the same: a record wrongly grouped is set aside for 15 minutes,
nothing is merged, and the resolver never reads the groups. Measured: of 25
random co-fit joins, 17 were one object by photograph and 1 was two. The owner
had said on 2026-10-08 that they generally trust the analyst's judgement on
such calls; this extension is theirs to reverse.

## The ask

Agree it for supervised runs, M3 sessions and the first M4 trials. A run's
geometry goals then name their targets, and the session write-up reports every
aimed filing beside its photograph. If it is not agreed, the code stays and no
look names a target.
