# Joining split things is on the rover, waiting for the owner's review

**The rover can now propose which of its things are one object, draw them for review,
join the pairs a person accepts, and put them back.** It was deployed at c42b49d, with
both suites passing on the Orin (world_state 1036, rover_daemon 1096). Asked on the
rover at 13:02, `world_state_merge` answered 18 proposals over the 192 things of map
session 67 in 2.2 s, and `merge_sheet.py` drew them into
`~/.ugv/world/review/merge-20261004-130218-1.jpg` and `-2.jpg`. **Nothing has been
joined yet**: the runs list is empty, and joining waits for the owner to choose which
proposals to accept. [R-WS-13](../requirements/world-state.md#r-ws-13) stays `open`.

How the score was chosen and what it was measured to do is in
[the morning's entry](2026-10-04-one-score-for-appearance-and-position.md): run after the
resolver in replay, it took the look-alike pairs within 0.5 m from 24 to 1 and put no two
labelled objects together. What to type is in
[the runbook](../runbooks/world-state-merge.md).

## What it proposes

Each thing is in at most one proposal, so asking again after joining gives the next
round. A pair is proposed only when its crops on their own favour one object; before
that rule, four pairs were carried by position alone with crops saying, if anything, two
objects. Pairs sharing a picture are never proposed.

Read by the coding agent from the two sheets, as a first opinion for the owner's review:

- **Plainly one object, 14:** the ceiling fan, the pendant lamp, the cow painting, the
  black cabinet twice, the floor lamp, the grey painting in a gold frame, the blue
  picture hung on its corner, the teal door, the armchair, the dark door, the dark
  painting of houses, the green landscape, and the painting of white buildings.
- **Not worth joining either way, 4:** a pair of glare and bare door edges (proposal 4),
  dining chairs that cannot be told apart (10), a blur of the owner and the sofa (14),
  and a blurred mixture of a door edge and a landscape at a score of +0.01 (18).

## What changed besides the call

The store's own merge, which reanchoring also uses, now carries the masked crops across
as well as the plain ones. Before, a thing joined to another kept only its own masked
crops, so the masked-crop check could refuse the other half's next look.
