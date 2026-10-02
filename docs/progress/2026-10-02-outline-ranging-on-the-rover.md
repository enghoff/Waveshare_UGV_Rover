# Ranging from the outline is on the rover, and today's world was rebuilt with it

**The rover now reads each region's distance under its own outline, and the world it
held was read again and rebuilt that way.** Deployed as `e0445da` (world_state and
rover_daemon; on the Orin, world_state 993 and rover_daemon 1050 pass). A look taken
afterwards with the depth camera awake ranged 5 of its 8 regions, all five from their
outlines, kept each outline beside its row, and kept the depth map they were read from.
`world_state_rebuild` then read the session's 112 kept looks again and built its things
again from the looks, oldest first. Things placed more than 0.3 m behind a wall fell from
3 to 1. [R-WS-10](../requirements/world-state.md#r-ws-10) stays `failing`.

The method and its replay are in [the outline entry](2026-10-02-ranging-from-the-outline.md);
the call is in [the runbook](../runbooks/world-state-rebuild.md).

## The rebuild

The dry run read 675 regions of 112 looks again and drew 667 outlines with the rover's own
region finder. Every one matched its stored row. 192 ranges would move by more than 2 cm:
270 now come from outlines and 38 from boxes. Applied, the store was copied to
`~/.ugv/world/world.db.before-rebuild-1790961975`, those ranges were written, and the 48
things of map session 67 were deleted and built again from 384 looks in 38.5 s, 66 s in
all. A rehearsal on a copy of the store taken earlier in the day had given the same counts.

Judged against the lidar map, as wall-mounted things should stand on its walls:

| | Things | From one look | Outside the mapped room | More than 0.3 m behind a wall |
|---|---|---|---|---|
| Before | 48 | 10 | 11 | 3: the mirror-fronted wardrobe 0.55 m, two others 0.40 m |
| After | 48 | 6 | 11 | 1: the bedroom wardrobe, 0.34 m |

Most of the 11 things outside the mapped room on either side sit within 0.3 m of a wall.
They are wall-mounted things placed just past the wall line. The one left behind a wall is
the wardrobe whose depth comes from the strip at the depth map's left edge, where the stereo
reads it at 3.2 m. That is the known failure in the outline entry, and the box read it
right only by luck.

## What this does not show

- **Accuracy against the tape on the rover.** The taped objects have not been driven past
  since the change. The replayed figures are the evidence until they are.
- **The depth camera's left edge.** Both methods read a wrong distance there, and nothing
  discounts that strip yet.
- **What the rebuild cost in identities.** Every thing has a new number, so anything that
  held an old one, such as an autonomy record or a console selection, refers to nothing now.

## Requirements

None moved. [R-WS-10](../requirements/world-state.md#r-ws-10) stays `failing`.
