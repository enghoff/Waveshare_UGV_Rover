# The rover's own map splits into rooms in under a second, but not into the owner's rooms

**Cut at doorway width, the map of 2026-10-15 splits into two to seven regions
depending on how wide a doorway is taken to be, and no width gives the three
rooms the owner named.** Against the eleven things the owner taped on
2026-10-09, each with the room the owner put it in, the kitchen always shares a
region with the living and dining room; the bedroom comes out whole only once a
doorway is taken as 1.1 m wide, and at that width the electrical-panel cover on
the living room's wall falls in the corridor's region. What the map gives is a
draft for the owner to correct and name once, which is how every robot that
splits a home into rooms uses it, not an answer. Nothing ran on the rover.
[R-WS-20](../requirements/world-state.md#r-ws-20) stays proposed.

## What was run

`experiments/rooms/split_rooms.py` on the grid the M5 session of 2026-10-15
saved (`tmp/session3-1015/session3-1015-grid.json`, map `7da19bef3888`, the map
the taped things are on), on this desktop, 0.8 s for three widths. The free
floor is shrunk by half a doorway; what is left falls into pieces, each piece
of at least 0.5 m² seeds a room, and every free cell then takes the seed it
reaches first through free floor. A taped thing takes the room of the nearest
roomed cell, since paintings and wardrobes stand on wall cells. The truth is
the `where` of each thing in
`experiments/perception_bench/ground_truth/m4_targets.json`: five in the living
and dining room, one in the kitchen, five in the bedroom.

| Half-doorway | Regions | Living and dining room (5) | Kitchen (1) | Bedroom (5) |
|---|---|---|---|---|
| 0.35 m | 7 | one region | in the living room's | three regions |
| 0.45 m | 5 | one region | in the living room's | two regions |
| 0.55 m | 4 | four in one, the panel cover in the corridor's | in the living room's | one region |
| 0.65 m | 3 | four in one, the panel cover in the corridor's | in the living room's | one region |
| 0.75 m | 2 | one region | in the living room's | one region; the rest of the flat is one |

Taking only the walls as obstacles (`--walls world_state/visit_model/walls.npz`,
the cells occupied in every session's grid) changed no taped thing's region:
the bed and the wardrobes are in every session's grid, so they count as walls
either way, and they are what cut the bedroom up at narrower widths.

## What it does and does not show

- **The width was read off the answer.** 0.55 m is where the bedroom holds
  together, chosen after seeing the table; it is development, not a test.
  Hydra's rule for choosing a width without the answer, the count of rooms
  that holds over the most widths, has nothing to choose here: the count
  falls 7, 5, 4, 3, 2.
- **The kitchen's join is the failure every published method names.** An
  open-plan room has no doorway for geometry to find (HOV-SG, Hydra and
  Matterport each say so; see
  [the plan](../plans/rooms-and-things-within-things.md)).
- **The corridor comes out as two regions at 0.55 m** where the M5 scoring's
  own hand-drawn rule (`experiments/m5_changes/score_session.py`, `room_of`)
  has one. Nothing taped says whether the second is the bathroom.
- **Eleven things in three rooms is a first look, not a measurement of
  accuracy.**

The picture the script draws (`--png`) is not committed, because the
repository is public and it is a floor plan of the flat.
