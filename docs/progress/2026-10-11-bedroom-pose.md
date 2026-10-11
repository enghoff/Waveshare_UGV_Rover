# The bedroom pose of 2026-10-11: a quarter metre from the wheels, doubled by the mapper

**Entering the bedroom at 14:18 the rover's pose went a quarter of a metre out
over the last three metres, and the mapper's own correction 35 s later doubled
that, to half a metre and 10 degrees; throughout, one scan against the map put
the rover where it really was.** Measured from the recording by matching each
scan against the morning's map. Why the mapper's correction went the wrong way
is not shown: the graph it had at 14:18 has been saved over, and a second
mapper on today's graph needs about 850 MB of the 1.7 GB the rover has free.

## How it was measured

Each scan of `m5-scene1-141241`, laid at the pose navigation recorded, was
matched by brute force against the occupancy map fetched at 08:04 (±0.7 m in
5 cm steps, ±20 degrees in 1-degree steps, scored by distance to the nearest
wall). Standing still the best match is unambiguous; driving along the corridor
it is not, and there it jumped 0.7 m along the corridor between neighbouring
scans, so the corridor tells nothing along its own length.

| When | Navigation believed | The scan says | Off |
|---|---|---|---|
| 14:17:44, the living room, still | (-17.30, -15.16) | (-17.30, -15.11) | 0.05 m |
| 14:18:03, half way down the corridor | (-11.42, -13.23) | (-11.32, -13.23) | 0.10 m, 3 deg |
| 14:18:06, at the bedroom door | (-10.16, -12.85) | (-10.01, -12.75) | 0.18 m, 3 deg |
| 14:18:11-44, parked in the bedroom after its turn | (-8.78, -11.63, 138 deg) | (-8.83, -11.38, 141 deg) | 0.25 m, 3 deg |
| 14:18:45 on, after the mapper's correction | (-8.67, -11.83, 131 deg) | (-8.82, -11.38, 141 deg) | 0.47 m, 10 deg |

Laid at the believed pose the parked scans scored 0.31 against the map; at the
scan's own best pose, 0.84. So a check of one scan against the map, from where
the rover stood, could have said it was out and by how much. The change watch's
false reports of that minute came from the parked half minute (refused since
71d8414).

The map at 14:36 shows the bedroom's walls within a cell of the morning's, with
no second set of walls a quarter or half a metre off, so the scans at the wrong
pose were not folded into the map in any amount that shows.

## Still open

- Why the mapper moved the pose the wrong way at 14:18:45. The session script
  now keeps the saved map at each session's start (`experiments/m5_changes/session.py`),
  so the next time it happens it can be replayed into the mapper from the graph
  it really had.
- Why the wheels were a quarter metre short over the doorway and the turn: the
  threshold, the bedroom floor or the turn itself. A drive through that doorway
  on the shakedown's loop, measured the same way, would say which.
