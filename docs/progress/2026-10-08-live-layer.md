# The planner sees a person in the way: a 4.4 m goal drove round the owner without stopping

**With a live layer on the planner's costmap, the rover drove the blocking
trials' 4.4 m leg round the owner standing still on its line, in one go: 3.9 m
in 11 s, its centre passing about half a metre from them, no stop and no
turning on the spot.** In the [blocking trials](2026-10-08-blocking-trials.md)
earlier that evening the same leg failed twice, because the planner could not
see the owner and the rover had to find its own way round in blind straight
legs, the last of which drifted into them. Before the drive, with the rover
still, Nav2's own planner was asked for the route with the layer on and off:
on, it went round the owner with 0.6 m to spare; off, it passed 15 cm from
their feet. Their marks were gone as soon as they stepped away. One gap the
layer opened, the route watch taking a person for a wall, was reproduced in the
model and fixed (50fa8e3); a person standing where there is no way round has
not been tried.

## What was built

`LiveObstacleLayer` (`ros_nav/behaviors`, fd74a1a) is a Nav2 costmap layer on
the planner's costmap only. At every update, now twice a second, it lays the
latest scan within 3 m onto the grid using the map transform as it is at that
moment, and keeps nothing: what it marked last time is inside the area the
costmap resets before the next update, so a person who has gone leaves nothing
behind. Scan points within 10 cm of a cell the map already has as lethal are
left alone, so it does not thicken walls. The ghost walls that once had the
obstacle layer removed came from marks kept in a frame the map later moved
under; with nothing kept, they cannot build up.

It can be taken back out two ways. At run time the bridge's
`{"op": "live_layer", "enabled": false}` switches it off, and its marks go at
the next update. For good, it comes out of `plugins` in `config/nav2.yaml`.

## Standing still, nothing moving (23:15 to 23:18)

`experiments/m3_trials/plan_check.py` asks the running planner for a route and
reads the costmap without moving anything. The rover stood at (-16.97, -15.72)
facing up the leg and the goal was the leg's end, (-17.05, -11.60).

| | route | furthest from the straight line | layer's own lethal cells |
|---|---|---|---|
| nobody there, layer on | 4.70 m | 0.66 m | 6, clutter behind the rover |
| nobody there, layer off | 4.70 m, the same | 0.66 m | 0 |
| owner 1.6 m up the leg, layer on | 4.82 m | 0.82 m | 5 on the owner, 1 clutter |
| owner there, layer off | 4.70 m | 0.66 m | 0 |
| owner stepped away, layer on | 4.70 m, back to the first | 0.66 m | clutter only |

The owner showed as five cells across 25 cm, the front of their legs. With the
layer on, the route swung out to pass 0.6 m from them; off, it ran 15 cm from
them, as if they were not there. Switching the layer off took its marks away,
and with it on, the owner stepping away took theirs. The route the planner
draws with nobody there already bends 0.66 m off the straight line, round
mapped furniture, and the layer left that alone.

## The drive (23:21:50)

`block_trial.py far`, the owner standing in the same spot before it set off,
recorded with `nav_record.py` (`far2-rec.json`). From the pose trace: it set
off at once, curved out to x = -17.86, passed the owner with its centre 0.51 m
from the middle of their marked cells at the closest, came back to the line and
arrived 3.9 m up the leg after 11 s. It never stopped or turned on the spot.
All eleven routes the planner sent during the drive went round the owner, the
first before the rover had moved, so it was the planner steering and not a
fallback.

The owner had earlier asked whether the large clearance of the blind way round
would keep the rover out of tight spaces. The planner here kept its route about
0.6 m from the owner's marks. That margin is not a rule about people: the
costmap spreads a cost 0.45 m out from anything lethal, walls included, and the
planner keeps out of that cost where there is room and accepts it where there
is not, which is how it gets through doorways. Whether it goes past a person in
a gap as tight as one it would take between two walls has not been tried.

## The route watch took a person for a wall (fixed, 50fa8e3)

Longer goals check the next metre of their route on the live scan every second
([M4 session 4](2026-10-08-m4-session-4.md)), and leave alone anything the
planner's costmap also has, which was meant to mean a door frame the route
passes close to. With the live layer, a person is on that costmap too. Where
there is a way round that does not matter, because the planner takes it. Where
there is none, the planner finds no route, Nav2 keeps following the last one,
through the person, and the watch would leave it to Nav2's recoveries, which
include a quarter turn on the spot. In `test_way_round.py` the case failed
first: the person was taken for a wall and the model rover turned on the spot
for the whole goal. The watch now asks slam_toolbox's map what a wall is, read
the way the planner's static layer reads it. On the rover's map that is 5,884
wall cells, every one also lethal on the planner's costmap, converted in 7 ms
once per map. ros_nav: 635 tests pass.

Replaying the drive's 178 local costmaps through the fixed watch with the
routes the planner actually sent, it would have stopped the rover 0 times. With
the route replaced by the straight line through the owner, it would have
stopped 3 times, so the replay is able to see a stop when there is one.

## What is still open

- **A person standing where there is no way round**, in a doorway. The planner
  then has no route at all, and Nav2's default behaviour tree answers that
  with its recoveries: clear the costmaps, a quarter turn on the spot, a 5 s
  wait, a short reverse. Those can start while the person is still more than
  the watch's one metre away. This is a prediction from the behaviour tree,
  not something seen, and it wants a trial in a real doorway before anything is
  changed.
- **A person stepping in close**, inside a metre of the route, is still left to
  the route watch and the near goal's blind straight legs, which drifted in the
  earlier trials.
- Near goals, under 1.5 m, are still driven by the straight-line code, not the
  planner, so the layer does nothing for them.

Captures: `captures/2026-10-08-live-layer/` (the three planner answers, the
drive trace `block-far-232150.jsonl`, and `far2-rec.json`).
