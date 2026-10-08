# A person with no way round: Nav2 spun the rover in front of them; now it waits, but it will squeeze past at the wall margin

**With the planner's live layer on, the owner stood in the 1.0 m mouth of a
corridor that was the only way to the rover's goal, and Nav2, finding no route,
ran its recoveries 2 m from them: a 225-degree spin, a wait, a 0.35 m reverse
and a second spin over 30 s, until they stepped aside and it drove through.**
That was predicted in [the live-layer entry](2026-10-08-live-layer.md) and is
now fixed (e715e25): when Nav2 starts recovering and the planner has sent no
route for the goal, the goal is stopped and the rover holds still, asking the
planner itself for 3 s, then drives on or gives the goal back as blocked. On a
second run that worked as far as it went: no route, the rover held still and
said it was waiting. A second later the planner found the 45 cm between the
owner's legs and the wall, and the rover squeezed into it, passing their legs
about 9 cm off its side. Standing that close it counted itself inside an
obstacle, and the back-off for that turned it on the spot beside them. The
owner chose to keep that margin. Afterwards, driving by map clicks, the owner was
told spots were "inside a wall" that were refused for something the scan saw;
the sentence now says so (3e576ee, below).

## The doorway

The corridor runs east from the charger room. On the map it is 1.0 m wide from
x = -15.5 to -13.5, and its mouth, about (-15.4, -14.0), is the only way to
`block_trial.py door`'s goal, (-13.5, -13.65). With 0.25 m of legs across its
middle, a walk of the map at the planner's 0.20 m clearance cannot reach the
goal; 5 cm off the middle, it can. Before the first drive the planner was asked
with the owner standing there: no route with the layer on, and with it off a
route through the middle of them.

## First run, before the fix (2026-10-08, 23:48)

The rover started 2.0 m from the owner. From Nav2's own logs: every plan failed
after about 2.5 s with "no valid path found", each failure cleared the
costmaps, and after the fourth the recoveries began.

| from start | what the rover did |
|---|---|
| 0 to 11 s | stood still while four plans failed |
| 11 s | spin, asked for 90 degrees, turned about 225 on the chassis |
| 14 to 23 s | Nav2's 5 s wait, then still |
| 23 s | reversed 0.35 m |
| 30 s | spun again, about 100 degrees |
| 34 s | the owner stepped aside; a route, and it drove to the mouth |
| 39 to 42 s | the route watch stopped it 0.5 m short of the owner, still close by; it waited 2 s |
| 48 s | arrived |

The route watch could not have helped: it looks 1 m along the route, and the
owner was 2 m away until they moved.

## The fix (e715e25)

Every recovery Nav2 makes is counted in its feedback, and the planner publishes
each route it finds, about once a second while a goal runs. A recovery with no
route from the planner for 2 s, or none at all for this goal, is the planner
failing rather than the controller, which keeps getting routes while it
struggles. The goal is stopped there, before the ladder reaches its spin: on
this run the first such recovery came at 2.2 s and the spin at 11. The rover
then asks the planner itself every second (nothing else is asking it now) for
the 3 s a near goal waits, the owner's choice of 2026-10-07; with a route it
drives on, and without one the goal is handed back as "there is no way past".
Any refusal other than no route goes back to Nav2 as before.

In `test_way_round.py` Nav2's logged timeline was replayed first: the model
rover was spun and reversed, as on the rover. With the fix, it does neither,
waits, and hands the goal back a few seconds later. Someone stepping aside
during the wait is driven on past, and Nav2 recovering from something else,
with the planner still answering, is left alone without the planner being asked.

## Second run, with the fix (2026-10-09, 00:00)

The rover started 2.6 m from the owner, by the charger. From the pose trace,
the recording (`door2-rec.json`) and the costmaps:

- It drove 1.8 m towards the mouth, then stopped where the planner failed
  (6 to 8.6 s) and said "something is in the way and there is no way round it;
  waiting for it to move". No spin and no reverse: the fix, seen on the rover.
- At 9.6 s the planner found a route. The scan held the owner as a cluster
  0.2 to 0.3 m across at about (-15.3, -13.95), just off the middle, which left
  45 cm between their legs and the wall. At the planner's clearance of 0.20 m
  from anything lethal, that is room, and it drove into the gap: centre 0.23 m
  from the nearest leg cell, the side of its body about 9 cm from it.
- There the route watch stopped it for the owner on the route. The next plan
  was refused because the rover's own spot was inside the owner's clearance,
  and the back-off for that turned it 180 degrees on the spot beside them and
  drove 0.4 m back into the room, then a near goal waited 3 s for the way
  and handed the goal back: no way round in straight legs.

The owner did not say whether it touched them. Asked whether to give anything
the map does not have an extra 15 cm of clearance, which would stop the
squeeze but close gaps under about 70 cm between unmapped things, they chose to
keep the present margin.

## A refusal blamed on the map (3e576ee)

Afterwards the owner drove the rover by clicking the map and was told, several
times, "there is nowhere within half a metre of that spot where the rover's
body fits -- it is inside a wall or under something". The spot they clicked is
not logged. Spots are fitted to the body on the planner's costmap, and since the
live layer that has on it whatever the scan sees, including the owner, who was
walking beside the rover: back at the charger at 00:16, 15 lethal cells the map
does not have sat 0.26 to 0.43 m from the rover, and they were gone ten minutes
later, so they were most likely the owner, not the dock. One of the refusals came
when a drive to (-10.26, -16.56), which fits now, was stopped after 2 s and a
point further along its route was then refused: most likely the owner was on
it. That is a judgment from the logs, not a measurement. The sentence now
looks: when the planner's costmap has something lethal within the reach of the
fit that the map does not have, it says the scan sees something there that the
map does not, a person or something moved. Tested in `test_way_round.py`;
ros_nav 642 tests pass. On the rover, with the deployed code against the live
costmap and map: beside clutter the scan sees at (-18.4, -17.2) it answers that
the scan sees something; at a bare corridor wall and on open floor it does not.

## What is still open

- **The squeeze.** The planner's clearance from a person is its clearance from
  a wall, a few centimetres off the body's side, from the legs the scan sees.
  Kept by the owner's choice.
- **The back-off beside a person.** A rover that has come that close counts
  itself inside an obstacle, and `back_off` gets out by the nearest fitting
  spot, which here meant a turn on the spot next to the owner.
- **The wait is 3 s**, the near goals' wait. In a doorway it is the only
  option, and whether it should be longer there has not been asked.

Captures: `captures/2026-10-08-live-layer/` (`block-door-234840.jsonl`,
`door-rec.json`, `block-door-000042.jsonl`, `door2-rec.json`).
