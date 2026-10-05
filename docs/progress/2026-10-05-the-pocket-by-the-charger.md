# Every frontier a run chose was behind a gap the rover does not fit through

**All seven frontier drives autonomous runs had ever chosen failed, and for one
reason.** Each was in a pocket beside the charger that the lidar sees into
through a 30-40 cm gap. The executive judged what it could reach by walking the
map as a point, while the planner keeps the rover's centre 20 cm from anything
solid, so a gap under 40 cm is closed to it. Twice the planner found a 38 m way
round instead, and the rover set off along it before giving up. The executive
now walks the map with the rover's body. Deployed at 4789e6d (autonomy and
ros_nav) and on main as c0c780c. On the rover the pocket's frontiers are now
refused, and frontiers across the house are still offered. No run has shown it
yet. [R-AUT-13](../requirements/autonomy.md#r-aut-13) stays open.

## What happened

On 2026-10-03, three of these drives ended a run: routes of 37 to 38 m for goals
4.4 to 5.0 m away, each 42 to 52 s of recoveries
([that day's entry](2026-10-03-a-run-stalled-on-a-place-it-could-not-reach.md)).
That day's fix refuses a failed place for half an hour. It does not cover a
pocket: session 2's frontier was 1.5 m from the first. In
[session 2](2026-10-05-m3-session-2.md) the planner found a 37.7 m route to a
goal 3.9 m away, sent the rover 2.9 m in the wrong direction, failed to plan
again, and gave up after 20 s.

## The cause

The executive's walk over the map crossed any free cell. Nav2's costmap marks
every cell within 0.20 m of a wall as contact for the rover's centre. Measured
on the live costmap, its ring reaches exactly 0.200 m and no cell beyond
0.206 m is in it. So the walk called the pocket about 5 m away, while the planner
either had no way in or, on some updates of the map, a way round through
unmapped ground. The bridge's own exploring asks the planner before every
frontier. The executive had no such check.

## The fix

The bridge now sends two numbers with the map: the clearance, read off the
costmap node, and the 0.5 m by which it will move a goal onto floor where the
body fits. The executive walks only free cells further than the clearance from
anything solid. A goal counts if it is within the 0.5 m of that floor. A map
without the numbers is walked as a point, as before. A frontier the body cannot
reach is still listed, refused as unreachable, and no longer takes one of the
eight places. On session 2's map, six of the eight nearest frontiers were in
the pocket.

## Evidence

- **Replayed over all 210 autonomous drives in the rover's record:** all 183
  that arrived are still reachable. 10 of the 27 that failed are not: all 7
  frontiers, and 3 looks with nowhere for the body near their spot. The other 17
  are not route problems a walk over the map can see. Five were the bridge's
  stricter whole-body test at the goal, four were safe-area refusals, three
  started beside a wall before the back-off existed, two timed out, and three
  gave no reason.
- **The clearance has to be the costmap's.** At 15 cm all five "no route" drives
  get through, and at the 19.3 cm of a twelve-sided stand-in for the body, one
  does. From 20 to 30 cm, no drive that arrived is lost.
- **Tests:** the two recorded decisions are a fixture. The test fails on the
  previous code (frontiers reachable at 5.3 and 5.55 m) and passes now. autonomy
  761 and ros_nav 574 passed here. On the rover, autonomy's suite passed in the
  deploy and ros_nav's passed 606 with the real message types.
- **On the rover:** `nav_grid` answers with a clearance of 0.2 m and a goal
  allowance of 0.5 m. One deliberation (episode 779) refused all nine pocket
  frontiers, which a point would have reached in 3.2 to 5.5 m. It kept six
  across the house at 10 to 15 m and weighed 57 candidates in 1.4 s.
  Navigation restarted for the deploy and came back settled.

## What it means

A frontier goal can now only send the rover where the planner can take it.
Session 3 is the first run that can show this. The next largest cause of failed
drives is the bridge's stricter whole-body test at the goal, which the walk does
not copy; this change does not address it.
