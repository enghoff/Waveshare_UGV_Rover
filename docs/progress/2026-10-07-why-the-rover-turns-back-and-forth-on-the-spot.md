# Why the rover turns back and forth on the spot

**A goal close by that must be reached facing a very different way is planned
as a loop several metres long, and the controller will not drive the loop.** It
turns on the spot instead, the next plan changes a little, and it turns back.
Nothing ends it: Nav2's progress check counts every 20-degree swing as
progress. Of 189 drives under 1 m since the M3 sessions began, 15 took more
than 10 s, 402 s in all. The worst was in M3 session 7 today: 72 s swinging
between 143 and 167 degrees, 10 cm from where it started, until the owner asked
for it to be stopped. Navigation now drives a near goal as a turn, a straight
line and a turn, and ends any drive that goes nowhere. 587 checks pass here;
**not yet seen on the rover.**

## Reproduced on the rover, standing still

All of this was asked of the live stack with the rover parked where it had
stopped, (-17.50, -14.61), and nothing moved.

- **The plan.** The goal was (-17.20, -15.06), 0.4 to 0.5 m away, facing -70
  degrees. The live planner returned a route 3.3 to 4.0 m long from every start
  heading between 130 and 173 degrees: off the way the rover faced, then round
  in a loop. The bridge's own result had said the same: "the route round was 4.0
  m for a goal 0.4 m away".
- **The controller.** `dwb_bench.py` ran the real controller beside the live one
  on that route. Turning on the spot scored 45.2 and driving forward 66.9 (lowest
  wins), because the loop sets off away from the goal and the goal-distance and
  goal-heading critics punish that. The bench was given a goal heading for this
  (`--goal x,y,heading`).
- **Why nothing stopped it.** The navigation log for those 72 s holds a new plan
  every second and nothing else: no failure and no recovery.
  `PoseProgressChecker` takes 10 cm or 20 degrees in 15 s as progress, and the
  swing was 24 degrees.
- **What plans straight.** Facing the goal, the same goal planned as 0.48 m.
  That held only facing within about 30 degrees of it (-73 to -28 against a
  bearing of -58), and only for a final heading within about 30 degrees of the
  way it travelled: 11.5 degrees off planned straight, 40 off looped.

This is the same swing frontier.Stall was written for on 2026-09-01 ("fifty
seconds, six centimetres, forty-three replans"). That fix was kept to exploring,
on the grounds that a person who chose a place is owed every recovery.

## The change

In `ros_nav/nav_moves.py`:

- **A goal under 1.5 m with a heading is a turn, a straight line and a turn.**
  Narrowed the same day at the owner's question: a console click carries no
  heading and had not been seen to swing, so it is left to Nav2 as before. The
  rover turns to face the goal, checking once more and correcting, because turns
  are counted by a gyro measured 7-9% out. It then drives to it with the heading
  set to the way it is travelling. If the heading asked for is more than 15
  degrees off, it turns to it after arriving. Where it cannot turn, it sends the
  one goal it always sent.
- **Every goal carries `frontier.Stall`.** 25 s without getting 0.5 m further on,
  with Nav2 attempting nothing, ends the drive and says why. Recoveries are left
  alone, as for exploring.

A test replays session 7's goal against Nav2 replaced by the measured rule. It
fails 10 checks on the old code (the drive swings until it times out) and passes
on the new. ros_nav 587 passed.

## Still to see

The first drives after deployment, on the rover. In particular: whether a turn
of 150 degrees lands within 20 of the goal, given the over-turn measured on
2026-09-04, and whether a near goal's three moves cost more time than they save
on goals that never swung.
