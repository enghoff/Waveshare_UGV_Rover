# The rover locked up pivoting in a doorway, and the fix that looked right, wasn't

A Nav2 failure on this rover, recorded in [`ros_nav/`](../ros_nav). Not the
zig-zag of a route the chassis cannot follow — a harder failure, in passages
around a metre wide, where the rover stops advancing at all and pivots left
and right for up to ninety seconds before anything intervenes. Two fixes were
tried and deployed before the real cause was found, and both are worth keeping
because of *how* they were wrong, not just that they were.

**The first fix was `PathAlign`/`GoalAlign.forward_point_distance`, 0.1 to 0.8 m,
and the evidence for it was worthless.** Four of the seven scoring critics read
the costmap at the point a candidate *ends*, and a pivot ends in the same 5 cm
cell it started in, so all sixteen pivot samples score identically and the
rover cannot tell left from right. The align critics are supposed to break that
tie by reading a point ahead of the nose instead, and at 0.1 m that point sweeps
about two grid cells across the whole turn range — too coarse to resolve
anything. Widening it to 0.8 m was tested by driving an offline copy of DWB
round the rover's own recorded costmap and plan, held fixed, and counting how
often it escaped: 0 of 24 at 0.1, 24 of 24 at 0.8. It shipped on that.

The test was measuring a stalemate, not a fix. The pose a run starts from is, by
construction, one the rover reached *under the settings it was running* — a
point where those settings had already deadlocked. Change any scoring number and
the deadlock breaks, and the model wanders off across a map that is no longer
allowed to move or replan, which looks exactly like escaping. Run on the next
recording, made after 0.8 was deployed and the rover locked up again in the same
doorway, it said the reverse: 0 of 14 at 0.8, 14 of 14 at 0.1. The test condemns
whatever the rover happens to be running, every time, and would have done the
same to the next number tried. Both look-aheads are back at Nav2's stock 0.1.
[`dwb_replay.py`](../ros_nav/dwb_replay.py)'s `closed_loop` now says so in its
own docstring, in words meant to stop this happening a third time.

**The second fix was real, and still wasn't enough on its own.** Nav2 already
carries a recovery for exactly this — `BackUp` — and the stock behaviour tree
puts it fourth in a round robin that resets to its first child every time the
recovery subtree finishes. So the recovery the rover actually got, over and
over, was "clear the costmap and try again", which changes nothing about where
its body is. [`navigate_to_pose.xml`](../ros_nav/config/navigate_to_pose.xml)
moves a 30 cm `BackUp` into `FollowPath`'s own context recovery, gated on the
three failures where being somewhere else is the answer. Checked against the
true footprint on the recorded costmaps, the reverse was clear with room to
spare at every locked-up moment in two separate recordings, and afterwards the
controller had a legal forward candidate again every time. Sent to the rover
directly, outside of any goal, it moved 0.39 m in 1.2 s.

It still wasn't the fix. A recording made after this deployed shows the rover
pivoting for 55 seconds before `BackUp` finally fired — because the progress
checker was not calling it stuck. `required_movement_angle` had been set to 20
degrees for the *other* lock-up, a rover turning to line up on its next leg,
so that would not be mistaken for one that was jammed. A rover swinging 25 to
130 degrees every few seconds resets that same checker's baseline just as
happily, and can pivot for a minute and a half without it firing once. Replayed
over real recorded pose traces — the real algorithm against real poses, no model
involved — 20 degrees let one drive go 72 seconds before it was ever called
stuck; 60 degrees calls it at 15, and is still comfortably clear of every real
turn this rover makes (its slowest pivot alone clears 60 degrees in five
seconds). It is `1.05` rad now.

**What finally answered it was asking DWB, not modelling it.** `publish_evaluation`
is one parameter, and it turns `FollowPath` into something that reports every
candidate it scored, each critic's contribution, and which one it refused —
whether or not anyone is listening. [`nav_record.py`](../ros_nav/nav_record.py)
now subscribes to `/evaluation` and saves it with the rest of a drive, so a
recording carries the controller's own working rather than a replay's
reconstruction of it. Two things this session's own offline model had gotten
wrong twice were settled outright once real numbers existed: it does not model
the oscillation critic's memory faithfully (`dwb_replay.py`'s `LATCH_NOTE` says
by how much and why), and the frozen-map trap above.

Over 617 real ticks of one pivoting drive: a legal forward candidate existed on
every single one, and lost to a pivot on all of them — never once scoring
better, by a median of 1.21 points. The reason is the plan it was following, not
the costmap: the route's first 1.2 m bends 44 to 67 degrees, real curvature
rather than the grid's staircase, and the tightest turn one DWB rollout can make
at full speed is about 36 degrees. Every forward option leaves the line faster
than standing still does, so standing still always wins. And the pivots then
tie — the same four critics that cannot tell one pivot from another cannot tell
this pivot from that one either — so the iterator's first candidate wins the
tie on about half of them, which is the slowest turn sampled. That is where the
loop closes: the slowest turns DWB can ask for, 3.0 and 8.9 deg/s, are both
below what this chassis can hold standing still, so `drive_mixer` rounds them up
to 12 — DWB plans 2.4 degrees of turn over its rollout and gets 9.6. The rover
overshoots the heading it was correcting toward, needs to swing back, and on
about a third of the ticks checked, `Oscillation` had just forbidden turning
that way.

`FollowPath.min_speed_xy: 0.1` and `min_speed_theta: 0.21` (12 deg/s, the mixer's
own floor) delete the two pivot samples the wheels cannot actually produce, so
DWB can no longer plan a correction it will not execute. This is deployed and
confirmed live. It does not touch the corner itself.

**The corner is still the open question, and one candidate answer has already
been ruled out rather than assumed.** `smoother_server` is configured and
running and the stock tree never calls it, which reads like the obvious fix —
except `SimpleSmoother` has no notion of curvature or of this rover's turning
limits at all; what it optimises is jaggedness against the costmap. Called
directly, against the real running node, on the four sharpest corners recorded
during this fault: the smoothed path came back point-for-point identical to
what went in. It is not going to fix a corner that is genuinely too tight,
because tightness is not a thing it can see.

What would see it is a planner that knows the rover's own turning limits and
never draws a corner past them — Nav2 ships `SmacPlanner` in a mode built for
exactly this — in place of `NavFn`'s plain grid search, which has no notion of
the body behind it at all.

That is now the configured planner. `hybrid_astar.py` is a Dubins Hybrid-A*
with this chassis's radius (`max_vel_x / max_vel_theta` = 0.51 m) and
`smac_replay.py` holds it to the same standard every earlier fix eventually
got: the same costmap, both searches, the path geometry. On a metre-wide
passage that turns 55 degrees — the middle of the 44-to-67 the recordings
measured in the first 1.2 m of plan — the grid path's tightest 0.32 m window
was 45 degrees and Hybrid-A*'s was 35.8, which is the rollout envelope
itself. The live plugin is `nav2_smac_planner::SmacPlannerHybrid`, still
named `GridBased` so the behaviour tree does not have to move, Dubins rather
than Reeds-Shepp because the controller has no reverse sample.

It does not close a loop on a frozen map. That is how the 0.8 m look-ahead
shipped. The number that decided this one is the path's own corner.
