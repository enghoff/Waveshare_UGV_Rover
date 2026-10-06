# A run's looks seldom improve the thing they were aimed at

**Of 171 looks the autonomy record holds for improving where a thing is placed,
10 improved it by the time the run measured, 9 more within a minute, and 152
never did.** The cause is not the depth camera. Nor is it, beyond a few cases,
the run measuring too early. A look's new evidence mostly does not reach the
thing it was aimed at. That is a world-state question, and it sits in the open
identity work under [R-WS-13](../requirements/world-state.md#r-ws-13). Nothing
was changed. No requirement moved.

## What was measured

Read from the episode record on the rover, sessions of 2026-10-02 to
2026-10-06. Each geometry goal records the thing's placement uncertainty
before the drive and straight after the look. Each later decision's snapshot
records it again.

| | Goals |
|---|---|
| better when the run measured | 10 |
| no better then, better within 60 s | 9 |
| no better within 60 s | 152 |

The second row is the most that measuring too early can explain. A look that
found the rover's own looking had just taken the same picture stores nothing.
The bearing it would have added waits for the next 10-second settling pass.
Some of those nine may equally be other looks taken in the minute after.

**Depth was asked for and delivered, then went elsewhere.** 63 of the looks were
for things never ranged, where the whole promised gain was a depth reading.
In 48 of them the depth camera ranged some regions in the picture. One left
its thing ranged by the next decision. One look found the camera still waking.

**In session 3, 8 of the 17 things aimed at gained no observation at all**
during the run, although the rover drove to a viewpoint facing each and looked.
The other nine gained one to six, and none came out better placed. Looks that
day carried heading corrections of up to 12.5 degrees against the map. That
fits the morning's turn-heading disagreement, and that much error would put a
bearing outside a thing's gate.

## What it means

The scorer believes a geometry goal's prediction, from 0.17 m against 1.09 m
for example, and that belief is right about one time in ten. So a run spends
most of its battery on looks that change nothing. The cooling-off stops it
spending more than one look on each thing, and that is what ended session 3
early.

Two things follow, neither done here. Why a targeted look's regions do not
attach to the thing belongs to the identity investigation
([entity-evidence-drive.md](../plans/entity-evidence-drive.md)). This record of
171 goals and the snapshots behind them is evidence for it. On the autonomy
side, weighting a goal by what goals of its kind have actually delivered is
Phase 10's outcome model in
[the plan](../plans/autonomous-curiosity.md), not a change to make before
the identity work says what a look can deliver.
