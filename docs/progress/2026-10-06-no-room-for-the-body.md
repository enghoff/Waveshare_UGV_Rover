# A goal the bridge has no room for is no longer chosen

**The run now refuses a goal the navigation bridge would refuse for want of room
for the rover's body, instead of driving at it and failing.** Seven of the 230
autonomous drives on record, two of them in
[M3 session 3](2026-10-06-m3-session-3.md), were refused this way in two seconds
each, and each counted towards the three failures in a row that end a run. The
run's check now agrees with the bridge on every recorded drive. It refuses all
ten of these, the three it already caught and the seven it missed, and keeps all
200 that arrived. Deployed at 0b89519 (autonomy) and proved on the rover's live
map. [R-SAFE-16](../requirements/safety.md#r-safe-16) is unaffected: this
removes failures rather than changing how they are counted.

## The cause

The run decides what it can reach by walking the map. It kept the rover's
centre 0.20 m from walls, which is the planner's own test for a route, and
let a goal count if it was within half a metre of that floor, because the
bridge moves a goal up to half a metre before it plans. The bridge's test of
where it may move a goal is stricter. No part of the body may lie over the
planner's 0.20 m ring round each wall (`ros_nav/goal_fit.py`), so the centre
needs about 0.40 m. Between furniture, a viewpoint the walk could reach was
often half a metre or more from anywhere the bridge would accept. That is
"there is nowhere within half a metre of that spot where the rover's body fits".

## The fix

The walk now moves each goal the way the bridge would: to the nearest place
within half a metre, in the bridge's own order of trying, where the body fits.
The goal counts only if that place exists and the walk reaches it. The body is
stamped as every cell a 0.20 m circle could touch at any heading. So a place
the walk calls fit is one the bridge accepts, and in rare cases the walk may
refuse a place the bridge would have taken. `goal_fit.py` is now deployed into
autonomy beside `frontier.py`, so the two share the same code.

## Evidence

- **Replayed over all 230 autonomous drives on record** (2026-10-02 to
  today), on each decision's own map: the bridge's `goal_fit.fit` on the
  rebuilt costmap finds nowhere for the body at all ten refused goals, and
  somewhere at all 200 arrivals. The new walk agrees with it on every one of
  the 230. The previous walk had already refused three of the ten.
- **The first, faster version disagreed on four.** It assumed the body covers
  the same cells wherever it sits within a cell. It does not: a millimetre's
  offset or a turn changes the edge cells a twelve-sided body touches. The
  stamp that counts every cell the body could touch fixed all four and lost no
  arrival.
- **Tests:** the seven drives are a fixture (`no-room-for-the-body.json.gz`),
  chosen as exactly those the previous walk called reachable. Each is refused by
  the bridge's own fit and by the new walk, and the next drive that arrived stays
  reachable. A second test lays the bridge's polygon at shifts and turns over
  every cell the walk calls fit near one recorded goal (over 1,000 poses) and
  finds none refused. autonomy 785 passed here and on the rover in the deploy.
- **On the rover:** with the live map, the two goals refused in session 3
  (episodes 783 and 789) are now unreachable, and the places the rover reached
  next are still 2.45 m and 4.45 m away. One read-only deliberation weighed 61
  candidates in 1.33 s, with the walk itself taking 0.51 s. Desktop replay
  measured the walk at 0.36 s against 0.30 s before.

## Deployment

The manifest gained an entry for `goal_fit.py`. Every component counts the
manifest as a trigger, so the other nine were marked changed while their files
had not. Each had no changed source since its deployed commit, so they were
adopted at 0b89519 with nothing copied and nothing restarted. Navigation was
not restarted.

## What it means

Of the 30 failed drives on record, the run's own check can now see 16 coming:
the ten above and six "no route" drives into the pocket by the charger. The
other 14 are things a walk over the map cannot see. Four were the safe area,
four were the rover found standing inside the costmap, two were timeouts, and
four were the run ending mid-drive.
