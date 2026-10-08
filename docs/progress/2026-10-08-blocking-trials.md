# Blocking trials: a short goal waits and goes round; a longer one waits, then drifts into the person on the way round

**With the owner told in advance where the rover would drive
(`experiments/m3_trials/block_trial.py`), the two person-in-the-way cases were
driven on the trials' straight leg in the charger room.** On a 1.2 m goal the
owner stood 0.7 m in front and stayed: the rover waited 3 s, went round in
three straight legs and arrived at its goal -- M3's last owed person condition,
met. On the 4.4 m leg, twice, the owner stood still on its line about 1.6 m up:
the route watch of [M4 session 4](2026-10-08-m4-session-4.md) stopped it and it
waited 3 s instead of turning on the spot, but the way round failed both times.
A recording of the live scan shows why: the last leg, planned to keep 0.30 m
from the owner, drifted 9 degrees towards them in half a metre of "straight"
driving, and Nav2's own collision check stopped it there. Safe, but it did not
get past. Nothing was changed on the rover after these trials.

## How it was run

`block_trial.py start` takes the rover to the charger end of the leg,
(-17.03, -16.01), facing up the room; `far` drives the leg to (-17.05, -11.60) as
one goal; `near` drives 1.2 m straight ahead arriving facing the same way. Each
waits before it moves -- the owner said "go" when in place -- and logs the pose
and navigation's account twice a second. It refuses to drive while the gyro
bias reads zero or beyond 5 deg/s: the motion sensor came up frozen at the
evening's first power-up, the same fault as that morning, and a power cycle
cleared it (0.47 deg/s). Traces and the recording are in
`captures/2026-10-08-block-trials/`.

## The short goal (M3)

`block-near-223141`: the owner 0.7 m in front before it set off. Waiting from
22:31:47 to 22:31:49 without moving; then legs west, north-east and north-west,
arriving within 6 cm of its goal at 22:31:58. No turning on the spot, and it
never drove at the owner. (An earlier attempt drove straight to its goal: the
owner was not yet in place, and both its checks rightly saw nothing.)

## The longer goal

`block-far-221919` and `block-far-223900`, the owner standing still about 1.6 m
up the leg, a little right of its line:

| | First | Second |
|---|---|---|
| stopped for the owner, after driving | 1.2 m | 1.0 m |
| waited | 3 s | 3 s |
| went round | left, 0.85 m, then back | left, 1.3 m, then across |
| last leg | stopped by Nav2's "Collision Ahead" after 1.5 s | the same, after 1.8 s |
| outcome | handed back as "something is in the way" | the same |

From the second trial's recording (`far-rec.json`, `ros_nav/nav_record.py`,
149 live costmaps): as the last leg began, the nearest cells the scan held were
the owner's, 0.67 m off, 0.24 m right of the rover's heading. The leg was
planned at the go-round's 0.30 m margin. During it the heading went from 3 to
12 degrees -- a straight drive on tracks, with nothing steering it back -- and
the rover's centre came within 0.29 m of the owner's cells, which Nav2's drive
check, looking ahead along the leg, would not pass. The owner, watching, saw
plenty of room: the stop was the check working on a drifting leg.

## What it means

A way round planned to a margin and driven open-loop, turn then straight, loses
the margin to the chassis: its turns land several degrees off and its straight
drives curve. A tighter margin, which the owner asked about for tight spaces,
needs the detour driven closed-loop. In order:

1. **Plan the next legs again from wherever a leg ends or is stopped**, from the
   live scan, a few times before handing the goal back. The drift then costs a
   re-plan rather than the goal.
2. **Aim past the obstacle further**: the point the way round aims for is 1.4 m
   along the route from the stop, which with the owner 1 m ahead is only about
   0.4 m past them.
3. **Then consider following the detour with Nav2's controller**, which steers
   continuously and checks the whole body every cycle, so it can keep a smaller
   margin safely.
