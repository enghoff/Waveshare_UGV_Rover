# A run with nothing left to do goes further, then home

Status: agreed 2026-10-03 by the owner; implemented in autonomy and rover_daemon.
Adds [R-AUT-13](../requirements/autonomy.md#r-aut-13).

A run no longer stands still waiting for something to become worth doing. In
order:

- **Further afield first.** When nothing nearby is worth its cost, the run takes
  the goal whose only fault is the trip, cheapest first. A goal refused for any
  other reason is still refused. So is one worth too little to disturb the rover
  for.
- **Then home.** When nothing anywhere is worth doing, the run drives back to
  where the rover stood when the run was started, on the map it started on, and
  ends. That is not a stop, so nothing latches.
- **A rover that cannot act waits two minutes, then goes home.** A failed camera
  or a pose the rover does not trust may clear, so the run waits for it. After
  two minutes it drives back if it can and ends.

## Why

The first console runs on 2026-10-03 spent their last minutes standing still.
Every thing nearby had been put aside for fifteen minutes, and the run waited for
one to come back. The owner's words: that "just has the rover sitting around
draining its battery". They want a rover that has used up where it is to go
somewhere else, and one with nowhere left to go to come back to where it started,
so it can be plugged in.

## What was considered

- **Keep waiting out the cooldown.** This was the design until now: standing still
  cost nothing, so a goal had to be worth more than it cost. A rover that is still
  switched on is not free, and waiting is what the owner objected to.
- **End the run where the rover is.** This saves the drive back, but leaves the
  rover somewhere nobody chose, away from its charger.
- **Go further with no floor on the gain.** This was declined. The minimum gain
  is what stops a long drive for nothing, and it still applies.

Shadow decisions are unchanged. They cannot act, so for them standing still is
still free, and they record what the scorer would do with idle worth zero. The
M0a protocol is frozen and keeps the scorer it had.

## What would reopen it

A run that goes further only to find nothing there, again and again. That would
mean the scorer's predictions of what a far goal buys are wrong, and the trip
should be weighed differently rather than taken regardless.
