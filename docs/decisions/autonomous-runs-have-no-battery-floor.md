# Autonomous runs have no battery floor

Status: agreed 2026-10-02 by the owner; implemented in rover_daemon and autonomy.

An autonomous run is conditioned on the battery as every other drive on this rover is:
not at all. Nothing refuses an autonomous action, ends a run or shuts the goal gate
because of the pack voltage. What ends a run on a flat pack is the board's own cutoff,
as it does for the owner's own drives and the voice model's. Runs are still bounded by
time, travel, actions and failures in a row
([R-SAFE-10](../requirements/safety.md#r-safe-10)), and by the lease, the pose, the map
and the safe area.

## Why

The floor was 11.2 V (3.73 V per cell), and the driver board's curve calls that about a
fifth left. It was read at the moment of each check, with the motors pulling, and the pack
sags 0.3 to 0.6 V under that load. So a pack resting at 11.3 to 11.5 V read below the
floor in the middle of a drive. On 2026-10-02 it ended M0a's supervised runs within 20 to
25 minutes of a charge, at 10.92, 11.13 and 11.16 V
([the runs](../progress/2026-10-02-m0a-runs-reach-twenty.md)). The owner judged that a
supervised indoor rover is better served by running until it stops.

## What was considered

- **Judge the floor on the resting voltage**, or one averaged over several seconds. That
  would have kept a reserve without the false stops. It was offered, and the owner chose
  to treat autonomy like every other drive instead.
- **Lower the floor to 10.0 V.** This was an earlier attempt, and it was refused at the
  tool level as weakening a safety limit without the owner's explicit decision.

## What would reopen it

An autonomous run that is not supervised, or a rover that stops somewhere it cannot be
reached or recharged. Either makes a reserve worth having again, and then it should be
judged on a resting reading.
