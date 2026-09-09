# The five autonomy review defects are fixed and deployed

The fixes were deployed to the Orin at `9d1a5c2` on September 8. This entry
records the completed verification after the session resumed on September 9.
The rover remained stationary throughout live verification, and autonomy was
left disabled and latched. Moving acceptance trials remain outstanding.

## What failed and what now passes

All five defects were reproduced offline before changes to the running rover.
The regression checks use the production permission and executive code with
fake hardware; navigation also exercises its actual goal-dispatch method without
requiring ROS on the workstation.

- A stop between permission checking and dispatch could be acknowledged before
  a drive started. Permission transitions and dispatch are now serialized, and
  delayed navigation requests carry the bridge's stop sequence. Navigation
  refuses a stale sequence and cancels a handle accepted after a stop.
- Clearing the semantic world ended authority without cancelling an existing
  drive. Takeover now stops navigation as well as revoking permission. Ending
  a run sends a stop even when a background trip has not started its worker.
- The safe area constrained destinations only. Navigation now checks adjusted
  destinations, current position and published routes, including replans; the
  daemon independently monitors position. Both reserve a 0.5 m inset for the
  body and stopping. Its physical adequacy is not yet measured.
- Each recovery stop reset the consecutive-failure counter. Recovery stops no
  longer forgive failed goals; three failed goals exhaust the default budget.
- A stop or lost connection during a drive could leave no action in the episode.
  Dispatch intent is committed before the request; interrupted calls retain an
  unknown-completion result. Replay and summaries also expose an unanswered
  intent after a process kill instead of reporting that nothing was called.

Implementation details belong to [autonomy](../../autonomy/README.md),
[the daemon](../../rover_daemon/README.md) and
[navigation](../../ros_nav/README.md).

## Verification on the Orin

The deployer copied the three affected components, restarted the daemon and ROS
navigation, and completed their readiness checks. The on-host suites reported:

| Component | Passed | Failed |
|---|---:|---:|
| autonomy | 651 | 0 |
| rover daemon | 978 | 0 |
| ROS navigation | 576 | 0 |

Over TCP 8769, an out-of-bounds autonomous destination was refused, a stop was
accepted and its repeated action identifier refused, and a latched stop refused
permission renewal. Over the navigation bridge on 8773, a goal carrying an
impossible old stop sequence was refused before navigation began. The new stop
sequence was also visible through the daemon's `nav_status` on 8769.

The reported pose was identical before and after: x -17.644 m, y -17.446 m,
heading 136.1 degrees. These are stationary protocol checks, not observations of
physical stopping or boundary containment while driving.

## Requirements and remaining acceptance

R-SAFE-9, R-SAFE-10, R-SAFE-11 and R-SAFE-12 remain open. Their software
regressions pass, but their supervised moving trials have not been performed.
R-AUT-11 also remains open: duplicate refusal was exercised without driving.
M0 still gates autonomous movement. Once it passes, measure stop latency,
physical braking distance and boundary containment before accepting M3.
