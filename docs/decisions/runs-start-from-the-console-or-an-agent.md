# Runs start from the console's button or an agent's call

Status: agreed 2026-10-03 by the owner; implemented in rover_daemon, autonomy and
drive_web. Retires [R-SAFE-10](../requirements/safety.md#r-safe-10) in favour of
[R-SAFE-16](../requirements/safety.md#r-safe-16), and changes who may clear the stop
in [R-SAFE-11](../requirements/safety.md#r-safe-11).

An autonomous run is started in one of two ways:

- **The console's run button**, which asks nothing of the person. The run has no
  limit on time, travel or actions. Three failures in a row still end it, and so
  does the console's last tab going away.
- **An agent's call**, `autonomy_start` over the daemon's protocol, with a purpose
  and optionally a budget and a safe area. Whatever budget it leaves out takes the
  standing limits of 15 minutes, 60 m and 40 actions. It may ask for more, or for
  none. An agent's run does not need a console open.

Both open the run and start the executive on it. The run records which way it was
started (`via`: `console` or `api`), not who started it. The safe area is the mapped
floor unless a geofence narrows it. Opening a run clears a stop, whichever way it is
opened, including straight after a person stopped the rover. The executive still
cannot open a run.

## Why

Until now a person opened a run with a terminal call, naming themselves, inside
standing limits they could only shorten. That made every supervised session start
from a shell, and it gave no way in to an agent working for the owner. The owner wants
the console to start a run in one press and an agent to start one with a purpose. They
judged that for a supervised indoor rover, the person at the console is the bound on a
console run, not a clock.

## What was considered

- **Keep the standing limits for console runs.** That was declined: the owner is
  watching, and a run that ends at 15 minutes ends a session that was going well.
- **No limit at all for console runs, failures included.** That was declined too.
  Three failures in a row means the rover is managing nothing from where it stands,
  and it should stop rather than keep trying until the battery dies.
- **Only a person may clear a stop.** That was offered as the recommendation and the
  owner chose otherwise. An agent may start the next run after a person's stop. A stop
  still ends the run in progress, and the executive still cannot restart itself
  (R-SAFE-11). What changed is that an agent's deliberate call is a way back, as well
  as a person's.
- **Require a console open for an agent's run.** That was also declined. An agent's
  run is bounded by the budget it declares, and is stopped by voice, the API, or a
  person driving the rover.
- **Replace the console's explore button with the run button.** That was declined:
  both stay.

## What stays

The 15 second lease, which is what stops a hung or disconnected executive. The checks
at dispatch: pose, map, safe area, a repeated action, and the inspection limits. The
three admitted operations. A person's stop and takeover ending a run. There is no
battery floor ([the decision](autonomous-runs-have-no-battery-floor.md)).

## What would reopen it

An unsupervised run, which the console cannot start but an agent now can. A rover left
running unattended by an agent would want a requirement of its own about where it may
go, as [R-SAFE-6](../requirements/safety.md#r-safe-6) already asks. Or a run that
should have stopped and did not.
