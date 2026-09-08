# The rover can be given permission to move itself, and three ways to take it back

**Phase 3 is built and the rover has not moved under it once.** The executive
that carries out what the deliberation chose is on the Orin, and so is the
permission it works under — a bounded run a person opens, a fifteen-second lease
the executive renews, and a watchdog in the daemon that stops the wheels when
nobody does. What is proved today is the logic and the wiring; what is not
proved is anything about a rover in motion, because none of the twenty
supervised sessions [M3](../plans/autonomous-curiosity.md) asks for has happened.

Deployed to the Orin at `4ce4d4a` and exercised there over TCP 8769. The rover's
pose is identical before and after every check on this page: `(-17.55, -17.33)`,
heading 147.1.

## What was built

The authority does not live in the thing that decides. A person opens a run with
`autonomy_enable`, naming themselves and a budget — minutes, metres, actions,
failures in a row, a battery floor, a safe area — and the daemon spends that
budget and closes the run when it is gone. Inside a run, permission to act is a
lease the executive renews as it works, and **nothing renews it on the
executive's behalf**: no heartbeat thread, deliberately, because a heartbeat
that outlives the loop it stands for is the failure the lease exists to catch.

Every autonomous action goes out as one call carrying the permit, the episode it
belongs to and an identifier built from that episode. The daemon re-checks the
latch, the run, the lease, the budget, the battery, the pose, the map identity
and the safe area at dispatch — not at the moment permission was granted — and
refuses an identifier it has already dispatched. Three operations are admitted:
drive to a point, take one look, stop.

`rover_daemon/permission.py` is the rule set, a state machine over a clock with
no rover in it, and it is deployed into `autonomy/` as well as into the daemon,
the way `ros_nav/frontier.py` already is. That is what lets the executive's own
checks run against the rules the rover enforces rather than against a stand-in
that agrees with them today.

## What was measured on the rover

Ten checks over the tool protocol, in one pass, moving nothing:

| Asked | Answered |
|---|---|
| status on a freshly restarted daemon | `enabled: false`, no latch, "autonomy has not been enabled since this daemon started" |
| permission, with no run open | refused: "autonomy is not enabled" |
| a drive, with no permit | refused `no run`; nothing reached the navigator |
| a run opened with a 2-minute, 1-metre, 6-action budget | granted, permit good for 15.0 s |
| one look under it | `ok`, 0.36 s, recorded like any other look |
| the same look again | refused `already done`, with what happened the first time |
| `explore`, `run_script`, `drive` | refused `not an autonomy action` |
| `stop_driving` | latched; the permit and every action refused afterwards |
| enabling again | latch cleared, new run |
| a 5-second lease and nothing renewing it | the run ended by itself: "the permission ran out and nothing renewed it", and **not** latched, because nobody stopped anything |

The last row is the one worth the reading. That is the daemon taking the wheels
back from an executive that had stopped asking, with Nav2 healthy underneath —
the case that makes the difference between a permission and a promise.

The executive itself was then run against the real house three times with the
safe area declared a centimetre wide and a kilometre away, so that no candidate
could be inside it and the daemon would refuse the dispatch even if the scorer
did not. It attached to the run, weighed 23 goals against the live world state,
refused every one, wrote the episode and handed the run back. `driving` was
false throughout.

## Two faults the rover found that the suite could not

**A parked rover would have ended its own run within a minute.** A turn that
finds nothing worth doing waits thirty seconds; the lease is fifteen. The loop
slept through the wait in one go, so the daemon — correctly, by its own rules —
took the wheels back from an executive that was merely waiting for the room to
change. It is the first turn the real daemon ever answered and it is exactly the
kind of thing no offline check was going to find, because the offline fake had
no watchdog running while the clock moved. Both halves are fixed: the wait is
taken in naps of a third of a lease with a renewal after each, and the fake now
runs the daemon's watchdog whenever anything winds its clock. The check written
for it fails against the loop as the rover ran it.

**A loop that finished its turns reported "the run ended: a run is open".** It
read the rover's account of why it had stopped without noticing that it had
stopped for its own reason, and that sentence then went into the record as the
reason the run was handed back.

## A failing suite on the rover could not fail a deploy

Flagged by the other agent working here and fixed in the same commit, because
the verification this work depends on had the fault: every component's check ran
`python3 selftest.py | tail -2`, so the exit status was tail's and was always
zero. A component whose suite failed on the rover deployed successfully and said
so.

It found something immediately. `autonomy`'s `test_every_episode_says_which_build_produced_it`
redirected the deploy-state file for the reading that set the recorder up but
not for the poll that follows — and the poll reads it again. On a workstation
there is no such file and the check passed; on the rover the poll replaced the
two commits it had been handed with the ones actually deployed, and it failed.
It had been failing there for as long as the file existed.

## Where this leaves M3

Criterion 1 — mock and replay tests over success, daemon refusal, timeout,
service loss, low-battery abort, manual stop and no-candidate idle — is met, with
a stop exercised in each state of the machine. Criterion 2 is met structurally
rather than by measurement: there is no model in the loop at all, and the check
for it is written against the record, since an episode may only say a model
answered by carrying a `model` event.

Everything else needs the rover to move: twenty supervised sessions totalling
two hours, stopping distances against limits declared in advance, and the
budgets being what actually stops a rover that is driving. **And M3 movement
still requires M0**, which has not passed: semantic inspection goals aim at
things whose identity is wrong about one time in five.

One thing is owed before the sessions can start comfortably. Enabling a run is a
person's act and today it is a call over 8769, so a supervised session begins at
a terminal rather than at the console. [The runbook](../runbooks/autonomy-session.md)
has what to type.

## Counts

`autonomy` 641 passed, 0 failed on the Orin; `rover_daemon` 963 passed, 0 failed
there across three runs. Of those, 93 are the daemon's permission rules and 81
the executive's states — all offline, none of them evidence about a moving rover.
