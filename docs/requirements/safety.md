<!-- requirement-area: SAFE -->

# Safety and authority

Who is allowed to move the rover, what stops it, and which boundaries no later
capability may cross. The conventions for these records are in
[README.md](README.md).

The shape of this area is that authority sits *below* cognition. The parts of the
system that decide what would be interesting are not the parts that are allowed
to move the wheels, and everything here exists to keep that true as more
deciding gets added.

The September 8 code review added serialized dispatch/takeover, cancellation of
queued and late accepted navigation goals, journey boundary checks and failure
accounting that excludes recovery stops. Local regressions exercise these paths.
R-SAFE-9, R-SAFE-10, R-SAFE-11 and R-SAFE-12 remain open: these changes do not
replace their supervised moving trials.

<a id="r-safe-1"></a>
### R-SAFE-1 — One process owns the driver-board UART and the gimbal camera

- **State:** settled
- **Evidence:** [rover_daemon/README.md](../../rover_daemon/README.md); the
  daemon is the only component the manifest starts with `--vision`

The ESP32 has a single UART and the gimbal camera is a single device. Two
processes opening either of them can interleave motor and gimbal commands or race
for frames, so single ownership is a correctness requirement rather than a
matter of taste. An inspection wanting a picture is not a reason for a second
process to open the camera; it asks the owner.

<a id="r-safe-2"></a>
### R-SAFE-2 — Every motor command reaches the board through the daemon

- **State:** settled
- **Evidence:** [rover_daemon/README.md](../../rover_daemon/README.md);
  [ros_nav/README.md](../../ros_nav/README.md)

ROS does not talk to the board. It receives odometry and sends motor commands
over loopback to the daemon, which keeps the single-owner rule intact while
letting Nav2 drive. Anything that wants the rover to move — the console, the
voice model, a script, a future executive — arrives at the same place.

<a id="r-safe-3"></a>
### R-SAFE-3 — Driving operations are checked before they move the rover, and the check cannot be skipped

- **State:** settled
- **Evidence:** [ros_nav/README.md](../../ros_nav/README.md) — `drive` is
  checked against the local costmap; routed moves are planned by Nav2

The daemon's movement tools are bounded operations with their own preconditions,
not a pass-through to a velocity command. There is no interface that accepts a
motion and declines to check it, which is what makes it safe to give movement to
progressively less predictable callers.

<a id="r-safe-4"></a>
### R-SAFE-4 — What the rover believes about the room grants it no authority to move

- **State:** settled
- **Evidence:** [world_state/README.md](../../world_state/README.md);
  [rover_daemon/README.md](../../rover_daemon/README.md) — world-state calls are
  absent from `list_tools`

The visual memory has no path to the motors. The conversational model reads it
through `find_thing` and `distance_between_things`, and the one thing it may act
on is `go_to_thing`, which is the existing navigation boundary with a placed
coordinate handed to it. It cannot write to the store or clear it.

Identity does not grant movement authority. Under the revised M0 gates, an
uncertain association may motivate a bounded verification request only through
the separately validated inspection path required by R-AUT-12. Actions that rely
on its identity still require [R-WS-13](world-state.md#r-ws-13). That inspection
path is not yet accepted; this revision enables no movement and changes none of
the daemon's checks.

<a id="r-safe-5"></a>
### R-SAFE-5 — A person can stop the rover at any time

- **State:** settled
- **Evidence:** [ros_nav/README.md](../../ros_nav/README.md) — `stop_driving`
  cancels movement; the console carries it

Every movement the rover makes is cancellable from the console, including a
background exploration run. Nothing the rover does is a committed sequence that
has to be waited out.

<a id="r-safe-6"></a>
### R-SAFE-6 — Unattended movement happens only where drops and off-plane obstacles are known to be absent

- **State:** open
- **Blocked by:** no drop or edge sensing exists — see
  [R-NAV-8](navigation.md#r-nav-8). Validating one is not on any current plan.

The rover cannot see a stair, a threshold, a table top or anything else outside
the lidar's single horizontal plane, so it cannot itself tell a safe room from an
unsafe one. Until something can, the standing rule is that a person watches
while the rover drives itself and the space has been cleared by hand.

This is stated as a requirement rather than left implicit because the autonomy
work in [../plans/autonomous-curiosity.md](../plans/autonomous-curiosity.md)
would otherwise inherit it silently. Every physical autonomy trial there is
supervised for this reason.

<a id="r-safe-7"></a>
### R-SAFE-7 — Movement the rover starts by itself is bounded and reports why it stopped

- **State:** settled
- **Evidence:** [ros_nav/README.md](../../ros_nav/README.md) — exploration has a
  time budget, abandons goals making no progress, and its status says why it ended

Exploration is the one thing today that drives without a person choosing each
goal. It ends on its time budget, on running out of useful frontier, or on being
cancelled, and it says which of those happened and how far it drove. A run that
cannot say why it stopped is indistinguishable from one that failed silently.

<a id="r-safe-8"></a>
### R-SAFE-8 — No generated or learned procedure drives the wheels or the gimbal directly

- **State:** proposed
- **Proposed in:** [../plans/autonomous-curiosity-design.md](../plans/autonomous-curiosity-design.md)

A skill acquired from experience composes existing bounded operations. It does
not emit velocities or servo angles, and it is not arbitrary Python: rover-side
scripts are process isolation and not a sandbox, which is exactly why they are
not the representation for anything the rover writes itself. See
[R-CTL-10](control.md#r-ctl-10).

<a id="r-safe-9"></a>
### R-SAFE-9 — Every autonomous decision and physical action is attributable to a recorded episode

- **State:** settled
- **Evidence:** [2026-10-06, the trace](../progress/2026-10-06-every-move-traced-to-its-episode.md):
  every move navigation logged while a run was open in M3 sessions 1-5, 145 of
  them, pairs with a `drive_to` a recorded episode dispatched; the two that do not
  were hand drives between runs; `python autonomy/selftest.py`

An action nobody can reconstruct afterwards cannot be reviewed, and a failure
nobody can replay cannot be fixed under this repository's rules. Episodic
recording is therefore the first piece of autonomy to be built and carries no
authority of its own.

The decision half is settled ([R-AUT-1](autonomy.md#r-aut-1) and the M1 pass).
The action half is built, and has carried real movement in M0a's runs: every autonomous action
is dispatched through one call carrying the episode it belongs to and an
identifier beginning with that episode's reference, and the daemon refuses an
action that names neither.

<a id="r-safe-10"></a>
### R-SAFE-10 — Autonomous runs are bounded by time, travel, actions and failures

- **State:** retired
- **Superseded by:** [../decisions/runs-start-from-the-console-or-an-agent.md](../decisions/runs-start-from-the-console-or-an-agent.md)

**Retired on 2026-10-03.** A run started from the console has no limit on time,
travel or actions, and one started by an agent has whatever it asks for.
[R-SAFE-16](#r-safe-16) is what every run is still held to. What follows is the
history up to then.

Exploration already has the time half of this ([R-SAFE-7](#r-safe-7)). A general
executive needs the rest, because the failure it protects against is not a
crash but a rover that keeps making locally reasonable decisions.

**There is no battery bound.** Until 2026-10-02 a run also ended below 11.2 V; read
under load, that ended supervised runs within minutes of a charge, and the owner
[decided](../decisions/autonomous-runs-have-no-battery-floor.md) that autonomous runs are conditioned on the battery as every other
drive is.

The bounds are declared when a person opens a run, are enforced by the daemon
rather than by the executive, and end the run when spent; the standing limits
and the reason for each number are in
[rover_daemon/permission.py](../../rover_daemon/permission.py). **Travel is
spent from where the rover actually gets to**, half a second at a time, rather
than from what each action said it would cost, so a move nobody is waiting for
is charged for. What is owed is a hardware run in which a budget is what stops
the rover.

<a id="r-safe-11"></a>
### R-SAFE-11 — A stop request prevents autonomy from restarting itself

- **State:** open
- **Blocked by:** [../plans/autonomous-curiosity.md](../plans/autonomous-curiosity.md)
  (M3) — shown against a moving rover in the stop trials of 2026-10-02; M3's
  sessions are owed

Stopping movement today stops the movement. Once something is choosing goals,
stopping has to also revoke its authority until it is deliberately given back, or
the stop becomes a pause and the person has to keep pressing it.

`stop_driving` now ends any autonomous run and latches autonomy off, and so does
any other sign of a person taking the rover back: driving it by hand, sending it
somewhere by voice, running a script, clearing the map, refitting the pose. The
latch is cleared only by opening a new run: the console's run button, or an agent's
`autonomy_start` (or `autonomy_enable`). Since 2026-10-03 an agent may do that
straight after a person's stop
([the decision](../decisions/runs-start-from-the-console-or-an-agent.md)).
**The executive's own client refuses both calls**, the same structural refusal
that keeps the recorder off the wheels, so an executive that crashed and restarted
cannot give itself back what a person removed.

<a id="r-safe-12"></a>
### R-SAFE-12 — The daemon enforces permission expiry and budgets on its own

- **State:** settled
- **Evidence:** [2026-10-06, on the rover](../progress/2026-10-06-repeat-hang-and-drop-on-the-rover.md):
  a hung executive and a lost connection each lost a moving rover its permit, at
  rest 0.31 m and 0.27 m after expiry with Nav2 up, and neither the woken nor a
  fresh executive got the run back; a restarted daemon came back with no run;
  [2026-10-02](../progress/2026-10-02-first-m0a-runs.md) for a killed one;
  `python rover_daemon/selftest.py`

The check has to live below the thing being checked. If the executive is what
notices that its own permission ran out, then an executive that has hung or is
looping keeps its authority precisely when it should lose it. A restart must not
restore revoked authority.

Permission is a fifteen-second lease the executive renews as it works, and
nothing renews it on the executive's behalf — no heartbeat thread, deliberately,
since a heartbeat that outlives the loop it stands for is the failure this
exists to prevent. A thread in the daemon ticks twice a second while a run is
open and stops the wheels when the lease, the budget, the pose or
the map says the run is over. None of that state is written to disk, so a
restart leaves no run and no permit to restore.

<a id="r-safe-13"></a>
### R-SAFE-13 — An operation a skill did not declare is refused before it runs

- **State:** proposed
- **Proposed in:** [../plans/autonomous-curiosity-design.md](../plans/autonomous-curiosity-design.md)

Validation happens against the declared operation graph ahead of execution, not
by catching a failure partway through a sequence that has already moved the
rover.

<a id="r-safe-14"></a>
### R-SAFE-14 — A skill cannot promote itself

- **State:** proposed
- **Proposed in:** [../plans/autonomous-curiosity.md](../plans/autonomous-curiosity.md) (M7)

Whatever proposes a new procedure is not what decides it may be used
autonomously. Promotion requires deterministic validation and measured physical
trials.

<a id="r-safe-15"></a>
### R-SAFE-15 — Losing a model fails closed

- **State:** proposed
- **Proposed in:** [../plans/autonomous-curiosity-design.md](../plans/autonomous-curiosity-design.md)

An unreachable or malformed model must stop the actions that depend on it rather
than let them proceed on a default. Operations that are fully validated without
a model stay available, so an outage costs the rover its judgement and not its
ability to stop safely. Today this is straightforward because the only model in
the loop is conversational and the rover works without it.

<a id="r-safe-16"></a>
### R-SAFE-16 — An autonomous run ends on the limits it was started with, and after three failures in a row

- **State:** settled
- **Evidence:** [2026-10-02, the stop trials](../progress/2026-10-02-drive-carry-and-stops.md):
  on hardware a run closing on its failure count stopped a moving leg within 0.08 m;
  `python rover_daemon/selftest.py`

Every run carries a budget of minutes, metres, actions and failures in a row, fixed
when it is opened and enforced by the daemon rather than the executive. Minutes,
metres and actions may each be no limit: a run from the console's button has none,
and an agent's run has the standing limits for whatever it leaves out
([the decision](../decisions/runs-start-from-the-console-or-an-agent.md)). Failures
in a row are always a limit, so a rover that is managing nothing from where it
stands stops rather than trying until the battery dies. The lease is not a budget
and cannot be lengthened ([R-SAFE-12](#r-safe-12)).

**Travel is spent from where the rover actually gets to**, half a second at a time,
rather than from what each action said it would cost, so a move nobody is waiting
for is charged for. The standing limits and the reason for each number are in
[rover_daemon/permission.py](../../rover_daemon/permission.py).

<a id="r-safe-17"></a>
### R-SAFE-17 — A rover that cannot measure its own turning does not move by itself

- **State:** proposed
- **Proposed in:** [2026-10-06, the gyro gate](../progress/2026-10-06-a-rover-that-cannot-feel-itself-turn.md)

The rover's turns end when its gyro says they have turned far enough. Three times
the gyro has gone wrong and stayed wrong until the rover was switched fully off:
its bias read +150 deg/s on 2026-10-03, exactly zero on 2026-10-05, and
-2,063 deg/s on 2026-10-06. Each time navigation still called the position
trusted, and a turn could not tell when it was done. On 2026-10-06 an
autonomous leg stood for 27 s wanting a turn it never made.

So an autonomous run is not opened, a drive is not dispatched, and a run that
is open is ended, while the bias the base reports is exactly zero or beyond
5 deg/s either way (`permission.rotation_fault`). A look is still allowed,
because it turns nothing. The same reason is carried in `nav_status` as
`rotation_fault`, and the drive console shows it under its header for as long
as it lasts, since only a person can cut the power. What would make this false is an autonomous drive
dispatched while the base reports such a bias.

Proposed rather than settled: no fault has happened since the rule was
deployed. Replayed over the 21,528 readings the base has logged, it refuses all
339 taken during the three faults, and one other.
