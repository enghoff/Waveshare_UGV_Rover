<!-- requirement-area: SAFE -->

# Safety and authority

Who is allowed to move the rover, what stops it, and which boundaries no later
capability may cross. The conventions for these records are in
[README.md](README.md).

The shape of this area is that authority sits *below* cognition. The parts of the
system that decide what would be interesting are not the parts that are allowed
to move the wheels, and everything here exists to keep that true as more
deciding gets added.

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

This matters most while identity is unproven: a wrong association should cost a
wrong answer, not a drive across the room. See [R-WS-13](world-state.md#r-ws-13).

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

- **State:** open
- **Blocked by:** [../plans/autonomous-curiosity.md](../plans/autonomous-curiosity.md)
  (M3) — no autonomous action has yet moved this rover

An action nobody can reconstruct afterwards cannot be reviewed, and a failure
nobody can replay cannot be fixed under this repository's rules. Episodic
recording is therefore the first piece of autonomy to be built and carries no
authority of its own.

The decision half is settled ([R-AUT-1](autonomy.md#r-aut-1) and the M1 pass).
The action half is now built and unproven on hardware: every autonomous action
is dispatched through one call carrying the episode it belongs to and an
identifier beginning with that episode's reference, and the daemon refuses an
action that names neither. What is owed is a supervised session in which real
movement is traced back that way.

<a id="r-safe-10"></a>
### R-SAFE-10 — Autonomous runs are bounded by time, travel and battery

- **State:** open
- **Blocked by:** [../plans/autonomous-curiosity.md](../plans/autonomous-curiosity.md)
  (M3) — the budgets have never been spent by a rover that was driving

Exploration already has the time half of this ([R-SAFE-7](#r-safe-7)). A general
executive needs all three, because the failure it protects against is not a
crash but a rover that keeps making locally reasonable decisions until its
battery is flat somewhere inconvenient.

All three are declared when a person opens a run, are enforced by the daemon
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
  (M3) — shown against a fake rover, not yet against a moving one

Stopping movement today stops the movement. Once something is choosing goals,
stopping has to also revoke its authority until a person gives it back, or the
stop becomes a pause and the person has to keep pressing it.

`stop_driving` now ends any autonomous run and latches autonomy off, and so does
any other sign of a person taking the rover back: driving it by hand, sending it
somewhere by voice, running a script, clearing the map, refitting the pose. The
latch is cleared by exactly one call, and **the executive's own client refuses
that call** — the same structural refusal that keeps the recorder off the
wheels, so an executive that crashed and restarted cannot give itself back what
a person removed.

<a id="r-safe-12"></a>
### R-SAFE-12 — The daemon enforces permission expiry and budgets on its own

- **State:** open
- **Blocked by:** [../plans/autonomous-curiosity.md](../plans/autonomous-curiosity.md)
  (M3) — the watchdog has never had to stop a rover that was really moving

The check has to live below the thing being checked. If the executive is what
notices that its own permission ran out, then an executive that has hung or is
looping keeps its authority precisely when it should lose it. A restart must not
restore revoked authority.

Permission is a fifteen-second lease the executive renews as it works, and
nothing renews it on the executive's behalf — no heartbeat thread, deliberately,
since a heartbeat that outlives the loop it stands for is the failure this
exists to prevent. A thread in the daemon ticks twice a second while a run is
open and stops the wheels when the lease, the budget, the battery, the pose or
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
