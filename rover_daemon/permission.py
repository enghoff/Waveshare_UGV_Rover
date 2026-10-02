"""Who may move the rover by itself, for how long, and what takes it away.

**This is the check, and it lives below the thing being checked.** If the
executive were what noticed that its own permission had run out, then an
executive that has hung or is looping would keep its authority at exactly the
moment it should lose it. So the account of a run -- how long it may last, how
far it may drive, how many things it may do, what it has spent, and whether a
person has stopped it -- is held here, in the daemon's own process, and the
executive is a client of it like anything else.

The rules, in the order they are enforced:

1. **A person's stop wins, and stays won.** Stopping the rover latches autonomy
   off, and nothing the executive can say clears that latch. It is cleared by
   one call, `enable`, which is the human re-enable; the client the executive
   uses refuses that call the way `autonomy/client.py` refuses `drive`.
2. **Autonomy is off until somebody turns it on, including after a restart.**
   Every bit of state here is in memory. A daemon that restarts has no run, no
   permit and no memory of having had either, so a crash or a redeploy cannot
   restore authority that was taken away -- and neither can restarting the
   executive, because the executive cannot open a run at all.
3. **A run is bounded before it starts.** Wall clock, distance travelled,
   actions dispatched and consecutive failures, declared when
   the run is opened and spent as it goes. The bound the plan cares about most
   is the one nobody remembers to check: travel accumulated during a move
   nobody is waiting for, which is why spending is recorded from the pose the
   rover actually reaches rather than from what each action said it would cost.
4. **Permission is a short lease, renewed by whoever is alive.** A permit is
   good for `PERMIT_TTL_S` and the executive renews it as it works. Nothing
   renews it on the executive's behalf -- no heartbeat thread, deliberately,
   because a heartbeat that outlives the loop it is supposed to represent is
   the exact failure this is here to prevent.
5. **An action is dispatched once.** Requests carry the episode and an action
   identifier, and a repeat of one already dispatched is answered with what
   happened the first time rather than done again. A request naming a map or a
   permit that is no longer current is refused rather than translated.

Nothing here talks to hardware, opens a socket or starts a thread. It is a
state machine over a clock, which is what lets the daemon's checks drive the
real rules and the executive's checks drive the same ones -- the file is
deployed into both components, like `ros_nav/frontier.py` is, so that what the
rover enforces and what the executive expects cannot come apart.

`rover_daemon/rover_autonomy.py` is the half that touches the rover.
"""
from __future__ import annotations

import os
import time
from typing import Any, Callable, NamedTuple

# --- the numbers, each with the reason it is that number ---------------------

#: How long one grant of permission is good for. The longest a working
#: executive legitimately goes without renewing is one inspection with a settle
#: -- half a second for the look, and a resolver pass that `world_state/
#: inspector.py` measures at 1.4 s over 500 bearings and 8 s over 2000 -- so
#: about nine seconds. Fifteen leaves that room without letting a hung
#: executive hold the wheels for longer than a supervised person would tolerate
#: watching.
PERMIT_TTL_S = 15.0

#: The longest a single autonomous run may last. The same fifteen minutes
#: `explore` already caps unsupervised driving at (`EXPLORE_MAX_S` in
#: rover_nav.py), because that is this rover's existing answer to "how long may
#: it drive with nobody choosing each goal", and inventing a second answer here
#: would leave two.
RUN_MAX_S = 900.0

#: How far one run may drive in total. The M0 acceptance run measured this
#: house at 10.4 by 11 m, so sixty metres is about five crossings of it: enough
#: for a session with several goals in it, and far short of a rover that has
#: been quietly driving in circles for a quarter of an hour.
RUN_MAX_TRAVEL_M = 60.0

#: How many actions one run may dispatch. Each goal is a drive and a look, so
#: forty is twenty goals -- more than the deliberation rate produces in fifteen
#: minutes, which makes this a backstop against a loop rather than a policy.
RUN_MAX_ACTIONS = 40

#: How many failures in a row end the run. Three, because two is the ordinary
#: unlucky pair -- a goal that turns out to be unreachable, then a second one
#: near it -- and a third says the rover is not managing anything from where it
#: is standing rather than that this goal was poor.
RUN_MAX_FAILURES = 3

#: **No battery floor.** Until 2026-10-02 no autonomous action started, and a
#: run ended, below 11.2 V (3.73 V per cell). The reading is taken under load,
#: and with the motors pulling a pack resting at 11.3 to 11.5 V read under it, so
#: M0a's supervised runs ended after 20 to 25 minutes on a charge. The owner
#: decided that autonomous runs are conditioned on the battery as every other
#: drive is -- not at all; the board's own cutoff is what ends them.

#: How fast this chassis can possibly be going. `MAX_SPEED_MS` in
#: lidar_slam/nav2 terms, kept here as a number rather than imported because
#: this module is deployed into two components and must depend on neither.
CHASSIS_MAX_SPEED_MS = 0.35

#: The slack on top of that before a pose step is called impossible. A step is
#: travel while it is under `CHASSIS_MAX_SPEED_MS * elapsed + POSE_JUMP_M`, and
#: a relocalisation above it: SLAM correcting the rover's place on the map, or a
#: refit moving it outright. Half a metre is more than the pose wanders while
#: standing still and far less than any correction worth noticing. It is not
#: merely discounted, it ends the run -- the goal was chosen in a frame the
#: rover has just stopped being in.
POSE_JUMP_M = 0.5

#: How often the daemon is expected to look at all this. Not enforced here --
#: this module has no thread -- but the jump threshold above is derived from it,
#: so the two are declared together.
TICK_S = 0.5

# Reserve room inside a declared boundary for the body and braking. Hardware
# acceptance must measure whether this allowance is sufficient at trial speed.
FENCE_MARGIN_M = 0.5

#: What one inspection of an uncertain hypothesis may spend, at most
#: ([R-AUT-12](../docs/requirements/autonomy.md#r-aut-12)). A request declares
#: its own limits and these are the ceilings on them, the same shape as a run's
#: budget: an executive may ask for less and never for more. Eight metres is most
#: of the room the acceptance drives were in; three minutes is a drive across it,
#: a look and a check with room to spare; two attempts is one retry for a look
#: that could not answer, and no more.
INSPECTION_MAX_TRAVEL_M = 8.0
INSPECTION_MAX_S = 180.0
INSPECTION_MAX_ATTEMPTS = 2

#: Two inspections are of the same physical case when the places they test are
#: this close on the same map, whatever their requests call them. **The budget
#: follows the place, not the name**: a thing renamed by a merge, or the same
#: question regenerated as a new goal, lands on the case it already spent.
CASE_RADIUS_M = 0.5

#: How close to the place under test an inspection may be sent to stand. The
#: place is a hypothesis and may be wrong in either direction -- something may be
#: there that the map does not show, or nothing may be where the looks crossed --
#: so the viewpoint is never on top of it. Half a metre is the rover's own
#: footprint and then some.
INSPECTION_STANDOFF_M = 0.5

#: The tilts an inspection's look may be taken at: the two the bearing
#: calibration covers, level and twenty degrees up.
INSPECTION_TILTS = (0.0, 20.0)

DEFAULT_BUDGET: dict[str, Any] = {
    "seconds": RUN_MAX_S,
    "travel_m": RUN_MAX_TRAVEL_M,
    "actions": RUN_MAX_ACTIONS,
    "failures": RUN_MAX_FAILURES,
    "permit_ttl_s": PERMIT_TTL_S,
    # No safe area unless a person declares one when they enable the run. None
    # means "the map is the boundary", which is not nothing: every goal still
    # has to be reachable over floor the mapper has confirmed is free.
    "geofence": None,
}

#: What an autonomous run may ask the rover to do. **Three operations, and the
#: list is short on purpose.**
#:
#: `drive_to` is Nav2 planning a route to a point the scorer chose, which is the
#: existing bounded move the console and the voice model already use.
#: `world_inspect` is one look, recorded, exactly as the rover's own looking
#: loop takes it. `stop` is the ordinary stop.
#:
#: What is deliberately *not* here is as much of the design as what is:
#:
#: - `explore` -- the rover's own frontier run -- is left out while
#:   [R-NAV-6](../docs/requirements/navigation.md#r-nav-6) is failing: a rover
#:   ringed by unmapped floor retires the whole rim on arriving without having
#:   moved. The executive drives to one frontier viewpoint at a time instead,
#:   which keeps every movement attributable to one goal and re-decides after
#:   each.
#: - Aiming the gimbal is left out because rest is the only pan angle this
#:   rover's bearings are calibrated at, and the chassis heading that
#:   `drive_to` already takes is what points the camera at the thing. A look
#:   may name a tilt, and only level or twenty up -- the two the calibration
#:   covers -- because the depth camera sees nothing below the camera's own
#:   height at twenty up, and an inspection of something on the floor needs it.
#: - `run_script` and `start_script` are left out because the design says so:
#:   rover-side scripts are process isolation and not a sandbox, and they are
#:   not the representation for anything the rover chooses by itself.
#:
#: Either of the first two may carry an `inspection`: a hypothesis check
#: (R-AUT-12) with its own case, place and limits, which `check` enforces at
#: dispatch and `inspection_over` during the drive.
ACTIONS: dict[str, tuple[str, ...]] = {
    "drive_to": ("x_m", "y_m"),
    "world_inspect": (),
    "stop": (),
}

#: The actions that move the wheels, which are the ones a geofence, a pose and a
#: map identity have to be checked for. `world_inspect` turns nothing and needs
#: none of them; refusing a look because the pose is untrusted would stop the
#: rover recording evidence at the moment it is most worth having.
DRIVING_ACTIONS = frozenset({"drive_to"})


def _inspection_limits(asked: Any) -> dict[str, float] | str:
    """An inspection's declared limits, checked against the ceilings, or why not.

    All three are required, finite and positive: a limit that can be left out,
    or asked for as zero, is not a limit, which is the same reasoning a run's
    budget is held to in `Permission.enable`.
    """
    asked = asked if isinstance(asked, dict) else {}
    out: dict[str, float] = {}
    for name, ceiling in (("travel_m", INSPECTION_MAX_TRAVEL_M),
                          ("seconds", INSPECTION_MAX_S),
                          ("attempts", INSPECTION_MAX_ATTEMPTS)):
        value = asked.get(name)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return f"an inspection's {name} must be a finite positive number"
        if value != value or value <= 0 or value == float("inf"):
            return f"an inspection's {name} must be a finite positive number"
        if float(value) > ceiling:
            return f"an inspection's {name} may be at most {ceiling:g}"
        out[name] = float(value)
    return out


def _apart(one: dict[str, Any] | None, other: dict[str, Any] | None) -> float | None:
    """How far apart two places on the map are, or None if either is unsaid."""
    if not one or not other:
        return None
    try:
        return ((float(one["x_m"]) - float(other["x_m"])) ** 2
                + (float(one["y_m"]) - float(other["y_m"])) ** 2) ** 0.5
    except (KeyError, TypeError, ValueError):
        return None


class Verdict(NamedTuple):
    """Whether an action may be dispatched, and the sentence saying why not.

    `code` is the short name the record and the tests use -- a reason a person
    reads changes wording, and a test written against the wording would fail
    for a rewrite. `why` is the sentence, because a refusal that reaches the
    executive has to explain itself to whoever reads the episode afterwards.
    """

    ok: bool
    code: str = ""
    why: str = ""
    #: Set when the request repeats one already dispatched: what happened the
    #: first time, so the caller can carry on rather than move twice.
    already: dict[str, Any] | None = None


def fence_breach(goal: dict[str, Any] | None,
                  fence: dict[str, Any] | None, margin_m: float = 0.0) -> str:
    """Whether a place is outside the configured safe area. '' when it is not.

    A circle -- `{"x_m", "y_m", "radius_m"}` -- or a box of any of
    `min_x_m`/`max_x_m`/`min_y_m`/`max_y_m`. Both shapes, because the
    pre-cleared area in a real room is a rectangle and the one somebody
    describes over a radio is a circle.

    `autonomy/scoring.py` vetoes a candidate outside the fence before it is ever
    chosen, and imports this function to do it: an advisory copy that had
    drifted from the enforcing one would refuse different goals in the record
    than the rover refuses in the room.
    """
    if not fence or not goal:
        return ""
    x, y = goal.get("x_m"), goal.get("y_m")
    if x is None or y is None:
        return ""
    x, y = float(x), float(y)
    if fence.get("radius_m") is not None:
        gap = ((x - float(fence.get("x_m", 0.0))) ** 2
               + (y - float(fence.get("y_m", 0.0))) ** 2) ** 0.5
        if gap > float(fence["radius_m"]) - margin_m:
            return (f"it is {gap:.1f} m from the middle of the safe area, "
                    f"which reaches {float(fence['radius_m']):.1f} m")
        return ""
    for low, high, value, axis in ((fence.get("min_x_m"), fence.get("max_x_m"),
                                    x, "x"),
                                   (fence.get("min_y_m"), fence.get("max_y_m"),
                                    y, "y")):
        if low is not None and value < float(low) + margin_m:
            return f"its {axis} is outside the safe area"
        if high is not None and value > float(high) - margin_m:
            return f"its {axis} is outside the safe area"
    return ""


class Run:
    """One bounded stretch of autonomy: what it may spend, and what it has.

    Opened by a person and closed by the first of its budgets to run out, a
    fault, or somebody stopping the rover. It is never reopened -- resuming
    after a stop is a new run with a new identifier, which is what makes "the
    rover stopped and started again" visible in the record rather than
    something to infer from a gap.
    """

    def __init__(self, run_id: str, *, by: str, why: str,
                 budget: dict[str, Any], at: float, wall: float) -> None:
        self.id = run_id
        self.by = by
        self.why = why
        self.budget = dict(budget)
        self.opened_at = at
        self.opened_wall = wall
        self.travel_m = 0.0
        self.actions = 0
        self.failures = 0
        self.ended = ""
        self.ended_at: float | None = None

    def spent(self, now: float) -> dict[str, Any]:
        return {"seconds": round(now - self.opened_at, 1),
                "travel_m": round(self.travel_m, 2),
                "actions": self.actions,
                "consecutive_failures": self.failures}

    def over(self, now: float) -> str:
        """Which budget has run out, as a sentence, or '' while none has."""
        seconds = float(self.budget.get("seconds") or 0.0)
        if seconds and now - self.opened_at >= seconds:
            return (f"the run's {seconds / 60:.0f} minutes are up "
                    f"({now - self.opened_at:.0f} s)")
        travel = float(self.budget.get("travel_m") or 0.0)
        if travel and self.travel_m >= travel:
            return (f"the run has driven {self.travel_m:.1f} m of its "
                    f"{travel:.0f} m")
        actions = int(self.budget.get("actions") or 0)
        if actions and self.actions >= actions:
            return f"the run has used all {actions} of its actions"
        failures = int(self.budget.get("failures") or 0)
        if failures and self.failures >= failures:
            return (f"{self.failures} actions in a row failed, which is the "
                    f"most this run allows")
        return ""

    def as_dict(self, now: float) -> dict[str, Any]:
        return {"id": self.id, "by": self.by, "why": self.why,
                "opened_at": self.opened_wall,
                "budget": dict(self.budget), "spent": self.spent(now),
                "ended": self.ended}


class Permission:
    """The daemon's account of who may move the rover by itself.

    `clock` is injected so that the tests can drive expiry without sleeping and
    the daemon can pass `time.monotonic`; `wall` is the second clock, used only
    for the human-readable stamps a status reply carries, because a monotonic
    reading means nothing to the person reading it.
    """

    def __init__(self, clock: Callable[[], float] = time.monotonic,
                 wall: Callable[[], float] = time.time,
                 boot: str | None = None) -> None:
        self.clock = clock
        self.wall = wall
        #: Minted per process. It goes into every run and permit identifier, so
        #: a permit issued before a restart cannot be mistaken for one issued
        #: after it even if the counters happen to line up.
        self.boot = boot or os.urandom(4).hex()
        self.runs = 0
        self.run: Run | None = None
        self.permit: dict[str, Any] | None = None
        #: Why autonomy is latched off, or None. Starts as None rather than as
        #: a latch: a fresh daemon has no run either, so autonomy is off for the
        #: honest reason -- nobody has enabled it -- rather than because of a
        #: stop that never happened.
        self.latch: dict[str, Any] | None = None
        #: Every action this run dispatched, by identifier, so that a repeat is
        #: answered rather than performed. Cleared when a run opens: identifiers
        #: carry the episode, and an episode belongs to one run.
        self.actions: dict[str, dict[str, Any]] = {}
        #: The last thing the run did, for a status a person reads.
        self.doing: dict[str, Any] | None = None
        #: Where the rover was at the previous tick, for accumulating travel.
        self._was: tuple[float, float] | None = None
        self._map_id: str | None = None
        #: Every physical case an inspection has been spent on, kept across runs
        #: for the life of this process, so that closing one run and opening
        #: another does not hand a case its attempts back. See `CASE_RADIUS_M`.
        self.cases: list[dict[str, Any]] = []

    # --- what a person does -------------------------------------------------

    def enable(self, *, by: str, why: str = "",
               budget: dict[str, Any] | None = None) -> dict[str, Any]:
        """Open a run. **This is the human act, and the only thing that clears
        a stop.**

        Refused while a run is already open, rather than extending it: a second
        press of the button must not double the budget of the run already going,
        because the budget is the whole of what makes the run bounded.
        """
        now = self.clock()
        if self.run is not None and not self.run.ended:
            return {"ok": False,
                    "error": "a run is already open; stop it before opening "
                             "another",
                    "run": self.run.as_dict(now)}
        asked = dict(DEFAULT_BUDGET)
        for name, value in (budget or {}).items():
            if name not in DEFAULT_BUDGET:
                return {"ok": False, "error": f"no such budget: {name}"}
            asked[name] = value
        # Every budget is a ceiling as well as a default. A person may ask for a
        # shorter run than the standing limit and not a longer one, so that the
        # limits argued for above cannot be talked out of by whoever is typing.
        for name in ("seconds", "travel_m", "actions", "failures",
                     "permit_ttl_s"):
            if asked[name] is None or float(asked[name]) <= 0:
                # Zero is refused rather than read as "no limit", which is what
                # `Run.over` would make of it: a budget that can be switched off
                # by asking for none of it is not a limit at all.
                return {"ok": False,
                        "error": f"{name} must be a positive number"}
            if float(asked[name]) > float(DEFAULT_BUDGET[name]):
                return {"ok": False,
                        "error": f"{name} may be at most "
                                 f"{DEFAULT_BUDGET[name]}"}
        self.runs += 1
        self.run = Run(f"run/{self.boot}/{self.runs}", by=by, why=why,
                       budget=asked, at=now, wall=self.wall())
        self.latch = None
        self.permit = None
        self.actions = {}
        self.doing = None
        self._was = None
        self._map_id = None
        return {"ok": True, "run": self.run.as_dict(now),
                "note": "autonomy is enabled for this run only; stopping the "
                        "rover ends it and it cannot be reopened from inside"}

    def stop(self, *, by: str, why: str = "") -> dict[str, Any]:
        """A person stopped the rover. Latch autonomy off and end any run.

        Never refused and never conditional. It is called for the console's stop
        button, for the voice model's `stop_driving`, and for any manual move
        arriving while a run is open -- taking the wheels by hand is a stop as
        far as autonomy is concerned, and a rover that let autonomy carry on
        around a person's driving would be arguing with them.
        """
        now = self.clock()
        ended = None
        if self.run is not None and not self.run.ended:
            ended = self.run.id
            self.run.ended = f"stopped by {by}" + (f": {why}" if why else "")
            self.run.ended_at = now
        self.latch = {"at": self.wall(), "by": by,
                      "why": why or f"{by} stopped the rover"}
        self.permit = None
        return {"ok": True, "latched": True, "ended_run": ended,
                "why": self.latch["why"]}

    # --- what the executive does --------------------------------------------

    def grant(self, run_id: str, ttl_s: float | None = None) -> dict[str, Any]:
        """Give out permission to act, or renew it. Never opens a run.

        The run identifier is required and checked rather than implied, so that
        an executive which slept through a stop and a re-enable cannot pick up
        the new run's authority believing it is still in its own.
        """
        now = self.clock()
        if self.latch is not None:
            return {"ok": False, "error": self.latch["why"],
                    "latched": True}
        run = self.run
        if run is None or run.ended:
            return {"ok": False,
                    "error": "autonomy is not enabled" if run is None else
                             f"the run ended: {run.ended}"}
        if run_id != run.id:
            return {"ok": False,
                    "error": f"{run_id} is not the run that is open"}
        over = run.over(now)
        if over:
            return {"ok": False, "error": over}
        ttl = float(ttl_s or run.budget["permit_ttl_s"])
        ttl = min(ttl, float(run.budget["permit_ttl_s"]))
        if self.permit is None:
            self.permit = {"id": f"permit/{self.boot}/{self.runs}",
                           "run": run.id, "granted_at": now, "renewals": 0}
        else:
            self.permit["renewals"] += 1
        self.permit["expires_at"] = now + ttl
        return {"ok": True, "permit": self.permit["id"],
                "expires_in_s": round(ttl, 1),
                "renewals": self.permit["renewals"],
                "run": run.as_dict(now)}

    def check(self, *, permit: str, action: str, action_id: str,
              episode: str, params: dict[str, Any] | None = None,
              conditions: dict[str, Any] | None = None) -> Verdict:
        """May this action be dispatched, right now, by this permit?

        Everything is re-checked here rather than trusted from the moment the
        permit was granted, because between the grant and the dispatch the
        battery drains, the map is replaced, the pose stops being believable and
        somebody presses stop -- and dispatch is the last moment any of that can
        be noticed before the wheels turn.
        """
        now = self.clock()
        facts = dict(conditions or {})
        params = dict(params or {})

        if self.latch is not None:
            return Verdict(False, "latched", self.latch["why"])
        run = self.run
        if run is None or run.ended:
            return Verdict(False, "no run",
                           "autonomy is not enabled" if run is None else
                           f"the run ended: {run.ended}")
        if self.permit is None or permit != self.permit["id"]:
            return Verdict(False, "no permit",
                           "this is not the permission that is current; ask "
                           "for one and try again")
        if now >= float(self.permit["expires_at"]):
            return Verdict(False, "permit expired",
                           f"the permission ran out "
                           f"{now - float(self.permit['expires_at']):.1f} s ago")
        over = run.over(now)
        if over:
            return Verdict(False, "budget", over)

        done = self.actions.get(action_id)
        if done is not None:
            return Verdict(False, "already done",
                           f"{action_id} was dispatched "
                           f"{now - done['at']:.1f} s ago and is not repeated",
                           already=dict(done))
        if not action_id or not episode:
            return Verdict(False, "unattributable",
                           "every autonomous action carries the episode and "
                           "the action it belongs to")
        if action not in ACTIONS:
            return Verdict(False, "not an autonomy action",
                           f"{action} is not one of the operations autonomy "
                           f"may ask for: {', '.join(sorted(ACTIONS))}")
        missing = [name for name in ACTIONS[action] if params.get(name) is None]
        if missing:
            return Verdict(False, "incomplete",
                           f"{action} needs {', '.join(missing)}")
        tilt = params.get("tilt_deg")
        if tilt is not None and (not isinstance(tilt, (int, float))
                                 or float(tilt) not in INSPECTION_TILTS):
            return Verdict(False, "tilt",
                           f"a look may be taken at tilt "
                           f"{' or '.join(f'{t:.0f}' for t in INSPECTION_TILTS)}"
                           f", which is what the bearings are calibrated at")
        if params.get("inspection") is not None:
            refused = self._inspection_refused(action, params, facts, episode,
                                               now)
            if refused is not None:
                return refused

        # No battery check: an autonomous drive is conditioned on the battery as
        # every other drive is. See the note where `BATTERY_FLOOR_V` used to be.
        if action in DRIVING_ACTIONS:
            if not facts.get("pose_trusted"):
                return Verdict(False, "pose",
                               "the rover does not know where it is on the map "
                               "well enough to be sent to a place on it")
            if facts.get("map_settled") is False:
                return Verdict(False, "map",
                               "the map has not settled since the last "
                               "restart, so its coordinates cannot be trusted")
            named = params.get("map_id")
            live = facts.get("map_id")
            if named is not None and live is not None and named != live:
                return Verdict(False, "stale map",
                               f"this goal was chosen on map {named} and the "
                               f"rover is on {live}")
            breach = fence_breach(params, run.budget.get("geofence"))
            if breach:
                return Verdict(False, "outside the safe area", breach)
        return Verdict(True)

    # --- what one inspection may spend ----------------------------------------

    def _inspection_refused(self, action: str, params: dict[str, Any],
                            facts: dict[str, Any], episode: str,
                            now: float) -> Verdict | None:
        """Why this inspection's step may not go, or None when it may.

        **One attempt is one episode**, and an attempt's travel and time are
        counted from its first step: the drive to the viewpoint, or the look if
        it needed none. A new episode on a case that has had its attempts is
        refused whatever it calls the case, and the limits a case was first given
        can be narrowed by a later request and never widened.
        """
        asked = params.get("inspection")
        if not isinstance(asked, dict):
            return Verdict(False, "inspection",
                           "an inspection says which case it tests, where, and "
                           "what it may spend")
        target = asked.get("target") or {}
        if not asked.get("case") or target.get("x_m") is None \
                or target.get("y_m") is None:
            return Verdict(False, "inspection",
                           "an inspection names its case and the place it tests")
        limits = _inspection_limits(asked.get("limits"))
        if isinstance(limits, str):
            return Verdict(False, "inspection limits", limits)
        case = self._case_for(asked, facts.get("map_id"))
        if case is not None:
            limits = {name: min(value, case["limits"][name])
                      for name, value in limits.items()}
        attempts = [] if case is None else case["attempts"]
        mine = next((one for one in attempts if one["episode"] == episode), None)
        if mine is None:
            if len(attempts) >= limits["attempts"]:
                return Verdict(False, "inspection exhausted",
                               f"this place has had {len(attempts)} inspection "
                               f"attempts, which is all it is allowed")
            spent_s, spent_m = 0.0, 0.0
        else:
            if mine.get("run") != (self.run.id if self.run else None):
                return Verdict(False, "inspection exhausted",
                               "an attempt does not carry on into another run")
            spent_s = now - mine["started_at"]
            spent_m = (self.run.travel_m if self.run else 0.0) - mine["travel_at"]
            if spent_s >= limits["seconds"]:
                return Verdict(False, "inspection exhausted",
                               f"this attempt has used its "
                               f"{limits['seconds']:.0f} s ({spent_s:.0f} s)")
            if spent_m >= limits["travel_m"]:
                return Verdict(False, "inspection exhausted",
                               f"this attempt has driven its "
                               f"{limits['travel_m']:.1f} m ({spent_m:.1f} m)")
        if action in DRIVING_ACTIONS:
            gap = _apart(params, target)
            if gap is not None and gap < INSPECTION_STANDOFF_M:
                return Verdict(False, "inspection standoff",
                               f"the viewpoint is {gap:.2f} m from the place "
                               f"under test, which may be wrong in either "
                               f"direction; it may be no nearer than "
                               f"{INSPECTION_STANDOFF_M:.1f} m")
            where = facts.get("where")
            if where is not None:
                away = _apart(params, {"x_m": where[0], "y_m": where[1]})
                left = limits["travel_m"] - spent_m
                if away is not None and away > left:
                    return Verdict(False, "inspection travel",
                                   f"the viewpoint is {away:.1f} m away and "
                                   f"this attempt may drive {left:.1f} m more")
        return None

    def _case_for(self, asked: dict[str, Any],
                  map_id: str | None) -> dict[str, Any] | None:
        """The case this request is of: by its name, or by the place it tests."""
        target = asked.get("target") or {}
        for case in self.cases:
            if case["case"] == asked.get("case"):
                return case
            if map_id is not None and case.get("map_id") not in (None, map_id):
                continue
            gap = _apart(target, case["target"])
            if gap is not None and gap <= CASE_RADIUS_M:
                return case
        return None

    def _inspection_began(self, params: dict[str, Any], episode: str,
                          action_id: str, map_id: str | None) -> None:
        """Count an inspection's step against its case and its attempt."""
        asked = params.get("inspection")
        if not isinstance(asked, dict):
            return
        limits = _inspection_limits(asked.get("limits"))
        if isinstance(limits, str):                            # pragma: no cover
            return
        case = self._case_for(asked, map_id)
        if case is None:
            target = asked.get("target") or {}
            case = {"case": str(asked.get("case")), "map_id": map_id,
                    "target": {"x_m": float(target["x_m"]),
                               "y_m": float(target["y_m"])},
                    "limits": limits, "attempts": []}
            self.cases.append(case)
        else:
            case["limits"] = {name: min(value, case["limits"][name])
                              for name, value in limits.items()}
        attempt = next((one for one in case["attempts"]
                        if one["episode"] == episode), None)
        if attempt is None:
            attempt = {"episode": episode, "run": self.run.id if self.run else None,
                       "started_at": self.clock(),
                       "travel_at": self.run.travel_m if self.run else 0.0,
                       "actions": []}
            case["attempts"].append(attempt)
        attempt["actions"].append(action_id)
        record = self.actions.get(action_id)
        if record is not None:
            record["case"] = case["case"]

    def inspection_over(self) -> str:
        """Why the inspection step now running must stop, or ''.

        The daemon's watchdog asks this on every tick, so an attempt's travel and
        time are enforced during the drive and not only at dispatch: a route
        that turns out longer than the straight line is stopped at its limit,
        and the step is failed with the reason rather than the run ended.
        """
        doing = self.doing
        if not doing or doing.get("ok") is not None or not doing.get("case"):
            return ""
        case = next((one for one in self.cases
                     if one["case"] == doing["case"]), None)
        if case is None:                                       # pragma: no cover
            return ""
        attempt = next((one for one in case["attempts"]
                        if doing["id"] in one["actions"]), None)
        if attempt is None:                                    # pragma: no cover
            return ""
        spent_s = self.clock() - attempt["started_at"]
        spent_m = (self.run.travel_m if self.run else 0.0) - attempt["travel_at"]
        if spent_s >= case["limits"]["seconds"]:
            return (f"the inspection's {case['limits']['seconds']:.0f} s ran out "
                    f"while {doing.get('action')} was running")
        if spent_m >= case["limits"]["travel_m"]:
            return (f"the inspection drove its {case['limits']['travel_m']:.1f} m "
                    f"({spent_m:.1f} m) before arriving")
        return ""

    def limit_reached(self, action_id: str, why: str) -> None:
        """Fail a step that an inspection's limit stopped, before the wheels do.

        Marked first, so that the stop it is followed by -- which the navigator
        reports as an ordinary `stopped` -- cannot be mistaken for success.
        """
        record = self.actions.get(action_id)
        if record is not None:
            record["limited"] = why
        self.finished(action_id, ok=False, detail=why)

    # --- what the daemon does around them -----------------------------------

    def began(self, action_id: str, action: str,
              params: dict[str, Any] | None = None,
              episode: str = "") -> None:
        """Record that an action was dispatched. Counts against the run."""
        now = self.clock()
        record = {"action": action, "params": dict(params or {}),
                  "episode": episode, "at": now, "wall": self.wall(),
                  "ok": None, "detail": "", "travel_m": 0.0}
        self.actions[action_id] = record
        if (params or {}).get("inspection") is not None:
            self._inspection_began(dict(params or {}), episode, action_id,
                                   (params or {}).get("map_id") or self._map_id)
        self.doing = {"id": action_id, **record}
        if self.run is not None:
            self.run.actions += 1

    def finished(self, action_id: str, *, ok: bool, detail: str = "",
                 travel_m: float | None = None) -> None:
        """Record how it went. Consecutive failures are counted here.

        Travel is *not* added from this: distance is accumulated from where the
        rover actually is, tick by tick, because an action that reports nothing
        -- because it was killed, or because nobody waited for it -- would
        otherwise cost the run nothing at all.
        """
        record = self.actions.get(action_id)
        if record is not None:
            if record.get("limited") and record.get("ok") is not None:
                # Already failed by the inspection limit that stopped it; the
                # navigator's own word for the stop comes second and is not news.
                return
            record["ok"] = bool(ok)
            record["detail"] = detail
            if travel_m is not None:
                record["travel_m"] = float(travel_m)
        if self.doing is not None and self.doing.get("id") == action_id:
            self.doing = {**self.doing, "ok": bool(ok), "detail": detail}
        if self.run is None:
            return
        # Recovery stopping is not progress and cannot forgive the failed goal.
        if record is not None and record.get("action") != "stop":
            self.run.failures = 0 if ok else self.run.failures + 1

    def moved(self, where: tuple[float, float] | None,
              map_id: str | None = None, elapsed_s: float | None = None) -> str:
        """Account for where the rover has got to. Returns a fault or ''.

        Called on every tick of the daemon's watchdog while a run is open. Two
        things come out of it: the run's travel budget is spent by the distance
        actually covered -- including during a move nobody is waiting for, which
        is the case the plan singles out -- and a pose that jumps further than
        the chassis could have driven since the last tick is reported as the
        fault it is. `elapsed_s` is how long ago that tick was, because a tick
        delayed by a slow bridge must not turn ordinary driving into a jump.
        """
        if self.run is None or self.run.ended:
            return ""
        if map_id is not None and self._map_id is not None \
                and map_id != self._map_id:
            was, self._map_id, self._was = self._map_id, map_id, where
            return f"the map changed under the run, from {was} to {map_id}"
        if map_id is not None:
            self._map_id = map_id
        if where is None:
            # Not a fault by itself: the bridge drops a status now and then, and
            # the pose gate at dispatch is what refuses to send the rover
            # anywhere without one. Forget where it was, so that the first
            # reading afterwards starts a fresh leg instead of measuring the gap.
            self._was = None
            return ""
        if self._was is not None:
            step = ((where[0] - self._was[0]) ** 2
                    + (where[1] - self._was[1]) ** 2) ** 0.5
            gap = float(TICK_S if elapsed_s is None else max(0.0, elapsed_s))
            if step > POSE_JUMP_M + CHASSIS_MAX_SPEED_MS * gap:
                self._was = where
                return (f"the rover's place on the map jumped {step:.2f} m in "
                        f"{gap:.1f} s, which it cannot have driven")
            self.run.travel_m += step
        self._was = where
        return ""

    def due(self, conditions: dict[str, Any] | None = None) -> str:
        """Why the run must end now, or '' while it may continue.

        The permit is part of this and it is the important part: an executive
        that has been killed, has hung, or has lost its connection stops
        renewing, and this is what notices. Nav2 is perfectly healthy in that
        situation and would go on driving to the goal it was given.
        """
        now = self.clock()
        run = self.run
        if run is None or run.ended:
            return ""
        facts = dict(conditions or {})
        fence = run.budget.get("geofence")
        if facts.get("driving") and fence:
            where = facts.get("where")
            if where is None or not facts.get("pose_trusted"):
                return "the safe area cannot be checked without a trusted position"
            breach = fence_breach({"x_m": where[0], "y_m": where[1]}, fence,
                                  margin_m=FENCE_MARGIN_M)
            if breach:
                return "the rover reached the safe area stopping margin: " + breach
        over = run.over(now)
        if over:
            return over
        if self.permit is not None and now >= float(self.permit["expires_at"]):
            return (f"the permission ran out "
                    f"{now - float(self.permit['expires_at']):.0f} s ago and "
                    f"nothing renewed it")
        return ""

    def end_run(self, why: str) -> dict[str, Any]:
        """End the run without latching. Not a stop: nobody asked for this.

        A run that has spent its budget, or lost its executive, is over -- and
        that is different from a person stopping the rover, which is why this
        does not set the latch. It makes no practical difference to what happens
        next, because only a person can open the next run either way; it makes
        every difference to the record, where "the rover ran out of minutes" and
        "somebody pressed stop" must not read alike.
        """
        now = self.clock()
        run = self.run
        if run is None or run.ended:
            return {"ok": False, "error": "no run is open"}
        run.ended = why
        run.ended_at = now
        self.permit = None
        return {"ok": True, "ended_run": run.id, "why": why}

    # --- what anybody may read ----------------------------------------------

    def status(self) -> dict[str, Any]:
        """The whole state, for a status call, a console or an episode.

        Written to be read by somebody who has just found the rover standing
        still and wants to know whether it is about to move.
        """
        now = self.clock()
        run = self.run
        permit = self.permit
        live = (run is not None and not run.ended and self.latch is None)
        return {
            "enabled": bool(live),
            "boot": self.boot,
            "latched": self.latch is not None,
            "latch": dict(self.latch) if self.latch else None,
            "run": None if run is None else run.as_dict(now),
            "permit": None if permit is None else {
                "id": permit["id"], "run": permit["run"],
                "renewals": permit["renewals"],
                "expires_in_s": round(float(permit["expires_at"]) - now, 1)},
            "doing": dict(self.doing) if self.doing else None,
            "actions_recorded": len(self.actions),
            "inspections": [{"case": case["case"], "map_id": case.get("map_id"),
                             "target": case["target"], "limits": case["limits"],
                             "attempts": len(case["attempts"])}
                            for case in self.cases[-20:]],
            "why": self._why(live, run),
        }

    def _why(self, live: bool, run: Run | None) -> str:
        if self.latch is not None:
            return self.latch["why"]
        if run is None:
            return "autonomy has not been enabled since this daemon started"
        if run.ended:
            return f"the last run ended: {run.ended}"
        if live:
            over = run.over(self.clock())
            return over or "a run is open"
        return "autonomy is off"                               # pragma: no cover
