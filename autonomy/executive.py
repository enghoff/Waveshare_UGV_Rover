#!/usr/bin/env python3
"""The loop that carries out what the deliberation chose, and stops when told.

    ssh orin 'cd ~/ugv/autonomy && python3 executive.py'

**It has no authority of its own.** It holds a `client.Acting`, which cannot
call `drive_to`, `explore` or `world_inspect` at all: every physical action goes
out as `autonomy_act`, carrying a permit the daemon issued, the episode it
belongs to and an identifier for the action itself. The daemon re-checks the
permit, the budgets, the battery, the pose and the map before anything turns,
refuses a repeat outright, and takes the wheels back on its own if this process
hangs or dies. See [rover_daemon/permission.py](permission.py), which is the
same file deployed beside this one.

So there are two independent reasons the rover stops: this loop deciding to, and
the daemon noticing that it has not. Only the second one survives this process
being killed, which is why it is the one that matters.

## The states, and what each is allowed to do

```text
IDLE -> SELECT -> PLAN -> EXECUTE -> EVALUATE -> IDLE
                            |          |
                            +-> ABORT <-+
```

- **IDLE** renews the permission and reads the rover. If the run has ended --
  budget, battery, a person's stop -- the loop is over; it does not wait for
  another, because opening one is a person's act.
- **SELECT** is `scoring.consider` with authority, which is the same
  deliberation the shadow runs record and the same arithmetic, differing only in
  that the answer can now be acted on.
- **PLAN** turns the chosen goal into a short list of admitted operations and
  checks every one of them against the daemon's own list *before* the first is
  dispatched. A plan with an operation nobody admits is abandoned rather than
  half-run.
- **EXECUTE** dispatches them in order, waiting for the ones that are not
  finished when the call returns. A move nobody waits for is finished when the
  wheels stop, so the wait is a poll of the daemon's own account of it.
- **EVALUATE** reads the rover again and records what the attempt actually
  changed -- how far it drove, whether the thing is better placed, whether there
  is less unmapped floor -- rather than what it hoped to.
- **ABORT** is any of those going wrong: a refusal, a failure, a timeout, the
  daemon losing the run underneath. It stops the rover if it still may, and
  closes the episode `interrupted` with the reason.

## What is in the record

One episode per turn of the loop, and the deliberation is its first half: the
candidates with their scores, the choice and why, then a `call` event per
action with what came back, what changed, and how it ended. That is what makes
"every movement is attributable to one episode and one goal" a property of the
record rather than a promise.

## What this does not do

There is no model here, and that is not an omission. Nothing in this loop asks
anything to be curious for it -- `scoring.py` is arithmetic over the situation
-- so an unreachable model cannot start a physical action, cannot stop one, and
cannot change what is chosen. The rover speaking about what it did is a separate
path that runs after the wheels stop; losing it costs the announcement and
nothing else.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from typing import Any, Callable

import client as client_mod
import cooling
import decide as decide_mod
import events
import goals
import hypotheses
import mapgrid
import refs
import scoring
import situation as situation_mod
import store as store_mod
import summary as summary_mod

# The daemon's own rules, for the one thing this needs them for: refusing to
# plan an operation the rover would not admit. Loaded the way `scoring.py` and
# `mapgrid.py` load their shared modules, and for the same reason -- one file in
# the repository, deployed into both components.
try:
    import permission
except ImportError:                                            # pragma: no cover
    import importlib.util

    _path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         os.pardir, "rover_daemon", "permission.py")
    _spec = importlib.util.spec_from_file_location("permission", _path)
    if _spec is None or _spec.loader is None:
        raise
    permission = importlib.util.module_from_spec(_spec)
    sys.modules["permission"] = permission
    _spec.loader.exec_module(permission)

#: The trigger these episodes open with. Different from the shadow
#: deliberation's on purpose: "considered" and "considered and then did
#: something" are not the same occasion, and a reader scanning a day of episodes
#: should be able to tell them apart without opening either.
TRIGGER = "the rover chose what to do next and did it"

#: How often the permission is renewed while an action is in flight. Well inside
#: `permission.PERMIT_TTL_S`, so that several renewals in a row have to fail
#: before the daemon takes the wheels back -- one dropped poll is a busy rover,
#: not a dead executive.
RENEW_EVERY_S = 2.0

#: How long one dispatched action may take before the loop gives up on it. A
#: drive across this house is a minute or two at 0.35 m/s with planning and
#: recoveries in it; four minutes is long enough that a slow arrival is not
#: mistaken for a hang, and short enough that a run of fifteen minutes is not
#: spent entirely inside one goal. The daemon's budgets bound it regardless --
#: this is the loop noticing first, so that the reason lands in the episode.
ACTION_TIMEOUT_S = 240.0

#: How long the executive waits for the daemon to answer a look. The daemon
#: lets a run's look wait up to 9 s for the rover's own to finish
#: (`rover_autonomy.AUTONOMY_LOOK_WAIT_S`) and the look itself is under a
#: second, so 13 s covers both and stays inside the 15 s permission renewed
#: just before it -- nothing renews it while this one call is out.
LOOK_CALL_TIMEOUT_S = 13.0

#: A look refused because the rover's own was still running is asked again,
#: this many times, this far apart. The daemon already waits up to 9 s for it;
#: on 2026-10-07 (M3 session 11) one ran past 11 s and the run's look was
#: refused and counted as a failed goal. Each retry is a new dispatch with its
#: own action id, recorded like any other; a look moves nothing.
LOOK_BUSY_RETRIES = 2
LOOK_BUSY_WAIT_S = 3.0

#: How long to stand still after a turn of the loop that chose nothing. The
#: rover is parked, its map is not changing and neither is the answer, so this is
#: about not filling the record with identical refusals rather than about
#: responsiveness -- and the shadow runs already deliberate once a minute.
IDLE_S = 30.0

#: How long a nap inside that wait may be. **Standing still is not being dead**,
#: and the difference is a renewal: a single thirty-second sleep outlasts the
#: fifteen-second lease, so the daemon would take the wheels back from an
#: executive that was merely waiting for the room to change. Found on the rover
#: on 2026-09-08, in the first turn the real daemon ever answered: one idle turn
#: ended the run. A third of the lease leaves two missed naps of margin.
IDLE_NAP_S = permission.PERMIT_TTL_S / 3.0

#: The trigger and the goal of the last episode of a run that ran out of things
#: worth doing, which drives back to where the run started.
RETURN_TRIGGER = "the run had nothing left worth doing and went back to where it started"
RETURN_GOAL = "return_to_start"

#: How near where the run started counts as being there already: the
#: navigator's own arrival tolerance (`xy_goal_tolerance`), inside which a drive
#: would arrive without moving.
HOME_NEAR_M = 0.22

#: How long a run waits while something about the rover refuses every goal --
#: a camera that has failed, a pose it does not trust -- before it goes back
#: and ends. Such a thing may clear, so it is waited out for a while; not for
#: ever, because a rover standing about drains its battery for nothing.
GATED_GIVE_UP_S = 120.0

#: Gates that mean the run itself is over or was never there. The loop ends on
#: those by itself; they are not a rover that cannot act just now.
RUN_GATES = frozenset({"stopped", "autonomy not enabled",
                       "no movement authority"})



#: Whether a geometry goal's look names the thing it is aimed at, so that the
#: world state files the region at the aim to it (world_state/aimed.py). An
#: action that relies on identity, agreed by the owner for supervised runs on
#: 2026-10-08: docs/decisions/aimed-looks-file-to-their-target.md.
NAME_THE_TARGET = True

#: The role an M4 trial's re-look steps carry in the record: a turn on the spot
#: to face the thing and one aimed look at it, from where the rover stood when it
#: chose, taken before it drives to the viewpoint it chose. M4 scores the chosen
#: viewpoint's look against this one, each filed into the same copy of the store
#: (docs/decisions/m4-measures-where-things-are.md).
RELOOK = "relook"
#: How a re-look step may end that still ends the attempt. Anything else -- a
#: look that saw nothing, a turn navigation refused -- is a baseline that gained
#: nothing, and the attempt goes on to its chosen viewpoint.
RELOOK_ENDS = frozenset({"stopped", "run over", "timed out", "connection lost"})

class Aborted(Exception):
    """Raised inside a turn to end it. Carries the reason the episode closes
    with, because an abort whose reason is assembled later is an abort whose
    reason is a guess."""

    def __init__(self, why: str, code: str = "aborted") -> None:
        super().__init__(why)
        self.why, self.code = why, code


class Executive:
    """One session of bounded autonomy, from a run that is already open.

    `sleep` and `now` are injected for the same reason the permission's clock is:
    the checks drive a whole run through timeouts and expiry without waiting for
    any of it, and a loop that reached for `time.sleep` directly could only be
    tested by taking four minutes to do it.
    """

    def __init__(self, store: store_mod.EpisodeStore,
                 rover: client_mod.Acting,
                 weights: scoring.Weights = scoring.DEFAULT, *,
                 sleep: Callable[[float], None] = time.sleep,
                 now: Callable[[], float] = time.time,
                 log: Callable[[str], None] = print) -> None:
        self.store = store
        self.rover = rover
        self.weights = weights
        self.sleep = sleep
        self.now = now
        self.log = log
        self.run: str = ""
        self.permit: str = ""
        self.state = "IDLE"
        self.episode = ""
        #: Which map the situation was last read on, so that every request can
        #: name what it assumed and be refused rather than translated if the
        #: rover is on a different one by the time it arrives.
        self._map_id: str | None = None
        self.turns = 0
        self.acted = 0
        self.ended = ""
        #: Every action this executive dispatched, so that a summary can say
        #: what a session did without re-reading the whole record.
        self.actions: list[dict[str, Any]] = []
        #: Where the rover stood when the run was opened, as the daemon
        #: recorded it, so that a run with nothing left can go back there.
        self.start: dict[str, Any] | None = None
        #: Since when every turn has been refused by the rover's own state, or
        #: None while it is not.
        self._gated_since: float | None = None
        #: Set when the run went back and ended itself; the loop is over.
        self.finished = False
        #: What the run was opened for, if it is an M4 trial: `targets`, the
        #: records it may look at; `relook`, whether each attempt starts with a
        #: re-look from where the rover stands; `snapshot`, whether the world
        #: store is copied before each attempt. Empty for an ordinary run.
        self.trial: dict[str, Any] = {}

    # --- the loop -----------------------------------------------------------

    def attach(self) -> dict[str, Any]:
        """Find the run a person opened, or say why there is nothing to do.

        The executive cannot open one. That is the whole architecture in one
        method: this process starts, looks for authority it was given, and stops
        if it was not given any.
        """
        status = self._status()
        run = (status.get("run") or {}) if status.get("enabled") else {}
        if not run:
            return {"ok": False,
                    "error": status.get("why") or "autonomy is not enabled",
                    "status": status}
        self.run = str(run["id"])
        self.start = run.get("start") or None
        # **The trial comes from the run, not from this process.** The daemon
        # keeps what the run was opened with and says it in its status, so a
        # trial's targets are recorded where its run is, and an executive
        # started by the console's button can never find itself in one.
        self.trial = dict(status.get("trial") or {})
        if self.trial.get("targets") is not None:
            # A copy: the weights this executive was given may be shared.
            self.weights = scoring.Weights.from_dict({
                **self.weights.as_dict(),
                "trial_targets": sorted(str(one) for one in self.trial["targets"])})
        return {"ok": True, "run": run, "status": status}

    def loop(self, turns: int | None = None) -> dict[str, Any]:
        """Run until the run ends, or for a fixed number of turns.

        `turns` is for the checks and for a person who wants one goal carried
        out and then the rover left alone; the ordinary session ends because the
        daemon ended the run, which is the answer this returns.
        """
        while turns is None or self.turns < turns:
            if not self._alive():
                break
            if self.finished:
                break
            self.turns += 1
            try:
                self.once()
            except client_mod.Unreachable as exc:
                # The daemon went away under us. Nothing here can stop the rover
                # without it, and nothing needs to: the permission expires in
                # seconds and the daemon's own watchdog is what stops the wheels.
                self.ended = f"the daemon stopped answering: {exc}"
                break
        if not self.ended:
            self.ended = self._why_stopped()
        return self.summary()

    def once(self) -> dict[str, Any]:
        """One turn: decide, plan, do, and write down what happened.

        The permission is renewed before the rover is read, so that the
        situation the decision is made from carries the authority the decision
        will be acted under rather than the authority of a moment earlier. A
        renewal that fails is not raised here: the same refusal is in the
        `autonomy_status` the situation reads, and it belongs in the record as
        the gate that stopped the goal rather than as an exception.
        """
        self.state = "IDLE"
        try:
            self.renew()
        except Aborted as stop:
            self.ended = self.ended or stop.why
        here = self._read()
        self.state = "SELECT"
        # **Authority is what this component is, not what it currently has.**
        # The executive can act, so it says so, and the gate then refuses on
        # what the *rover* says -- a person's stop, a spent budget, a daemon too
        # old to have an opinion. Passing `False` when the permit had lapsed
        # would put "every call it may make is a read" into the record of a
        # rover that had simply been stopped, which is a different fact.
        got = decide_mod.deliberate(self.store, here, self.weights,
                                    authority=True, close=False,
                                    note="an autonomous turn: it chose this and "
                                         "carried it out under a permit from "
                                         "the daemon")
        episode, decision = got["episode"], got["decision"]
        self.episode = episode
        chose = decision.get("chose")
        if not chose:
            why = decision.get("why_nothing") or "nothing worth doing"
            self.store.close_episode(episode, "abandoned", detail=why)
            self.log(f"nothing to do: {why}")
            gates = [one for one in decision.get("gate") or []
                     if one.get("gate") not in RUN_GATES]
            if not decision.get("gate"):
                # **Nothing worth doing anywhere, so the run is over.** The
                # scorer has already looked further afield (`go_further`); what
                # is left is standing about on a draining battery, which the
                # owner asked on 2026-10-03 not to have.
                self.go_back(here, got["inputs"],
                             f"there was nothing left worth doing: {why}")
            elif gates:
                since = self._gated_since
                if since is None:
                    since = self._gated_since = self.now()
                if self.now() - since >= GATED_GIVE_UP_S:
                    self.go_back(here, got["inputs"],
                                 f"the rover could not act for "
                                 f"{self.now() - since:.0f} s: "
                                 + "; ".join(one["why"] for one in gates))
                else:
                    self.idle(IDLE_S)
            else:
                self.idle(IDLE_S)
            return {"episode": episode, "acted": False, "why": why}
        self._gated_since = None

        candidate = decision["preferred"]["candidate"]
        inspecting = candidate["type"] == hypotheses.GOAL_TYPE
        request: dict[str, Any] | None = None
        step: dict[str, Any] = {}
        try:
            self.state = "PLAN"
            plan = self.plan(candidate, here)
            if inspecting:
                request = self.freeze(episode, here, candidate)
            self.state = "EXECUTE"
            if (self.trial.get("snapshot")
                    and candidate["type"] == "improve_geometry"):
                self.snapshot(episode)
            looked: dict[str, Any] = {}
            for step in plan:
                if step.get("role") == RELOOK:
                    self.relook(episode, step, candidate)
                    continue
                looked = self.do(episode, step, candidate)
            if inspecting:
                self.state = "CHECK"
                self.check(episode, request, looked)
            self.state = "EVALUATE"
            after = self.evaluate(episode, here, candidate, looked)
            if request is not None and request.get("answered"):
                said = request["answered"]
                after["what"] = f"{said['outcome']}: {said.get('why')}"
        except Aborted as stop:
            self.state = "ABORT"
            if (step.get("action") == "drive_to"
                    and stop.code in ("failed", "timed out")):
                # Navigation could not get there: written down where the next
                # deliberation reads it, so the same place is not driven at
                # again. See `cooling.after_failed_drive`.
                places = decide_mod._loads(
                    self.store.marked(decide_mod.UNREACHABLE_MARK)) or []
                self.store.mark(decide_mod.UNREACHABLE_MARK, decide_mod._dumps(
                    cooling.after_failed_drive(
                        places, candidate["constraints"].get("goal"), stop.why,
                        now=self.now())))
            if request is not None and "answered" not in request:
                self.answered(episode, request, {
                    "outcome": "unresolved", "code": stop.code,
                    "why": f"the attempt ended before it could answer: "
                           f"{stop.why}"})
            self.give_up(episode, stop)
            return {"episode": episode, "acted": True, "why": stop.why,
                    "outcome": "interrupted"}
        self.state = "IDLE"
        self.store.close_episode(episode, "succeeded", detail=after["what"])
        self.log(f"{candidate['id']}: {after['what']}")
        return {"episode": episode, "acted": True, "outcome": "succeeded",
                "measured": after}

    # --- the states ---------------------------------------------------------

    def plan(self, candidate: dict[str, Any],
             here: situation_mod.Situation | None = None
             ) -> list[dict[str, Any]]:
        """The admitted operations that would carry this goal out, in order.

        **Two goal types, and neither of them is `explore`.** A frontier goal is
        a drive to the place the chooser picked; a geometry goal is a drive to
        the viewpoint and then one look from it. The rover's own frontier run is
        not used even for the frontier goal, because a run that chooses its own
        next goal is a movement this episode could not attribute -- and because
        [R-NAV-6](../docs/requirements/navigation.md#r-nav-6) is failing.

        Checked against the daemon's list here, before the first step is
        dispatched, rather than discovered half-way through a sequence that has
        already moved the rover.
        """
        goal = candidate["constraints"].get("goal") or {}
        if goal.get("x_m") is None or goal.get("y_m") is None:
            raise Aborted(f"{candidate['id']} has nowhere to drive to",
                          "no goal")
        drive = {"action": "drive_to",
                 "params": {"x_m": float(goal["x_m"]), "y_m": float(goal["y_m"]),
                            "said": candidate.get("expects") or ""}}
        heading = goal.get("heading_deg")
        if heading is not None:
            drive["params"]["heading_deg"] = float(heading)
        steps = [drive]
        if candidate["type"] == "improve_geometry":
            # **Recorded, not settled.** Settling decides identities from every
            # bearing pending, and with 2,000 pending that is the better part
            # of ten seconds holding the lock every look needs: in M3 session 6
            # six looks in 39 failed waiting for it or timed out inside it. The
            # rover's own clock settles this look within ten seconds; what the
            # evaluation straight afterwards misses by that was measured at
            # 9 goals in 171 (2026-10-06, looks-seldom-reach-their-thing).
            look = {"settle": False}
            if NAME_THE_TARGET:
                # Named, so that the world state gives the thing the region it
                # was aimed at rather than filing it like any other look: 5
                # aimed looks in 157 reached their thing that way (2026-10-08).
                # See world_state/aimed.py.
                look["target"] = candidate["target"]
            # Aimed at the thing, which the daemon does from the heading it
            # measures, so the arrival tolerance does not leave it off the
            # picture. A record from before 2026-10-03 has no place to aim at.
            aim = candidate["constraints"].get("look_at")
            if isinstance(aim, dict):
                look["aim_at"] = aim
            # Tilted level only when the thing needs it to be in the depth
            # camera's view (goals.LOOK_TILTS_DEG); the resting tilt moves
            # nothing, and saying so would move the gimbal there and back.
            tilt = candidate["constraints"].get("look_tilt_deg")
            if tilt is not None and float(tilt) != goals.LOOK_TILTS_DEG[0]:
                look["tilt_deg"] = float(tilt)
            steps.append({"action": "world_inspect", "params": look})
        if candidate["type"] == hypotheses.GOAL_TYPE:
            # Both steps carry the case and its limits, which is what lets the
            # daemon count them against the place rather than the goal, and stop
            # a drive that runs past them. The look is taken fresh, keeps its
            # depth, and leaves identity to the rover's own settling: deciding
            # which thing this look belongs to is not this attempt's business.
            limits = candidate["constraints"].get("inspection")
            if not isinstance(limits, dict):
                raise Aborted(f"{candidate['id']} carries no inspection limits",
                              "no limits")
            drive["params"]["inspection"] = limits
            # Aimed at the place as well: the daemon measures where the rover
            # really faces and pans the camera, within its calibration, to put
            # the place in the middle of the picture.
            steps.append({"action": "world_inspect",
                          "params": {"settle": False, "fresh": True,
                                     "keep_depth": True,
                                     "tilt_deg": candidate["constraints"].get(
                                         "tilt_deg"),
                                     "aim_at": limits.get("target"),
                                     "inspection": limits}})

        if (self.trial.get("relook") and candidate["type"] == "improve_geometry"
                and here is not None):
            steps = [*self._relook_steps(candidate, here), *steps]

        for step in steps:
            if step["action"] not in permission.ACTIONS:
                raise Aborted(f"{step['action']} is not an operation the rover "
                              f"admits from autonomy", "not admitted")
        return steps

    def _relook_steps(self, candidate: dict[str, Any],
                      here: situation_mod.Situation) -> list[dict[str, Any]]:
        """An M4 trial's re-look: face the thing from where the rover stands,
        and take one aimed look at it, as the chosen viewpoint's look is taken.

        The turn is a drive to the spot the rover is on, with a heading, which
        navigation carries out as a turn alone (a near goal closer than its
        arrival tolerance is only its heading). The look is tilted for the
        thing's elevation from here, worked out from its height as the goal
        measured it, so that a re-look is not refused the depth camera's view
        that the chosen viewpoint was given. [] when the rover's position or the
        thing's is not known.
        """
        facts = candidate["constraints"]
        aim = facts.get("look_at")
        where = here.where
        if not isinstance(aim, dict) or where is None or aim.get("x_m") is None:
            return []
        dx, dy = float(aim["x_m"]) - where[0], float(aim["y_m"]) - where[1]
        facing = math.degrees(math.atan2(dy, dx))
        turn = {"action": "drive_to", "role": RELOOK,
                "params": {"x_m": round(where[0], 3), "y_m": round(where[1], 3),
                           "heading_deg": round(facing, 1),
                           "said": f"turning to look at {candidate['target']} "
                                   f"again from where it stands"}}
        look: dict[str, Any] = {"settle": False, "aim_at": aim}
        if NAME_THE_TARGET:
            look["target"] = candidate["target"]
        elevation, range_m = facts.get("elevation_deg"), facts.get("range_m")
        if elevation is not None and range_m:
            height = math.tan(math.radians(float(elevation))) * float(range_m)
            tilt, _ = goals._tilt_for({"height_m": height}, math.hypot(dx, dy))
            if tilt is not None and float(tilt) != goals.LOOK_TILTS_DEG[0]:
                look["tilt_deg"] = float(tilt)
        return [turn, {"action": "world_inspect", "role": RELOOK, "params": look}]

    def relook(self, episode: str, step: dict[str, Any],
               candidate: dict[str, Any]) -> dict[str, Any]:
        """Carry out one re-look step. **One that did not succeed does not end
        the attempt**: the re-look is the baseline the chosen viewpoint is
        compared with, and a re-look that could not turn or saw nothing is a
        baseline that gained nothing, which the scoring counts as such. A stop,
        a run that ended or a lost connection still ends it, like any step."""
        try:
            return self.do(episode, step, candidate)
        except Aborted as stop:
            # `do` has already recorded the call and what it answered. What
            # still ends the attempt is what would end any step: the run over,
            # a person's stop, a move that may still be going, or no daemon.
            if stop.code in RELOOK_ENDS:
                raise
            return {"ok": False, "error": stop.why}

    def snapshot(self, episode: str) -> dict[str, Any]:
        """Copy the world store and the map as they are before the attempt, for
        M4's scoring, and write down where they went. A copy that fails is
        recorded and the attempt goes ahead: an attempt without one is counted
        as unscorable rather than not made."""
        began = self.now()
        try:
            got = self.rover.call("world_snapshot", {"name": episode})
        except client_mod.Unreachable as exc:
            raise Aborted(str(exc), "connection lost") from exc
        self.store.append(episode, events.call(
            "world_snapshot", {"name": episode}, ok=bool(got.get("ok")),
            result={k: got.get(k) for k in ("path", "map_path", "bytes",
                                            "took_s") if k in got},
            error=str(got.get("error") or ""),
            duration_s=round(self.now() - began, 2)))
        return got

    def do(self, episode: str, step: dict[str, Any],
           candidate: dict[str, Any], busy_retries: int = LOOK_BUSY_RETRIES
           ) -> dict[str, Any]:
        """Dispatch one step, wait for it if it is not over, and record it.

        The call event carries the answer as well as the question, which is what
        lets a replay reconstruct the run without touching the rover.
        """
        self.renew()
        action_id = f"{episode}#{len(self.actions) + 1}"
        params = dict(step["params"])
        # The map the goal was chosen on travels with the request, so that the
        # daemon refuses it rather than translating it if the map has been
        # replaced in between. This is the whole of the staleness check from
        # this side: naming what was assumed.
        params.setdefault("map_id", self._map_id)
        began = self.now()
        # Commit intent before the socket write: a kill or lost reply must not
        # erase the fact that this episode may have dispatched a physical move.
        self.store.append(episode, events.make("dispatch", {
            "action_id": action_id, "call": step["action"], "params": params,
            **({"role": step["role"]} if step.get("role") else {})}))
        try:
            answer = self.rover.call("autonomy_act", {
                "permit": self.permit, "action": step["action"],
                "action_id": action_id, "episode": episode, "params": params},
                timeout=(LOOK_CALL_TIMEOUT_S if step["action"] == "world_inspect"
                         else None))
        except client_mod.Unreachable as exc:
            self._lost_action(episode, step, params, action_id, began, str(exc))
            raise Aborted(str(exc), "connection lost") from exc
        self.acted += 1
        self.actions.append({"id": action_id, "action": step["action"],
                             "ok": bool(answer.get("ok"))})

        if not answer.get("ok"):
            self.store.append(episode, events.call(
                step["action"], params, ok=False,
                result={"action_id": action_id,
                        **({"role": step["role"]} if step.get("role") else {})},
                error=str(answer.get("error") or "refused"),
                duration_s=round(self.now() - began, 2)))
            if (step["action"] == "world_inspect" and busy_retries > 0
                    and "has been running" in str(answer.get("error") or "")):
                self.sleep(LOOK_BUSY_WAIT_S)
                return self.do(episode, step, candidate, busy_retries - 1)
            raise Aborted(f"{step['action']} was refused: "
                          f"{answer.get('error')}",
                          str(answer.get("refused") or "refused"))

        result = answer
        if answer.get("running"):
            try:
                result = self.wait(action_id, step["action"])
            except (Aborted, client_mod.Unreachable) as exc:
                self._lost_action(episode, step, params, action_id, began, str(exc))
                if isinstance(exc, client_mod.Unreachable):
                    raise Aborted(str(exc), "connection lost") from exc
                raise
        self.store.append(episode, events.call(
            step["action"], params, ok=bool(result.get("ok")),
            result={"action_id": action_id,
                    **({"role": step["role"]} if step.get("role") else {}),
                    **{k: v for k, v in result.items()
                    if k in ("note", "detail", "regions", "attached",
                             "placed", "ranged", "stopped", "going",
                             "frame_id", "pose", "status", "stored",
                             "aimed_filing")}},
            error=str(result.get("error") or ""),
            duration_s=round(self.now() - began, 2)))
        if not result.get("ok"):
            raise Aborted(f"{step['action']} did not succeed: "
                          f"{result.get('error') or result.get('detail')}",
                          "failed")
        return result

    def _lost_action(self, episode, step, params, action_id, began, why):
        self.store.append(episode, events.call(
            step["action"], params, ok=False,
            result={"action_id": action_id, "completion_known": False},
            error=why, duration_s=round(self.now() - began, 2)))

    def wait(self, action_id: str, action: str) -> dict[str, Any]:
        """Poll until the daemon says the action is over, or give up.

        A move nobody waits for is finished when the wheels stop, and the
        daemon's own account of it is what says so -- not the navigator's, and
        not a guess from the pose. Every poll renews the permission, which is
        also how the daemon learns this process is still alive.
        """
        until = self.now() + ACTION_TIMEOUT_S
        while True:
            self.sleep(RENEW_EVERY_S)
            self.renew(soft=True)
            status = self._status()
            if not status.get("enabled"):
                raise Aborted(f"the run ended while {action} was running: "
                              f"{status.get('why')}",
                              "stopped" if status.get("latched") else "run over")
            doing = status.get("doing") or {}
            if doing.get("id") == action_id and doing.get("ok") is not None:
                return {"ok": bool(doing["ok"]),
                        "detail": str(doing.get("detail") or "")}
            if self.now() >= until:
                # The stop belongs to `give_up`, which every abort goes
                # through; stopping here as well would send two of them.
                raise Aborted(f"{action} was still running after "
                              f"{ACTION_TIMEOUT_S:.0f} s", "timed out")

    def evaluate(self, episode: str, before: situation_mod.Situation,
                 candidate: dict[str, Any],
                 looked: dict[str, Any] | None = None) -> dict[str, Any]:
        """Read the rover again and record what the attempt actually changed.

        **What it hoped for is already in the record**, as the candidate's gain,
        so what is worth adding is the difference -- and it is worth adding even
        when it is disappointing, because a goal type that never delivers what it
        promises is only visible in the gap between the two.
        """
        after = self._read()
        measured: dict[str, Any] = {
            "travelled_m": self._travelled(),
            "battery_v": after.battery_v,
        }
        if candidate["type"] == hypotheses.GOAL_TYPE:
            what = (f"tested whether anything stands where "
                    f"{candidate['target']}'s looks crossed")
        elif candidate["type"] == "improve_geometry":
            was = _uncertainty(before, candidate["target"])
            now = _uncertainty(after, candidate["target"])
            measured.update({"placement_uncertainty_before_m": was,
                             "placement_uncertainty_after_m": now})
            if self.trial.get("relook"):
                # What changed is the re-look's and the chosen look's together;
                # M4's scoring separates them against the snapshot.
                measured["with_relook"] = True
            if was is not None and now is not None:
                measured["placement_improved_m"] = round(was - now, 3)
            what = _said_geometry(candidate["target"], was, now)
            # Put aside if it got nowhere, written where the next deliberation
            # reads its cooling from. See `cooling.after_attempt`.
            cooled = decide_mod._loads(self.store.marked(decide_mod.COOLED_MARK)) or []
            # **Nothing found where the depth camera could see is worth more
            # than nothing found.** A look taken with the thing inside the depth
            # camera's view that filed nothing to it says the record is not
            # where it claims, and such a record is set aside for longer
            # (`cooling.EMPTY_COOLDOWN_S`); one taken from where the camera
            # could not see the place says nothing of the kind.
            filing = (looked or {}).get("aimed_filing") or {}
            seen_empty = (bool(looked) and not filing.get("filed")
                          and "points at it" in str(filing.get("why") or "")
                          and candidate["constraints"].get("in_depth_view") is True)
            if seen_empty:
                measured["seen_empty"] = True
            now_cooled = cooling.after_attempt(cooled, candidate["target"],
                                               before.as_dict(), after.as_dict(),
                                               now=after.at, seen_empty=seen_empty)
            if now_cooled != cooled:
                self.store.mark(decide_mod.COOLED_MARK, decide_mod._dumps(now_cooled))
                measured["put_aside"] = True
                what += "; put aside for a while"
        else:
            was = _unknown_m2(before)
            now = _unknown_m2(after)
            measured.update({"unmapped_before_m2": was, "unmapped_after_m2": now})
            if was is not None and now is not None:
                measured["mapped_m2"] = round(was - now, 2)
            what = _said_frontier(was, now)
        self.store.append(episode, events.measured("the attempt", **measured))
        return {"what": what, **measured}

    # --- an inspection's own record --------------------------------------------

    def freeze(self, episode: str, here: situation_mod.Situation,
               candidate: dict[str, Any]) -> dict[str, Any]:
        """Write the request down before anything moves (R-AUT-12).

        The claim, its alternatives and the question come from the candidate,
        which was generated from the snapshot the decision names. The source
        looks are read now, once, from the world state, and frozen with it: the
        resolver may attach this attempt's own look to the thing, or merge it
        away, and the claim being tested must be the one that was asked.
        """
        detail = candidate.get("gain_detail") or {}
        facts = candidate.get("constraints") or {}
        source: list[int] = []
        entity = candidate.get("target") or ""
        if entity:
            try:
                got = self.rover.call("world_state_entity", {"id": entity})
            except client_mod.Unreachable as exc:
                raise Aborted(str(exc), "connection lost") from exc
            source = [int(one["id"]) for one in (got.get("observations") or [])
                      if one.get("id") is not None]
        request = {
            "case": facts.get("case") or detail.get("case"),
            "claim": detail.get("claim"),
            "claimed_by": entity,
            "alternatives": detail.get("alternatives"),
            "question": detail.get("question"),
            "evidence_needed": detail.get("evidence_needed"),
            "source": source,
            "viewpoint": facts.get("goal"),
            "tilt_deg": facts.get("tilt_deg"),
            "patch_fits": detail.get("patch_fits"),
            "limits": (facts.get("inspection") or {}).get("limits"),
            "attempt": int(facts.get("case_attempts") or 0) + 1,
            "map_session": here.map_session,
            "map_id": here.nav.get("map_id"),
        }
        self.store.append(episode, events.inspection(
            str(request["case"]), "request", refs=candidate.get("refs") or (),
            **{k: v for k, v in request.items() if k != "case"}))
        return request

    def check(self, episode: str, request: dict[str, Any],
              looked: dict[str, Any]) -> dict[str, Any]:
        """Ask whether the look shows the claim, and record what it answers.

        A read, not an act: `world_state_check` reads the stored look, its depth
        and the frozen source looks, and decides nothing about identity. A look
        whose direction was withheld goes to the check anyway, which answers
        unresolved and says why -- that is an attempt with an answer, not one
        that failed.
        """
        frame_id = looked.get("frame_id")
        if not frame_id:
            return self.answered(episode, request, {
                "outcome": "unresolved", "code": "no look",
                "why": "the look recorded no picture: "
                       + str(looked.get("detail") or looked.get("status")
                             or "nothing said why")})
        pose = looked.get("pose")
        try:
            got = self.rover.call("world_state_check", {
                "frame_id": frame_id, "claim": request["claim"],
                "source": request["source"], "pose": pose,
                "withheld": "" if pose else str(looked.get("detail") or
                                                "the look kept no pose")})
        except client_mod.Unreachable as exc:
            raise Aborted(str(exc), "connection lost") from exc
        if not got.get("ok"):
            return self.answered(episode, request, {
                "outcome": "unresolved", "code": "check failed",
                "why": f"the check could not be made: {got.get('error')}"})
        return self.answered(episode, request, {
            "outcome": got.get("outcome"), "code": got.get("code"),
            "why": got.get("why"), "frame_id": frame_id,
            "evidence": got.get("evidence")})

    def answered(self, episode: str, request: dict[str, Any],
                 result: dict[str, Any]) -> dict[str, Any]:
        """Record an attempt's result and spend it against its place.

        Every attempt ends here, whatever ended it, which is what makes "every
        attempt is recorded" a property of the code rather than of the run: a
        refusal, a stop and a lost connection are attempts with an unresolved
        answer, and they count against the case like any other.
        """
        outcome = result.get("outcome")
        if outcome not in events.INSPECTION_OUTCOMES:
            result = {**result, "outcome": "unresolved",
                      "why": f"the check answered {outcome!r}, which is not an "
                             f"answer: {result.get('why')}"}
        self.store.append(episode, events.inspection(
            str(request["case"]), "result", **result))
        claim = request.get("claim") or {}
        self.store.mark(decide_mod.INSPECTION_MARK, json.dumps({
            "case": request["case"], "episode": episode,
            "target": {"x_m": claim.get("x_m"), "y_m": claim.get("y_m")},
            "map_session": request.get("map_session"),
            "map_id": request.get("map_id"),
            "claimed_by": request.get("claimed_by"),
            "attempt": request.get("attempt"),
            "outcome": result["outcome"], "code": result.get("code")}),
            at=self.now())
        request["answered"] = result
        return result

    def give_up(self, episode: str, stop: Aborted) -> None:
        """Close an episode that did not finish, having stopped the rover.

        The stop is attempted and its refusal ignored, because the reasons an
        abort happens include the ones that make stopping unavailable: a person
        has already stopped the rover, or the daemon has already ended the run
        and taken the wheels back. Neither is a rover still moving.
        """
        if stop.code not in ("stopped", "run over", "latched"):
            self.stop(stop.why)
        self.store.append(episode, events.note(
            f"abandoned in {self.state}: {stop.why}"))
        self.store.close_episode(episode, "interrupted", detail=stop.why)
        self.log(f"aborted: {stop.why}")

    def go_back(self, here: situation_mod.Situation, inputs: str,
                why: str) -> dict[str, Any]:
        """Drive back to where the run started, and end the run.

        **The last turn of a run with nothing left worth doing.** Its own
        episode with a decision of its own, so that the drive is attributable
        like every other; the deliberation that found nothing is closed already.
        The drive names the map the run started on, so that the daemon refuses
        it rather than translating it if the map has been replaced since. Getting
        back is attempted once: a refusal or a failure ends the run where the
        rover is, and says so.
        """
        self.finished = True
        generation = (here.world_generation
                      if here.world_generation != refs.UNKNOWN else None)
        episode = self.store.open_episode(
            RETURN_TRIGGER, world_generation=generation,
            map_session=here.map_session,
            note="the run's last turn: back to where it started, then the run "
                 "is handed back")
        self.episode = episode
        start = self.start or {}
        where = here.where
        if start.get("x_m") is None or start.get("y_m") is None:
            ended = (f"{why}; the run did not record where it started, so the "
                     f"rover stayed where it was")
            self.store.append(episode, events.decision(RETURN_GOAL, ended, inputs))
            self.store.close_episode(episode, "abandoned", detail=ended)
        elif where is not None and math.hypot(
                where[0] - float(start["x_m"]),
                where[1] - float(start["y_m"])) <= HOME_NEAR_M:
            ended = f"{why}; it was already where the run started"
            self.store.append(episode, events.decision(RETURN_GOAL, ended, inputs))
            self.store.close_episode(episode, "succeeded", detail=ended)
        else:
            x, y = float(start["x_m"]), float(start["y_m"])
            say = f"back to ({x:.2f}, {y:.2f}), where the run started"
            self.store.append(episode, events.decision(
                RETURN_GOAL, f"{why}; so {say}", inputs))
            params: dict[str, Any] = {"x_m": x, "y_m": y, "said": say}
            if start.get("heading_deg") is not None:
                params["heading_deg"] = float(start["heading_deg"])
            if start.get("map_id") is not None:
                params["map_id"] = start["map_id"]
            try:
                self.state = "EXECUTE"
                self.do(episode, {"action": "drive_to", "params": params},
                        {"id": RETURN_GOAL, "type": RETURN_GOAL})
            except Aborted as stop:
                self.state = "ABORT"
                self.give_up(episode, stop)
                ended = (f"{why}; it could not get back to where the run "
                         f"started: {stop.why}")
            else:
                ended = f"{why}; it went back to where the run started"
                self.store.close_episode(episode, "succeeded", detail=ended)
        self.state = "IDLE"
        self.ended = ended
        self.log(ended)
        self.release(ended)
        return {"episode": episode, "ended": ended}

    # --- talking to the rover -----------------------------------------------

    def renew(self, soft: bool = False) -> str:
        """Ask for permission, or renew what we have. Aborts the turn if not.

        `soft` is for the polling loop, where a single unanswered renewal is not
        news: the permission is good for several of these, and treating one
        refusal as the end would abandon goals over a busy daemon.
        """
        answer = self.rover.call("autonomy_permit", {"run": self.run})
        if answer.get("ok"):
            self.permit = str(answer["permit"])
            return self.permit
        if soft and self.permit:
            return self.permit
        self.permit = ""
        raise Aborted(f"the rover would not renew permission: "
                      f"{answer.get('error')}",
                      "latched" if answer.get("latched") else "no permit")

    def idle(self, seconds: float) -> bool:
        """Wait, without letting the waiting look like a death.

        The permission is renewed through the wait, because it says that this
        loop is alive and it is: a rover with nothing worth doing is the
        ordinary state of a parked one, and a run that ended every time the room
        was uninteresting would end within a minute of starting.

        Returns False when the run went away while waiting, which is not a
        failure either -- a person stopping the rover, or a budget running out,
        is the expected way for an idle session to finish.
        """
        until = self.now() + seconds
        while True:
            left = until - self.now()
            if left <= 0:
                return True
            self.sleep(min(IDLE_NAP_S, left))
            try:
                self.renew()
            except Aborted as stop:
                self.ended = self.ended or stop.why
                return False

    def stop(self, why: str) -> dict[str, Any]:
        """Stop the rover through the permission, and never mind a refusal."""
        if not self.permit:
            return {"ok": False, "error": "no permit"}
        try:
            return self.rover.call("autonomy_act", {
                "permit": self.permit, "action": "stop",
                "action_id": f"stop/{self.acted}/{int(self.now())}",
                "episode": self.episode or self.run,
                "params": {"why": why}})
        except client_mod.Unreachable as exc:                  # pragma: no cover
            return {"ok": False, "error": str(exc)}

    def release(self, why: str = "") -> dict[str, Any]:
        """Hand the run back, so that a finished session does not look live."""
        if not self.run:
            return {"ok": True}
        try:
            return self.rover.call("autonomy_release",
                                   {"run": self.run,
                                    "why": why or self.ended
                                    or "the executive finished"})
        except client_mod.Unreachable as exc:                  # pragma: no cover
            return {"ok": False, "error": str(exc)}

    def summary(self) -> dict[str, Any]:
        return {"run": self.run, "turns": self.turns, "actions": self.acted,
                "ended": self.ended, "state": self.state}

    # --- reading ------------------------------------------------------------

    def _read(self) -> situation_mod.Situation:
        here = situation_mod.Situation.read(self.rover, at=self.now())
        nav = here.nav
        self._map_id = nav.get("map_id")
        return here

    def _status(self) -> dict[str, Any]:
        answer = self.rover.call("autonomy_status", {})
        return answer if isinstance(answer, dict) else {}

    def _alive(self) -> bool:
        """Is there still a run, and may we still act in it?

        Asked before every turn rather than assumed from the last one, because
        the interesting case is the one where something happened in between --
        which is every case this exists for.
        """
        try:
            status = self._status()
        except client_mod.Unreachable as exc:
            self.ended = f"the daemon stopped answering: {exc}"
            return False
        if not status.get("enabled"):
            self.ended = str(status.get("why") or "the run has ended")
            return False
        run = status.get("run") or {}
        if run.get("id") != self.run:
            self.ended = "the run this executive was attached to has been replaced"
            return False
        try:
            self.renew()
        except Aborted as stop:
            self.ended = stop.why
            return False
        return True

    def _travelled(self) -> float | None:
        """How far the run has driven, from the daemon's own accounting."""
        run = (self._status().get("run") or {})
        spent = run.get("spent") or {}
        return spent.get("travel_m")

    def _why_stopped(self) -> str:
        """Why the loop is over, when it was not the run that ended it.

        A run still open means the loop stopped for its own reason -- it was
        asked for a fixed number of turns and has done them. Reading the rover's
        answer here regardless produced "the run ended: a run is open", which is
        the sort of sentence that makes a person distrust the rest of the
        record.
        """
        try:
            status = self._status()
        except client_mod.Unreachable:                         # pragma: no cover
            return "the daemon stopped answering"
        if status.get("enabled"):
            return "the executive finished the turns it was asked for"
        return str(status.get("why") or "the loop finished")


# --- the sentences a person reads --------------------------------------------

def _uncertainty(here: situation_mod.Situation, target: str) -> float | None:
    """What the rover claims for the thing's placement; see `situation.claimed_m`."""
    for one in here.entities:
        if one.get("id") == target:
            return situation_mod.claimed_m(one)
    return None


def _unknown_m2(here: situation_mod.Situation) -> float | None:
    grid = here.grid
    return None if grid is None else round(mapgrid.unknown_m2(grid), 2)


def _said_geometry(target: str, was: float | None,
                   now: float | None) -> str:
    if was is None or now is None:
        return f"looked at {target}; how well it is placed is not known"
    if now < was:
        return (f"{target} is placed to {now:.2f} m, {was - now:.2f} m better "
                f"than before")
    return (f"{target} is placed to {now:.2f} m, no better than the "
            f"{was:.2f} m it was")


def _said_frontier(was: float | None, now: float | None) -> str:
    if was is None or now is None:
        return "drove to the edge of the map; the map did not come back"
    if now < was:
        return (f"{was - now:.1f} m2 less of the house is unmapped "
                f"({now:.0f} m2 left)")
    return f"the map did not grow; {now:.0f} m2 is still unmapped"


# --- running it ---------------------------------------------------------------

def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Carry out what the rover decides, under a permit the "
                    "daemon issued. Cannot enable itself.")
    parser.add_argument("--dir", default=None,
                        help="where to keep the record (default ~/.ugv/autonomy)")
    parser.add_argument("--turns", type=int, default=None,
                        help="stop after this many goals (default: until the "
                             "run ends)")
    parser.add_argument("--m0a", action="store_true",
                        help="run the frozen M0a protocol: hypothesis "
                             "inspections only (R-AUT-12)")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8769)
    args = parser.parse_args(argv)

    store = store_mod.EpisodeStore(args.dir)
    weights = scoring.Weights.load(args.dir)
    if args.m0a:
        weights.m0a_protocol = True
    else:
        # A run goes further afield when nothing nearer is worth doing, and
        # goes back and ends only when nothing anywhere is. M0a's protocol is
        # frozen and keeps the scorer as it was.
        weights.go_further = True
    rover = client_mod.Acting(args.host, args.port)
    executive = Executive(store, rover, weights)

    attached = executive.attach()
    if not attached.get("ok"):
        print(f"nothing to do: {attached['error']}")
        print("A run is opened from the console or with autonomy_start; this "
              "program cannot open one.")
        store.close()
        return 1
    run = attached["run"]
    print(f"attached to {run['id']}, started via {run.get('via')}"
          + (f" for {run['why']}" if run.get("why") else ""))
    print(f"budget {run['budget']}")

    try:
        got = executive.loop(turns=args.turns)
    except KeyboardInterrupt:
        executive.stop("the operator interrupted the executive")
        got = executive.summary()
        got["ended"] = "interrupted at the keyboard"
    executive.release(got.get("ended") or "")
    print(f"\n{got['turns']} turns, {got['actions']} actions; "
          f"ended because {got['ended']}")
    print(summary_mod.of_store(store) if hasattr(summary_mod, "of_store")
          else "")
    store.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
