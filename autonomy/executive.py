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
import os
import sys
import time
from typing import Any, Callable

import client as client_mod
import decide as decide_mod
import events
import mapgrid
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

#: How long to stand still after a turn of the loop that chose nothing. The
#: rover is parked, its map is not changing and neither is the answer, so this is
#: about not filling the record with identical refusals rather than about
#: responsiveness -- and the shadow runs already deliberate once a minute.
IDLE_S = 30.0


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
            self.store.close_episode(
                episode, "abandoned",
                detail=decision.get("why_nothing") or "nothing worth doing")
            self.log(f"nothing to do: {decision.get('why_nothing')}")
            self.sleep(IDLE_S)
            return {"episode": episode, "acted": False,
                    "why": decision.get("why_nothing")}

        candidate = decision["preferred"]["candidate"]
        try:
            self.state = "PLAN"
            plan = self.plan(candidate)
            self.state = "EXECUTE"
            for step in plan:
                self.do(episode, step, candidate)
            self.state = "EVALUATE"
            after = self.evaluate(episode, here, candidate)
        except Aborted as stop:
            self.state = "ABORT"
            self.give_up(episode, stop)
            return {"episode": episode, "acted": True, "why": stop.why,
                    "outcome": "interrupted"}
        self.state = "IDLE"
        self.store.close_episode(episode, "succeeded", detail=after["what"])
        self.log(f"{candidate['id']}: {after['what']}")
        return {"episode": episode, "acted": True, "outcome": "succeeded",
                "measured": after}

    # --- the states ---------------------------------------------------------

    def plan(self, candidate: dict[str, Any]) -> list[dict[str, Any]]:
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
            steps.append({"action": "world_inspect", "params": {"settle": True}})

        for step in steps:
            if step["action"] not in permission.ACTIONS:
                raise Aborted(f"{step['action']} is not an operation the rover "
                              f"admits from autonomy", "not admitted")
        return steps

    def do(self, episode: str, step: dict[str, Any],
           candidate: dict[str, Any]) -> dict[str, Any]:
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
        answer = self.rover.call("autonomy_act", {
            "permit": self.permit, "action": step["action"],
            "action_id": action_id, "episode": episode, "params": params})
        self.acted += 1
        self.actions.append({"id": action_id, "action": step["action"],
                             "ok": bool(answer.get("ok"))})

        if not answer.get("ok"):
            self.store.append(episode, events.call(
                step["action"], params, ok=False,
                error=str(answer.get("error") or "refused"),
                duration_s=round(self.now() - began, 2)))
            raise Aborted(f"{step['action']} was refused: "
                          f"{answer.get('error')}",
                          str(answer.get("refused") or "refused"))

        result = answer
        if answer.get("running"):
            result = self.wait(action_id, step["action"])
        self.store.append(episode, events.call(
            step["action"], params, ok=bool(result.get("ok")),
            result={k: v for k, v in result.items()
                    if k in ("note", "detail", "regions", "attached",
                             "placed", "ranged", "stopped", "going")},
            error=str(result.get("error") or ""),
            duration_s=round(self.now() - began, 2)))
        if not result.get("ok"):
            raise Aborted(f"{step['action']} did not succeed: "
                          f"{result.get('error') or result.get('detail')}",
                          "failed")
        return result

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
                 candidate: dict[str, Any]) -> dict[str, Any]:
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
        if candidate["type"] == "improve_geometry":
            was = _uncertainty(before, candidate["target"])
            now = _uncertainty(after, candidate["target"])
            measured.update({"placement_uncertainty_before_m": was,
                             "placement_uncertainty_after_m": now})
            if was is not None and now is not None:
                measured["placement_improved_m"] = round(was - now, 3)
            what = _said_geometry(candidate["target"], was, now)
        else:
            was = _unknown_m2(before)
            now = _unknown_m2(after)
            measured.update({"unmapped_before_m2": was, "unmapped_after_m2": now})
            if was is not None and now is not None:
                measured["mapped_m2"] = round(was - now, 2)
            what = _said_frontier(was, now)
        self.store.append(episode, events.measured("the attempt", **measured))
        return {"what": what, **measured}

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
        try:
            return str(self._status().get("why") or "the loop finished")
        except client_mod.Unreachable:                         # pragma: no cover
            return "the daemon stopped answering"


# --- the sentences a person reads --------------------------------------------

def _uncertainty(here: situation_mod.Situation, target: str) -> float | None:
    for one in here.entities:
        if one.get("id") == target:
            value = one.get("placement_uncertainty_m")
            return None if value is None else float(value)
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
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8769)
    args = parser.parse_args(argv)

    store = store_mod.EpisodeStore(args.dir)
    weights = scoring.Weights.load(args.dir)
    rover = client_mod.Acting(args.host, args.port)
    executive = Executive(store, rover, weights)

    attached = executive.attach()
    if not attached.get("ok"):
        print(f"nothing to do: {attached['error']}")
        print("A person opens a run with autonomy_enable; this program cannot.")
        store.close()
        return 1
    run = attached["run"]
    print(f"attached to {run['id']}, opened by {run['by']}"
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
