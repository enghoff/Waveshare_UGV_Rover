"""The rover's half of autonomous permission: the calls, and the watchdog.

[`permission.py`](permission.py) holds the rules and knows nothing about a
rover. This is what wires them to one: the five calls a person or an executive
makes, the dispatch of the three operations autonomy may ask for, and the loop
that takes the wheels away when a run is over whether or not anything asked it
to.

**The watchdog is the part that matters.** Everything else here could be done by
a careful executive; this is the part that works when the executive is not
careful, or is not there. It runs on the daemon's own thread, it reads the
rover's own pose, and it stops the wheels when the run's time, distance or
permission has run out -- while Nav2 is perfectly healthy and would otherwise
carry on driving to the goal it was given.

None of these appear in `tools()`. They are control calls like `nav_status`, so
no model is ever shown them: the voice model can ask the rover to stop, because
`stop_driving` is a tool, and it has no way to ask for autonomy to be turned
back on, because enabling is not.

## Who may make which call

- `autonomy_enable` and `autonomy_stop` are the person's. Enabling opens a
  bounded run and is the only thing that clears a stop.
- `autonomy_permit` and `autonomy_act` are the executive's. Neither can open a
  run, and `autonomy/client.py` refuses `autonomy_enable` outright so that the
  executive cannot re-enable itself after being stopped -- the same structural
  refusal that keeps the recorder off the wheels.
- `autonomy_status` is anybody's. It is a read.
"""
from __future__ import annotations

from typing import Any

#: The calls a person makes that end a run. Stopping is the obvious one; the
#: rest are a person taking the rover back by doing something with it. Driving
#: it by hand, sending it somewhere by voice, running a script, clearing the map
#: or refitting the pose all say the same thing -- somebody else is in charge
#: now -- and a rover that let an autonomous run carry on around that would be
#: arguing with the person in the room.
#:
#: The last three are not moves at all, and they are here because they move the
#: ground the run is standing on: a goal chosen on a map that has just been
#: thrown away is a drive to somewhere nobody picked.
HUMAN_MOVES = frozenset({
    "drive", "drive_to", "drive_to_map_point", "turn_in_place", "explore",
    "go_to_thing", "stop_driving", "run_script", "start_script",
    "clear_map", "refit_pose", "world_state_clear",
})

#: What a human intervention is called in the record, so that a person reading
#: an episode a fortnight later is told which of these it was.
TAKEOVER = {
    "stop_driving": "somebody stopped the rover",
    "clear_map": "somebody cleared the map",
    "refit_pose": "somebody refitted the rover onto the map",
    "world_state_clear": "somebody emptied the world state",
}


class RoverAutonomy:
    """Autonomous permission, mixed into Rover."""

    # --- what a person calls ------------------------------------------------

    def _tool_autonomy_enable(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """Open a bounded autonomous run. A person's act, not the executive's.

        `budget` narrows the standing limits and can never widen them, so the
        argument for each of those numbers -- which is written where they are
        declared -- cannot be talked out of by whoever is typing the command.

        Whoever enables says who they are and why, because the first question
        about a rover found driving itself is who let it.
        """
        by = str(arguments.get("by") or "").strip()
        if not by:
            return {"ok": False,
                    "error": "say who is enabling this; a run nobody is named "
                             "for is a run nobody is watching"}
        answer = self.permission.enable(
            by=by, why=str(arguments.get("why") or ""),
            budget=arguments.get("budget") or {})
        if answer.get("ok"):
            print(f"[autonomy] enabled by {by}: "
                  f"{answer['run']['id']}, budget {answer['run']['budget']}",
                  flush=True)
        return {**answer, "autonomy": self.permission.status()}

    def _tool_autonomy_stop(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """Stop the rover and latch autonomy off until somebody re-enables it.

        Separate from `stop_driving` only in what it says out loud: both stop
        the wheels and both latch, because a person who wants the rover to stop
        does not care which button they found.
        """
        answer = self.autonomy_taken(str(arguments.get("by") or "a person"),
                                     str(arguments.get("why") or ""))
        stopped = {"stopped": True}
        if self.nav is not None:
            stopped = self.nav.stop()
        return {"ok": True, **stopped, **answer,
                "autonomy": self.permission.status()}

    # --- what the executive calls -------------------------------------------

    def _tool_autonomy_permit(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """Ask for permission to act, or renew it. Never opens a run.

        Renewing is how the daemon learns the executive is still alive, so this
        is called far more often than anything else here -- once every couple of
        seconds while a goal is being carried out.
        """
        run = str(arguments.get("run") or "")
        if not run:
            return {"ok": False,
                    "error": "name the run this permission is for",
                    "autonomy": self.permission.status()}
        ttl = arguments.get("ttl_s")
        answer = self.permission.grant(run, None if ttl is None else float(ttl))
        return {**answer, "autonomy": self.permission.status()}

    def _tool_autonomy_release(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """Hand the run back. The executive saying it has finished.

        Not a stop and not a latch: nothing went wrong and nobody intervened.
        Without it a finished executive would leave the run open until its
        permission lapsed, and a person looking at the rover in those fifteen
        seconds would be told it was still allowed to move.
        """
        run = str(arguments.get("run") or "")
        open_run = self.permission.run
        if open_run is None or open_run.ended:
            return {"ok": True, "note": "no run was open",
                    "autonomy": self.permission.status()}
        if run and run != open_run.id:
            return {"ok": False,
                    "error": f"{run} is not the run that is open",
                    "autonomy": self.permission.status()}
        why = str(arguments.get("why") or "the executive handed it back")
        ended = self.autonomy_end(why)
        return {**ended, "autonomy": self.permission.status()}

    def _tool_autonomy_act(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """Do one autonomous thing, if everything still says it may.

        **Every condition is checked here rather than when the permission was
        granted**, because between the two the battery drains, the map is
        replaced, the pose stops being believable and somebody presses stop --
        and this is the last moment any of that can be noticed before the wheels
        turn.

        A repeat of an action already dispatched is answered with what happened
        the first time. That is not politeness: an executive that lost its
        connection mid-call and asked again would otherwise drive the same leg
        twice, and the second one would be attributed to the same goal.
        """
        permit = str(arguments.get("permit") or "")
        action = str(arguments.get("action") or "")
        action_id = str(arguments.get("action_id") or "")
        episode = str(arguments.get("episode") or "")
        params = dict(arguments.get("params") or {})

        facts = self.autonomy_conditions()
        verdict = self.permission.check(
            permit=permit, action=action, action_id=action_id,
            episode=episode, params=params, conditions=facts)
        if not verdict.ok:
            answer = {"ok": False, "refused": verdict.code, "error": verdict.why,
                      "autonomy": self.permission.status()}
            if verdict.already is not None:
                answer["already"] = verdict.already
            return answer

        self.permission.began(action_id, action, params, episode=episode)
        # Where the rover is standing as the action begins, so that the travel
        # budget is spent from the start of the move rather than from the
        # watchdog's first look at it. Half a second of driving is only 18 cm,
        # but an under-count that happens once per goal adds up over a run.
        self.permission.moved(facts.get("where"), facts.get("map_id"))
        try:
            result = self._autonomy_do(action, params, action_id, episode)
        except Exception as error:      # a bug here must not leave a run open
            self.permission.finished(action_id, ok=False,
                                     detail=f"{type(error).__name__}: {error}")
            return {"ok": False, "error": f"{type(error).__name__}: {error}",
                    "autonomy": self.permission.status()}
        # A move nobody waits for is not finished when this returns -- it is
        # finished when the wheels stop, which `autonomy_trip_ended` hears
        # about. The flag goes back to the caller rather than being consumed
        # here: the executive has to know whether to wait, and an answer that
        # looked complete when it was not is a rover still driving under a loop
        # that has moved on.
        running = bool(result.pop("running", False))
        if not running:
            self.permission.finished(action_id, ok=bool(result.get("ok")),
                                     detail=str(result.get("error") or
                                                result.get("note") or ""))
        return {**result, "running": running, "action_id": action_id,
                "autonomy": self.permission.status()}

    def _autonomy_do(self, action: str, params: dict[str, Any],
                     action_id: str, episode: str) -> dict[str, Any]:
        """Dispatch one admitted operation. Nothing here decides anything.

        Deliberately not routed through `Rover.call`: that is where a human
        move is noticed and turned into a takeover, and an autonomous drive
        arriving there would stop the run it belongs to. The two roads to the
        same hardware are the point -- one of them carries a permit.
        """
        if action == "stop":
            if self.nav is None:
                return {"ok": True, "stopped": True,
                        "note": "this rover does not drive itself"}
            return {"ok": True, **self.nav.stop()}

        if action == "world_inspect":
            return self._tool_world_inspect({"settle": params.get("settle", True)})

        if action == "drive_to":
            if self.nav is None:
                return {"ok": False, "error": "this rover cannot drive itself"}
            if self.nav.driving:
                return {"ok": False,
                        "error": "the rover is already driving somewhere, so "
                                 "this goal was not started"}
            heading = params.get("heading_deg")
            started = self.nav.drive_to_in_background(
                float(params["x_m"]), float(params["y_m"]),
                heading_deg=None if heading is None else float(heading),
                for_what={"autonomy_action": action_id, "episode": episode,
                          "said": str(params.get("said") or "")})
            if not started.get("started"):
                return {"ok": False,
                        "error": "the rover would not take the goal: "
                                 + (started.get("running") or "it is busy")}
            return {"ok": True, "running": True, "going": True,
                    "note": "the rover has set off; it is stopped by "
                            "autonomy_act stop, by stop_driving, or by this "
                            "run's budget running out"}

        # Unreachable through `_tool_autonomy_act`, which checks the list first.
        return {"ok": False, "error": f"{action} is not an autonomy action"}

    # --- what anybody calls -------------------------------------------------

    def _tool_autonomy_status(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        """Whether the rover may move by itself, and what it has spent.

        A read, and on `autonomy/client.py`'s allow-list, so that the shadow
        recorder and the executive both see the same account of authority that
        the daemon enforces -- and a deliberation can say "it would go and look
        at the sofa, but a person stopped the rover" instead of guessing.
        """
        return {"ok": True, **self.permission.status()}

    # --- the parts the rest of the daemon calls -----------------------------

    def autonomy_conditions(self) -> dict[str, Any]:
        """What the permission rules need to know about the rover right now.

        One battery read -- cached, so a tick costs nothing -- and one status
        from the navigation bridge. Everything the checks look at is here rather
        than reached for inside them, so that the rules stay a state machine
        over a clock and can be driven by a test with no rover at all.
        """
        volts, _age = self._sample_battery()
        facts: dict[str, Any] = {"battery_v": volts, "pose_trusted": False,
                                 "map_id": None, "map_settled": None,
                                 "driving": False, "where": None}
        if self.nav is None:
            return facts
        try:
            status = self.nav.status()
        except Exception as error:                             # pragma: no cover
            facts["nav_error"] = f"{type(error).__name__}: {error}"
            return facts
        pose = status.get("pose")
        facts.update({
            "pose_trusted": bool(status.get("position_trusted")),
            "map_id": status.get("map_id"),
            "map_settled": status.get("map_settled"),
            "driving": bool(status.get("driving")),
            "where": (None if not pose else
                      (float(pose["x_m"]), float(pose["y_m"]))),
        })
        return facts

    def autonomy_tick(self) -> str:
        """One turn of the watchdog. Returns why the run ended, or ''.

        Cheap and silent while no run is open, which is almost always: the
        daemon calls this on a timer whether or not anything is happening, and a
        rover nobody has enabled autonomy on must not be paying for a bridge
        round trip twice a second.
        """
        run = self.permission.run
        if run is None or run.ended:
            self._autonomy_ticked = None
            return ""
        was = getattr(self, "_autonomy_ticked", None)
        # The permission's own clock rather than `time.monotonic` directly, so
        # that the elapsed time the jump test is measured against is the same
        # clock the expiry is measured against. They are the same thing on the
        # rover and different things under a test that winds one of them.
        now = self.permission.clock()
        self._autonomy_ticked = now
        facts = self.autonomy_conditions()
        fault = self.permission.moved(facts.get("where"), facts.get("map_id"),
                                      elapsed_s=None if was is None else now - was)
        why = fault or self.permission.due(facts)
        if not why:
            return ""
        self.autonomy_end(why)
        return why

    def autonomy_end(self, why: str) -> dict[str, Any]:
        """Stop the wheels and close the run. Not a latch: nobody asked.

        The wheels first and the bookkeeping second, in that order and not the
        other, because everything after the stop is a record of something that
        has already been made safe.
        """
        if self.nav is not None and self.nav.driving:
            try:
                self.nav.stop()
            except Exception as error:                         # pragma: no cover
                print(f"[autonomy] the run ended and the stop failed: {error}",
                      flush=True)
        ended = self.permission.end_run(why)
        if ended.get("ok"):
            print(f"[autonomy] {ended['ended_run']} ended: {why}", flush=True)
        return ended

    def autonomy_taken(self, by: str, why: str = "") -> dict[str, Any]:
        """A person took the rover back. Latch autonomy off and end any run."""
        run = self.permission.run
        running = run is not None and not run.ended
        answer = self.permission.stop(by=by, why=why)
        if running:
            print(f"[autonomy] {answer.get('ended_run')} ended: "
                  f"{answer.get('why')}", flush=True)
        return answer

    def autonomy_notice(self, name: str) -> None:
        """Called for every tool the daemon dispatches, and does nothing for
        almost all of them.

        It lives in `Rover.call` rather than in each movement tool because that
        is the one place every caller passes through, and a check that has to be
        remembered in eleven handlers is a check that will be missing from the
        twelfth. Autonomy's own actions do not come this way; see
        `_autonomy_do`.
        """
        if name not in HUMAN_MOVES:
            return
        run = self.permission.run
        running = run is not None and not run.ended
        if not running:
            # Nothing to take over. A stop still latches, so that a person who
            # stopped the rover before enabling autonomy is not surprised by it
            # setting off; ordinary manual driving on a rover with no run open
            # is simply driving, and a second stop changes nothing.
            if name != "stop_driving" or self.permission.latch is not None:
                return
        self.autonomy_taken("a person", TAKEOVER.get(name)
                            or f"somebody drove the rover by hand ({name})")

    def autonomy_trip_ended(self, asked: dict[str, Any], outcome: Any) -> None:
        """How an autonomous move ended, heard from the thread that ran it.

        A background move is not finished when `autonomy_act` answers -- it is
        finished when the wheels stop, minutes later -- so this is where the
        run's failure count learns about it. The travel is not taken from here:
        distance is accumulated tick by tick from where the rover actually is,
        so that a move which was killed, or which nobody waited for, still costs
        the run what it drove.
        """
        action_id = str((asked or {}).get("autonomy_action") or "")
        if not action_id:
            return
        reason = str(getattr(outcome, "reason", "") or "")
        detail = str(getattr(outcome, "detail", "") or "")
        self.permission.finished(
            action_id, ok=reason in ("arrived", "stopped"),
            detail=f"{reason}{': ' + detail if detail else ''}")
