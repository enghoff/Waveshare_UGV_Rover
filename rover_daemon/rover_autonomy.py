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

- `autonomy_start` opens a run and starts the executive on it: the console's
  run button, or an agent over this protocol with a purpose and, if it likes, a
  budget and a safe area. `autonomy_enable` opens a run without starting
  anything, for a person who runs the executive by hand (`--m0a`). Either one
  is the only thing that clears a stop. Each run records which way it was
  started (`via`), not who started it.
- `autonomy_stop` is anybody's stop, and latches.
- `autonomy_permit` and `autonomy_act` are the executive's. Neither can open a
  run, and `autonomy/client.py` refuses both opening calls outright so that the
  executive cannot re-enable itself after being stopped -- the same structural
  refusal that keeps the recorder off the wheels.
- `autonomy_status` is anybody's. It is a read.
"""
from __future__ import annotations

import subprocess
import sys
import time
from pathlib import Path
from typing import Any
from functools import wraps

import permission as permission_mod


def serialized(method):
    """Serialize permission transitions with dispatch, never an entire drive."""
    @wraps(method)
    def locked(self, *args, **kwargs):
        with self._autonomy_lock:
            return method(self, *args, **kwargs)
    return locked

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
    "clear_map", "refit_pose", "world_state_clear", "world_state_rebuild",
    "world_state_merge",
})

#: The person's calls above that set the wheels going themselves. When one of
#: them is what ended a run, it is carried out once the stopped leg has let go
#: of the wheels, rather than refused as busy in the moment between: recorded on
#: 2026-10-02 (trial S3), a person's drive sent mid-leg ended the run and was
#: then itself refused, so the person had stopped the rover and not moved it.
PERSON_MOVES = frozenset({
    "drive", "drive_to", "drive_to_map_point", "turn_in_place", "explore",
    "go_to_thing", "run_script", "start_script",
})

#: How long a person's move waits for a stopped autonomous leg to let go. The
#: same three seconds `go_to_thing` waits for any move it interrupts
#: (`rover_recall.HANDOVER_S`); the stop itself reaches the wheels in well under
#: half a second.
TAKEOVER_HANDOVER_S = 3.0
TAKEOVER_POLL_S = 0.05

#: How long a run's recording-only look waits for the rover's own look or
#: settling pass to finish. Nine seconds covers a pass over 2,000 pending
#: bearings (8.4 s measured on 2026-09-03), and with the look itself under a
#: second it answers inside the executive's 13 s (`LOOK_CALL_TIMEOUT_S`) and
#: the 15 s permission renewed just before it.
AUTONOMY_LOOK_WAIT_S = 9.0

#: What a human intervention is called in the record, so that a person reading
#: an episode a fortnight later is told which of these it was.
TAKEOVER = {
    "stop_driving": "somebody stopped the rover",
    "clear_map": "somebody cleared the map",
    "refit_pose": "somebody refitted the rover onto the map",
    "world_state_clear": "somebody emptied the world state",
    "world_state_rebuild": "somebody rebuilt the world state",
    "world_state_merge": "somebody joined things in the world state",
}


#: Where the executive is: beside this daemon on the rover (`~/ugv/autonomy`
#: next to `~/ugv`), and a sibling directory in the repository.
EXECUTIVE_PATHS = (Path(__file__).resolve().parent / "autonomy" / "executive.py",
                   Path(__file__).resolve().parent.parent / "autonomy" / "executive.py")
#: Where a started executive writes what it says, which is one line per goal.
EXECUTIVE_LOG = Path.home() / ".ugv" / "autonomy" / "executive.log"


def launch_executive(run_id: str) -> dict[str, Any]:
    """Start the executive as a process of its own, to attach to `run_id`.

    The same program a person starts by hand, with the same lack of authority:
    it finds the open run, renews the permit while it works, and exits when the
    run ends. Its own session, so that a daemon restart does not take it down
    mid-goal -- the run's lease is what decides how long it may go on.
    """
    path = next((one for one in EXECUTIVE_PATHS if one.is_file()), None)
    if path is None:
        return {"ok": False,
                "error": "the executive is not installed beside this daemon"}
    try:
        EXECUTIVE_LOG.parent.mkdir(parents=True, exist_ok=True)
        with open(EXECUTIVE_LOG, "ab") as log:
            stamp = time.strftime("%Y-%m-%dT%H:%M:%S")
            log.write(f"--- {stamp} starting for {run_id} ---\n".encode())
            log.flush()
            process = subprocess.Popen(
                # Unbuffered, so the log shows each goal as it happens.
                [sys.executable, "-u", str(path)], cwd=str(path.parent),
                stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True)
    except OSError as error:
        return {"ok": False, "error": f"the executive could not be started: {error}"}
    return {"ok": True, "pid": process.pid, "log": str(EXECUTIVE_LOG),
            "process": process}


class RoverAutonomy:
    """Autonomous permission, mixed into Rover."""

    #: What starts the executive once a run is open. A test replaces it.
    executive_launcher = staticmethod(launch_executive)

    # --- what opens a run ---------------------------------------------------

    def _opening(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """The run `autonomy_start` and `autonomy_enable` would open, or why not.

        From the console a run has no limit on time, travel or actions unless a
        budget is given (`permission.CONSOLE_BUDGET`); through the API what is
        left out takes the standing limits. Either may narrow the safe area with
        `budget.geofence`, and without one the mapped floor is the boundary.
        """
        via = str(arguments.get("via") or "api").strip()
        purpose = str(arguments.get("purpose") or arguments.get("why") or "").strip()
        budget = arguments.get("budget") or {}
        if not isinstance(budget, dict):
            return {"ok": False, "error": "budget is an object of limits"}
        facts = self.autonomy_conditions()
        if not facts.get("pose_trusted"):
            return {"ok": False,
                    "error": "the rover has not confirmed where it is on the map"}
        if facts.get("map_settled") is False:
            return {"ok": False, "error": "the map has not settled yet"}
        turning = permission_mod.rotation_fault(facts.get("gyro_bias_dps"))
        if turning:
            return {"ok": False, "error": turning}
        base = (permission_mod.CONSOLE_BUDGET if via == "console"
                else permission_mod.DEFAULT_BUDGET)
        where = facts.get("where")
        start = (None if where is None else
                 {"x_m": where[0], "y_m": where[1],
                  "heading_deg": facts.get("heading_deg"),
                  "map_id": facts.get("map_id")})
        answer = self.permission.enable(via=via, why=purpose, budget=budget,
                                        base=base, start=start)
        if answer.get("ok"):
            print(f"[autonomy] started via {via}: {answer['run']['id']}, "
                  f"budget {answer['run']['budget']}"
                  + (f", for {purpose}" if purpose else ""), flush=True)
        return answer

    @serialized
    def _tool_autonomy_start(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """Open a run and start the executive on it. The console's run button,
        and an agent's way in.

        `via` is `console` or `api` (the default), `purpose` says what the run
        is for, and `budget` may set `seconds`, `travel_m` and `actions` (a
        number, or null for no limit), `failures` (in a row), and `geofence` (a
        circle `{x_m, y_m, radius_m}` or a box of `min_x_m`/`max_x_m`/`min_y_m`/
        `max_y_m`, in map coordinates). Clears a stop, as opening any run does.
        """
        answer = self._opening(arguments)
        if not answer.get("ok"):
            return {**answer, "autonomy": self.permission.status()}
        run_id = answer["run"]["id"]
        launched = self.executive_launcher(run_id)
        if not launched.get("ok"):
            self.autonomy_end(launched.get("error") or "the executive did not start")
            return {"ok": False, "error": launched.get("error"),
                    "autonomy": self.permission.status()}
        # Held so the finished process is reaped rather than left a zombie.
        self._executive_process = launched.get("process")
        return {**answer, "executive": {"pid": launched.get("pid"),
                                        "log": launched.get("log")},
                "autonomy": self.permission.status()}

    @serialized
    def _tool_autonomy_enable(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """Open a run and start nothing: for a person running the executive by
        hand. The same arguments as `autonomy_start`."""
        answer = self._opening(arguments)
        return {**answer, "autonomy": self.permission.status()}

    def autonomy_brief(self) -> dict[str, Any] | None:
        """The open run as the console's run button needs it, or None.

        A read without the lock: `nav_status` asks three times a second, and a
        stale answer for one poll is all a race can cost.
        """
        permission = getattr(self, "permission", None)
        run = None if permission is None else permission.run
        if run is None or run.ended or permission.latch is not None:
            return None
        return {"id": run.id, "via": run.via, "why": run.why,
                "spent": run.spent(permission.clock())}

    # --- what a person calls ------------------------------------------------

    @serialized
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

    @serialized
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

    @serialized
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
        # Gather slow telemetry before taking the transition lock. A stop that
        # arrived during the read is then seen by check, not overwritten by it.
        with self._autonomy_lock:
            return self._autonomy_dispatch(permit, action, action_id, episode,
                                           params, facts)

    def _autonomy_dispatch(self, permit, action, action_id, episode, params, facts):
        verdict = self.permission.check(
            permit=permit, action=action, action_id=action_id,
            episode=episode, params=params, conditions=facts)
        if not verdict.ok:
            answer = {"ok": False, "refused": verdict.code, "error": verdict.why,
                      "autonomy": self.permission.status()}
            if verdict.already is not None:
                answer["already"] = verdict.already
            return answer

        if action == "drive_to" and facts.get("stop_seq") is None:
            return {"ok": False, "refused": "navigation guard unavailable",
                    "error": "navigation cannot invalidate a delayed autonomous goal"}

        self.permission.began(action_id, action, params, episode=episode)
        # Where the rover is standing as the action begins, so that the travel
        # budget is spent from the start of the move rather than from the
        # watchdog's first look at it. Half a second of driving is only 18 cm,
        # but an under-count that happens once per goal adds up over a run.
        self.permission.moved(facts.get("where"), facts.get("map_id"))
        try:
            if action == "drive_to":
                params = {**params, "stop_seq": facts.get("stop_seq"),
                          "geofence": self.permission.run.budget.get("geofence")}
            if action == "world_inspect":
                # Inspection does not drive; let a person's stop through while
                # the camera/resolver works. It has already been reserved once.
                self._autonomy_lock.release()
                try:
                    result = self._autonomy_do(action, params, action_id, episode)
                finally:
                    self._autonomy_lock.acquire()
            else:
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
            # A look for a hypothesis check may ask for the other calibrated tilt,
            # a fresh picture and its depth kept; `permission.check` has already
            # held the tilt to the two the bearings are calibrated at.
            # It waits for a look the rover's own looking is taking: on
            # 2026-10-03 six goals in nineteen were refused at that instant,
            # each counted as a failed goal.
            # A look that only records, as a run's geometry look does, may
            # wait longer for its turn: a settling pass can hold the camera for
            # most of ten seconds once 2,000 bearings are pending (M3 session
            # 6). A look that settles, or wakes the depth camera, keeps the
            # shorter wait, so that it finishes inside its permission.
            quick = (params.get("settle") is False
                     and not params.get("keep_depth"))
            return self._tool_world_inspect({
                "settle": params.get("settle", True),
                "tilt_deg": params.get("tilt_deg"),
                "aim_at": params.get("aim_at"),
                # The thing the look is aimed at, whose region is then filed to
                # it (world_state/aimed.py).
                "target": params.get("target"),
                "fresh": bool(params.get("fresh")),
                "keep_depth": bool(params.get("keep_depth")),
                "wait": True,
                **({"wait_s": AUTONOMY_LOOK_WAIT_S} if quick else {})})

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
                guard={"stop_seq": params.get("stop_seq"),
                       "geofence": params.get("geofence")},
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
            "stop_seq": status.get("stop_seq"),
            "where": (None if not pose else
                      (float(pose["x_m"]), float(pose["y_m"]))),
            "heading_deg": (None if not pose or pose.get("heading_deg") is None
                            else float(pose["heading_deg"])),
            "gyro_bias_dps": status.get("gyro_bias_dps"),
        })
        return facts

    def autonomy_tick(self) -> str:
        """One turn of the watchdog. Returns why the run ended, or ''.

        Cheap and silent while no run is open, which is almost always: the
        daemon calls this on a timer whether or not anything is happening, and a
        rover nobody has enabled autonomy on must not be paying for a bridge
        round trip twice a second.
        """
        process = getattr(self, "_executive_process", None)
        if process is not None and process.poll() is not None:
            self._executive_process = None
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
        with self._autonomy_lock:
            # The run may have been replaced while telemetry was being read.
            if self.permission.run is not run or run.ended:
                return ""
            return self._autonomy_tick_checked(facts, was, now)

    def _autonomy_tick_checked(self, facts, was, now):
        fault = self.permission.moved(facts.get("where"), facts.get("map_id"),
                                      elapsed_s=None if was is None else now - was)
        why = fault or self.permission.due(facts)
        if not why:
            # An inspection's own limits end the step and not the run: the
            # attempt is over, and the next goal is the executive's to choose.
            over = self.permission.inspection_over()
            if over:
                doing = self.permission.doing or {}
                self.permission.limit_reached(str(doing.get("id") or ""), over)
                if self.nav is not None and doing.get("action") == "drive_to":
                    self.nav.stop()
                print(f"[autonomy] {doing.get('id')} stopped: {over}", flush=True)
            return ""
        self.autonomy_end(why)
        return why

    @serialized
    def autonomy_end(self, why: str) -> dict[str, Any]:
        """Stop the wheels and close the run. Not a latch: nobody asked.

        The wheels first and the bookkeeping second, in that order and not the
        other, because everything after the stop is a record of something that
        has already been made safe.
        """
        # A background trip may be queued but not have taken the wheel mutex
        # yet. Send stop even then: its bridge sequence invalidates that goal.
        if self.nav is not None:
            try:
                self.nav.stop()
            except Exception as error:                         # pragma: no cover
                print(f"[autonomy] the run ended and the stop failed: {error}",
                      flush=True)
        ended = self.permission.end_run(why)
        if ended.get("ok"):
            print(f"[autonomy] {ended['ended_run']} ended: {why}", flush=True)
        return ended

    @serialized
    def autonomy_taken(self, by: str, why: str = "") -> dict[str, Any]:
        """A person took the rover back. Latch autonomy off and end any run."""
        run = self.permission.run
        running = run is not None and not run.ended
        answer = self.permission.stop(by=by, why=why)
        if running and self.nav is not None:
            self.nav.stop()
        if running:
            print(f"[autonomy] {answer.get('ended_run')} ended: "
                  f"{answer.get('why')}", flush=True)
        return answer

    @serialized
    def autonomy_notice(self, name: str) -> bool:
        """Called for every tool the daemon dispatches, and does nothing for
        almost all of them. True when this call ended a run that was open.

        It lives in `Rover.call` rather than in each movement tool because that
        is the one place every caller passes through, and a check that has to be
        remembered in eleven handlers is a check that will be missing from the
        twelfth. Autonomy's own actions do not come this way; see
        `_autonomy_do`.
        """
        if name not in HUMAN_MOVES:
            return False
        run = self.permission.run
        running = run is not None and not run.ended
        if not running:
            # Nothing to take over. A stop still latches, so that a person who
            # stopped the rover before enabling autonomy is not surprised by it
            # setting off; ordinary manual driving on a rover with no run open
            # is simply driving, and a second stop changes nothing.
            if name != "stop_driving" or self.permission.latch is not None:
                return False
        self.autonomy_taken("a person", TAKEOVER.get(name)
                            or f"somebody drove the rover by hand ({name})")
        return running

    def autonomy_handover(self) -> None:
        """Wait, briefly, for a stopped autonomous leg to let go of the wheels.

        Not serialized, and it must not be: the leg's own thread reports how it
        ended through `autonomy_trip_ended`, which takes the autonomy lock, and
        a wait held inside that lock would be a wait for itself. Returns when the
        navigator has no move running or after `TAKEOVER_HANDOVER_S`, whichever
        is first; a move that has still not let go by then is refused as busy,
        as before, and says so.
        """
        if self.nav is None:
            return
        deadline = time.monotonic() + TAKEOVER_HANDOVER_S
        while self.nav.driving and time.monotonic() < deadline:
            time.sleep(TAKEOVER_POLL_S)

    @serialized
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
