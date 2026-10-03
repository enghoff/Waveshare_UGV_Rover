"""What one hypothesis inspection may spend, enforced by the daemon (R-AUT-12).

The executive declares a case, the place it tests, and limits on travel, time
and attempts; these checks drive the daemon's own rules with a fake navigator
and a clock wound by hand. The ones worth the closest reading are the budget
following the place -- a renamed case, a regenerated goal and a new run all
land on the attempts already spent -- and the watchdog stopping a drive at its
limit without ending the run.
"""
from __future__ import annotations

import permission as permission_mod
from test_autonomy import Clock, Ended, FakeNav, a_rover, enabled, permitted
from test_harness import check

PLACE = {"x_m": 3.0, "y_m": 0.0}
LIMITS = {"travel_m": 4.0, "seconds": 60.0, "attempts": 2}


def inspection(case: str = "case/7/3.00,0.00", target=None, limits=None) -> dict:
    return {"case": case, "target": dict(target or PLACE),
            "limits": dict(LIMITS if limits is None else limits)}


def act(rover, permit: str, action: str, action_id: str, episode: str,
        **params):
    return rover.call("autonomy_act",
                      {"permit": permit, "action": action,
                       "action_id": action_id, "episode": episode,
                       "params": params})


def test_an_inspection_must_declare_finite_limits_under_the_ceilings():
    rover = a_rover(Clock())
    permit = permitted(rover, enabled(rover))
    for name, limits in (
            ("with none", {}),
            ("with a zero", dict(LIMITS, seconds=0)),
            ("with no attempts said", {"travel_m": 4.0, "seconds": 60.0}),
            ("with more travel than the ceiling",
             dict(LIMITS, travel_m=permission_mod.INSPECTION_MAX_TRAVEL_M + 1)),
            ("with more attempts than the ceiling",
             dict(LIMITS, attempts=permission_mod.INSPECTION_MAX_ATTEMPTS + 1))):
        got = act(rover, permit, "drive_to", f"e#{name}", "episode:1",
                  x_m=1.5, y_m=0.0, inspection=inspection(limits=limits))
        check(f"an inspection {name} is refused", got.get("refused"),
              "inspection limits")


def test_the_viewpoint_is_never_on_top_of_the_place_under_test():
    rover = a_rover(Clock())
    permit = permitted(rover, enabled(rover))
    got = act(rover, permit, "drive_to", "e#1", "episode:1",
              x_m=2.7, y_m=0.0, inspection=inspection())
    check("a viewpoint 0.3 m from the place is refused",
          got.get("refused"), "inspection standoff")
    got = act(rover, permit, "drive_to", "e#2", "episode:1",
              x_m=1.5, y_m=0.0, inspection=inspection())
    check("...one 1.5 m from it is dispatched", got.get("ok"), True)


def test_a_viewpoint_further_than_the_attempt_may_drive_is_refused():
    rover = a_rover(Clock(), nav=FakeNav(where=(-3.0, 0.0)))
    permit = permitted(rover, enabled(rover))
    got = act(rover, permit, "drive_to", "e#1", "episode:1",
              x_m=1.5, y_m=0.0, inspection=inspection())
    check("a viewpoint 4.5 m away on a 4 m attempt is refused",
          got.get("refused"), "inspection travel")


def test_the_attempts_follow_the_place_and_not_the_name():
    clock = Clock()
    nav = FakeNav()
    rover = a_rover(clock, nav=nav)
    run = enabled(rover)
    permit = permitted(rover, run)
    for n in (1, 2):
        got = act(rover, permit, "drive_to", f"e{n}#1", f"episode:{n}",
                  x_m=1.5, y_m=0.0, inspection=inspection())
        check(f"attempt {n} on the place is dispatched", got.get("ok"), True)
        nav.driving = False
        rover._trip_ended("errand", nav.sent[-1]["for_what"], Ended("arrived"))
    got = act(rover, permit, "drive_to", "e3#1", "episode:3",
              x_m=1.5, y_m=0.0, inspection=inspection())
    check("a third is refused", got.get("refused"), "inspection exhausted")
    got = act(rover, permit, "drive_to", "e4#1", "episode:4",
              x_m=1.5, y_m=0.2,
              inspection=inspection(case="case/7/3.10,0.20",
                                    target={"x_m": 3.1, "y_m": 0.2}))
    check("...and so is the same place under another name, a merge or a "
          "regenerated goal", got.get("refused"), "inspection exhausted")
    got = act(rover, permit, "drive_to", "e5#1", "episode:5",
              x_m=1.5, y_m=1.2,
              inspection=inspection(case="case/7/3.00,1.20",
                                    target={"x_m": 3.0, "y_m": 1.2}))
    check("...while a place 1.2 m away is a different case", got.get("ok"),
          True)
    nav.driving = False
    rover._trip_ended("errand", nav.sent[-1]["for_what"], Ended("arrived"))

    rover.call("autonomy_release", {"run": run})
    permit = permitted(rover, enabled(rover))
    got = act(rover, permit, "drive_to", "e6#1", "episode:6",
              x_m=1.5, y_m=0.0, inspection=inspection())
    check("closing the run and opening another does not hand the attempts back",
          got.get("refused"), "inspection exhausted")


def test_a_later_request_cannot_widen_a_case():
    clock = Clock()
    nav = FakeNav()
    rover = a_rover(clock, nav=nav)
    permit = permitted(rover, enabled(rover))
    act(rover, permit, "drive_to", "e1#1", "episode:1", x_m=1.5, y_m=0.0,
        inspection=inspection(limits=dict(LIMITS, attempts=1)))
    nav.driving = False
    rover._trip_ended("errand", nav.sent[-1]["for_what"], Ended("arrived"))
    got = act(rover, permit, "drive_to", "e2#1", "episode:2", x_m=1.5, y_m=0.0,
              inspection=inspection(limits=dict(LIMITS, attempts=2)))
    check("a case first given one attempt is not given a second by asking",
          got.get("refused"), "inspection exhausted")


def test_an_attempt_that_has_used_its_time_may_not_look():
    clock = Clock()
    nav = FakeNav()
    rover = a_rover(clock, nav=nav)
    run = enabled(rover)
    permit = permitted(rover, run)
    act(rover, permit, "drive_to", "e1#1", "episode:1", x_m=1.5, y_m=0.0,
        inspection=inspection())
    nav.driving = False
    rover._trip_ended("errand", nav.sent[-1]["for_what"], Ended("arrived"))
    clock.tick(LIMITS["seconds"] + 1.0)
    permit = permitted(rover, run)
    got = act(rover, permit, "world_inspect", "e1#2", "episode:1",
              tilt_deg=0.0, inspection=inspection())
    check("a look after the attempt's minute is refused",
          got.get("refused"), "inspection exhausted")
    check("...saying it was the time", "s (" in got.get("error", ""), True)


def test_the_watchdog_stops_a_drive_at_the_attempts_limit_and_not_the_run():
    clock = Clock()
    nav = FakeNav(where=(0.0, 0.0))
    rover = a_rover(clock, nav=nav)
    run = enabled(rover)
    permit = permitted(rover, run)
    act(rover, permit, "drive_to", "e1#1", "episode:1", x_m=-1.5, y_m=0.0,
        inspection=inspection(target={"x_m": -3.0, "y_m": 0.0},
                              limits=dict(LIMITS, travel_m=2.0)))
    check("a viewpoint 1.5 m away on a 2 m attempt is dispatched",
          rover.permission.actions["e1#1"]["ok"], None)
    # A route far longer than its straight line: sideways and round.
    for step in ((0.0, 0.5), (0.0, 1.0), (0.0, 1.5), (0.0, 2.0), (0.0, 2.5)):
        clock.tick(permission_mod.TICK_S)
        rover.call("autonomy_permit", {"run": run})
        nav.where = step
        rover.autonomy_tick()
    check("the drive is stopped when the attempt has driven its two metres",
          nav.stops, 1)
    record = rover.permission.actions["e1#1"]
    check("...and the step failed with the reason",
          (record["ok"], "drove its" in record["detail"]), (False, True))
    check("...while the run goes on", rover.call("autonomy_status", {})["enabled"],
          True)
    nav.driving = False
    rover._trip_ended("errand", nav.sent[-1]["for_what"], Ended("stopped"))
    check("the navigator's own word for the stop does not turn it into success",
          rover.permission.actions["e1#1"]["ok"], False)


def test_a_look_may_be_taken_at_the_two_calibrated_tilts_only():
    rover = a_rover(Clock())
    permit = permitted(rover, enabled(rover))
    got = act(rover, permit, "world_inspect", "t#1", "episode:1", tilt_deg=45.0)
    check("a look at tilt 45 is refused", got.get("refused"), "tilt")
    got = act(rover, permit, "world_inspect", "t#2", "episode:1", tilt_deg=0.0)
    check("...a look at tilt 0 is not refused for its tilt",
          got.get("refused"), None)


def test_a_check_look_is_aimed_from_the_measured_heading():
    """Found on the rover: a check facing 34 degrees away from its place.

    The navigator called a turn arrived 17 degrees short, and its heading was
    17.5 degrees out besides, so the place was outside the depth camera's view.
    The pan is now worked out from where one scan says the rover faces, held to
    the twenty degrees the pan calibration covers.
    """
    from types import SimpleNamespace

    import rover_world

    def aimed(believed, measured, place, trusted=True):
        rover = SimpleNamespace(
            _world_pose=lambda: {"x_m": 0.0, "y_m": 0.0, "heading_deg": believed},
            _world_inspector=lambda: SimpleNamespace(heading=None),
            _world_measure=lambda _offset: {
                "trusted": trusted, "settled": measured == believed,
                "x_m": 0.0, "y_m": 0.0, "heading_deg": measured})
        return rover_world.RoverWorld._aim_pan(rover, place)

    ahead_left = {"x_m": 1.0, "y_m": 0.18}            # 10 degrees to the left
    check("a place ten degrees to the left is a pan ten degrees left",
          aimed(0.0, 0.0, ahead_left)["pan_deg"], -10.0)
    check("...worked out from the heading the scan measured, not the one "
          "believed", aimed(15.0, 0.0, ahead_left)["pan_deg"], -10.0)
    check("...and from the believed one when the scan cannot say",
          aimed(15.0, 0.0, ahead_left, trusted=False)["pan_deg"], 5.0)
    check("a place further round than the calibration is aimed at its limit",
          aimed(0.0, 0.0, {"x_m": 0.0, "y_m": 1.0})["pan_deg"], -20.0)


def test_a_check_look_whose_depth_service_was_down_is_taken_once_more():
    """Found in M0a's runs of 2026-10-02: four check looks in eleven kept no
    depth because the depth service was restarting after the camera dropped off
    USB. The look is retaken once the camera answers again; a second failure
    stands, and an ordinary look is never retaken."""
    from types import SimpleNamespace

    import rover_world

    def run(results, keep_depth=True, wakes=("", "")):
        looks, woken = list(results), list(wakes)
        taken = []

        def inspect(**kwargs):
            taken.append(kwargs)
            return dict(looks.pop(0))
        rover = SimpleNamespace(
            _world_ready=lambda: "",
            depth_wake=lambda _wait: woken.pop(0) if woken else "",
            _world_inspector=lambda: SimpleNamespace(inspect=inspect),
            _world_store=lambda: SimpleNamespace(
                depth=lambda frame: "kept" if frame == "f2" else None),
            centre_gimbal=lambda *a, **k: True,
            _aim_pan=lambda aim: {"pan_deg": 0.0})
        got = rover_world.RoverWorld._tool_world_inspect(
            rover, {"settle": False, "fresh": True, "keep_depth": keep_depth})
        return got, taken

    down = {"frame_id": "f1", "detail": "no ranges (ConnectionRefusedError: refused)"}
    fine = {"frame_id": "f2", "detail": "3 regions kept"}
    got, taken = run([down, fine])
    check("a check look that found the depth service down is taken again",
          (got["frame_id"], len(taken)), ("f2", 2))
    check("...and says which look it replaced",
          got["depth_retry"]["first_frame_id"], "f1")
    got, taken = run([down, dict(down, frame_id="f3")])
    check("...once: a second failure stands", (got["frame_id"], len(taken)), ("f3", 2))
    got, taken = run([down], wakes=("", "the depth camera was still waking"))
    check("a camera that does not come back is not looked at again",
          (got["frame_id"], len(taken), got.get("depth_note")),
          ("f1", 1, "the depth camera was still waking"))
    got, taken = run([down], keep_depth=False)
    check("an ordinary look is never retaken", len(taken), 1)


def test_an_autonomous_look_waits_for_the_rovers_own_look():
    """Found on 2026-10-03: six of nineteen geometry goals in runs 3 and 4 were
    refused at the look, because the rover's own once-a-second look held the
    camera at that instant, and each refusal counted as a failed goal. The
    rover's own looking still never waits."""
    from types import SimpleNamespace

    import rover_autonomy
    import rover_world

    taken = []
    rover = SimpleNamespace(
        _world_ready=lambda: "",
        _world_inspector=lambda: SimpleNamespace(
            inspect=lambda **kwargs: taken.append(kwargs) or {"ok": True}),
        centre_gimbal=lambda *a, **k: True,
        _aim_pan=lambda aim: {"pan_deg": 0.0})
    rover._tool_world_inspect = lambda arguments: (
        rover_world.RoverWorld._tool_world_inspect(rover, arguments))
    rover_autonomy.RoverAutonomy._autonomy_do(
        rover, "world_inspect", {"settle": True}, "a#2", "episode:1")
    check("a run's look waits for one already being taken",
          taken[-1]["wait_s"] > 0.0, True)
    rover._tool_world_inspect({"settle": False})
    check("...and the rover's own looking does not", taken[-1]["wait_s"], 0.0)


TESTS = (
    test_an_inspection_must_declare_finite_limits_under_the_ceilings,
    test_the_viewpoint_is_never_on_top_of_the_place_under_test,
    test_a_viewpoint_further_than_the_attempt_may_drive_is_refused,
    test_the_attempts_follow_the_place_and_not_the_name,
    test_a_later_request_cannot_widen_a_case,
    test_an_attempt_that_has_used_its_time_may_not_look,
    test_the_watchdog_stops_a_drive_at_the_attempts_limit_and_not_the_run,
    test_a_look_may_be_taken_at_the_two_calibrated_tilts_only,
    test_a_check_look_is_aimed_from_the_measured_heading,
    test_a_check_look_whose_depth_service_was_down_is_taken_once_more,
    test_an_autonomous_look_waits_for_the_rovers_own_look,
)
