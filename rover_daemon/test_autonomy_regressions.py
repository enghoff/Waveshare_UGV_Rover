"""Reproductions from the September 8 review, with fake hardware only."""
import threading
from test_autonomy import Clock, a_rover, enabled, permitted, act
from test_harness import check


def test_stop_during_telemetry_refuses_dispatch():
    rover = a_rover(Clock())
    permit = permitted(rover, enabled(rover))
    original = rover.autonomy_conditions
    def conditions():
        facts = original()
        rover.call("autonomy_stop", {"by": "review"})
        return facts
    rover.autonomy_conditions = conditions
    result = act(rover, permit, "drive_to", "race#1", x_m=1., y_m=0.)
    check("stop during telemetry refuses pending drive", result["ok"], False)
    check("no drive reached navigation", rover.nav.sent, [])


def test_stop_and_dispatch_are_serialized():
    rover = a_rover(Clock())
    permit = permitted(rover, enabled(rover))
    checked, proceed, stopped = threading.Event(), threading.Event(), threading.Event()
    original = rover.permission.check
    def pause(**kwargs):
        verdict = original(**kwargs)
        checked.set()
        assert proceed.wait(5)
        return verdict
    rover.permission.check = pause
    drive = threading.Thread(target=lambda: act(rover, permit, "drive_to", "race#2", x_m=1., y_m=0.))
    drive.start()
    assert checked.wait(5)
    def stop():
        rover.call("autonomy_stop", {"by": "review"})
        stopped.set()
    thread = threading.Thread(target=stop)
    thread.start()
    check("stop cannot acknowledge across unfinished dispatch", stopped.wait(.05), False)
    proceed.set()
    drive.join(5); thread.join(5)
    check("stop acknowledged after serialized dispatch", stopped.is_set(), True)
    check("acknowledged stop leaves navigation stopped", rover.nav.driving, False)
    check("watchdog cannot restart it", rover.autonomy_tick(), "")


def test_clear_takes_wheels_as_well_as_permission():
    rover = a_rover(Clock())
    permit = permitted(rover, enabled(rover))
    act(rover, permit, "drive_to", "clear#1", x_m=2., y_m=0.)
    rover._tool_world_state_clear = lambda _: {"ok": True}
    rover.call("world_state_clear", {})
    check("clear latches authority", rover.permission.status()["latched"], True)
    check("clear cancels the existing trip", rover.nav.driving, False)


def test_watchdog_enforces_journey_boundary():
    clock = Clock(); rover = a_rover(clock)
    permit = permitted(rover, enabled(rover, geofence={"radius_m": 2.}))
    act(rover, permit, "drive_to", "fence#1", x_m=.2, y_m=0.)
    for x in [.2, .4, .6, .8, 1., 1.2, 1.4, 1.6]:
        clock.tick(.5); rover.nav.where=(x,0.)
        why = rover.autonomy_tick()
    check("boundary margin stops a detour before the edge", rover.nav.driving, False)
    check("boundary stop explains why", "safe area" in why, True)


def test_expiry_cancels_a_queued_background_trip():
    clock = Clock(); rover = a_rover(clock)
    permitted(rover, enabled(rover))
    rover.nav.driving = False  # background worker has not acquired the wheel mutex
    clock.tick(16)
    rover.autonomy_tick()
    check("expiry sends stop even before queued worker starts", rover.nav.stops, 1)


def test_navigation_must_support_guard_and_receives_it():
    rover = a_rover(Clock())
    fence = {"radius_m": 3.}
    permit = permitted(rover, enabled(rover, geofence=fence))
    act(rover, permit, "drive_to", "guard#1", x_m=1., y_m=0.)
    check("navigation receives the stop sequence and fence", rover.nav.guard,
          {"stop_seq":0,"geofence":fence})
    rover.nav.driving = False
    original = rover.autonomy_conditions
    def old_navigation():
        facts = original(); facts.pop("stop_seq",None); return facts
    rover.autonomy_conditions = old_navigation
    answer = act(rover, permit, "drive_to", "guard#2", x_m=1., y_m=0.)
    check("old navigation cannot silently ignore the guard", answer.get("refused"),
          "navigation guard unavailable")


TESTS = (test_stop_during_telemetry_refuses_dispatch,
         test_stop_and_dispatch_are_serialized,
         test_clear_takes_wheels_as_well_as_permission,
         test_watchdog_enforces_journey_boundary,
         test_expiry_cancels_a_queued_background_trip,
         test_navigation_must_support_guard_and_receives_it)
