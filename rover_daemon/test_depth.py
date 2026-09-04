"""The depth camera's switch, now that the wheels work it and nobody else does.

Two things matter here. The reporting call answers a browser in every state the
camera can be in -- off, waking, on, not answering, and not fitted at all --
because the console draws a lamp from those answers and a lamp that goes blank
while the camera wakes is a lamp somebody reports as a fault. And the rule
itself switches on the moment the rover drives and off half a minute after it
stops, without ever raising at the thread it runs on.
"""
from __future__ import annotations

import threading
import time

from test_fakes import FakeLink
from test_harness import SKIP, check


class _Wheels:
    """A navigator that only remembers when it last had the wheels.

    `wheels_at` is a property on the real one that answers now while a move is
    running and the moment it let go otherwise; a plain number is the same thing
    from the rule's side, and it lets a test move the clock about without a ROS
    graph behind it.
    """

    def __init__(self) -> None:
        self.wheels_at = time.monotonic()

    def moved(self) -> None:
        self.wheels_at = time.monotonic()


def _parked_rover():
    """A rover that can drive, with a fake depth camera and nothing moving."""
    import rover_daemon
    from world_state import depth_client

    rover = rover_daemon.Rover(FakeLink(), "unused", device="/dev/null")
    rover.nav = _Wheels()
    fake = depth_client.FakeRanger()
    rover._world_ranger = lambda: fake
    return rover, fake


def test_the_depth_camera_reports_in_every_state():
    """Off, on, waking, unreachable, and never fitted, through the daemon's call.

    The client is the fake ranger from `world_state.depth_client`, which answers
    `waking` to a switch-on exactly as the real service does. That is the case
    worth writing a test around: the honest answer to "switch it on" is not "on",
    because the firmware upload to a VPU with no flash has only just started.
    """
    try:
        import rover_daemon                                 # noqa: F401
        from world_state import depth_client
    except ImportError as exc:
        SKIP.append(f"depth camera switch ({type(exc).__name__})")
        return

    rover, fake = _parked_rover()

    reading = rover.call("get_depth_power", {})
    check("a camera that is on says so", reading["power"], "on")
    check("...and is a thing the console may draw", reading["supported"], True)

    fake.switched = "off"
    check("...and a camera that is off says that",
          rover.call("get_depth_power", {})["power"], "off")

    # A depth service that is not answering. Worth telling apart from a camera
    # that is off: this one is asked again, and the sentence goes on the panel.
    down = depth_client.FakeRanger(fail="Connection refused")
    rover._world_ranger = lambda: down
    refused = rover.call("get_depth_power", {})
    check("a service that is down is not a camera that is off", refused["ok"], False)
    check("...and it stays a thing worth asking about", refused["supported"], True)
    check("...and says what happened", refused["error"], "Connection refused")

    # And a rover with no depth camera at all, which is a lamp to take off the
    # screen rather than an error to show every ten seconds.
    rover._world_ranger = lambda: None
    missing = rover.call("get_depth_power", {})
    check("a rover with no depth camera says so once", missing["supported"], False)

    # The switch is no longer anybody's to throw from outside.
    check("nothing may set the power by hand any more",
          hasattr(rover_daemon.Rover, "_tool_set_depth_power"), False)


def test_the_camera_follows_the_wheels():
    """On while it drives, off half a minute after it stops, and nothing between.

    The clock is the tick's own `time.monotonic`, so the half minute is moved by
    winding the navigator's stamp back rather than by sleeping through it.
    """
    try:
        import rover_daemon                                 # noqa: F401
        import rover_depth
    except ImportError as exc:
        SKIP.append(f"depth camera rule ({type(exc).__name__})")
        return

    rover, fake = _parked_rover()

    # The first tick of all. The rover has only just started, so it is inside the
    # idle window and the camera is meant to be on; the switch goes out anyway,
    # which is how the rule learns what state the camera is really in.
    rover.depth_tick()
    check("the first tick asks for a camera that is on", fake.switches, [True])
    rover.depth_tick()
    check("...and the second asks for nothing, the answer not having changed",
          fake.switches, [True])

    # Half a minute of standing still.
    rover.nav.wheels_at -= rover_depth.DEPTH_IDLE_OFF_S + 1.0
    rover.depth_tick()
    check("a rover that has stood still switches the camera off",
          fake.switches, [True, False])
    check("...and it really went off", fake.power().state, "off")
    rover.depth_tick()
    check("...and is not switched off again every half second",
          fake.switches, [True, False])

    # And it drives. **This is the case the rule got wrong on the rover.** It
    # used to ask the navigator whether the wheels were turning *now*, and an
    # eleven-degree turn began and ended between two ticks -- so the camera never
    # woke, which is exactly the nudge somebody at the console makes. What is
    # read instead is when the wheels last moved, and a move over before the next
    # tick still leaves that behind.
    rover.nav.moved()
    rover.depth_tick()
    check("a move already over by the next tick still switches it on",
          fake.switches, [True, False, True])
    check("...and it answers waking rather than on", fake.power().state, "waking")

    # Still going half an hour later. The real navigator answers `now` for as
    # long as the move mutex is held, so nothing switches off mid-drive.
    rover.nav.moved()
    rover.depth_tick()
    check("a long drive never switches the camera off",
          fake.switches, [True, False, True])

    # And the half minute is counted from the last movement rather than from the
    # last time anything was switched.
    rover.nav.wheels_at -= rover_depth.DEPTH_IDLE_OFF_S - 1.0
    rover.depth_tick()
    check("twenty-nine seconds after the last move it is still on",
          fake.switches, [True, False, True])
    rover.nav.wheels_at -= 2.0
    rover.depth_tick()
    check("...and half a minute after it does go off",
          fake.switches, [True, False, True, False])


def test_a_rover_that_cannot_drive_keeps_its_camera():
    """No navigator, no rule -- and no camera quietly switched off for ever.

    Such a daemon never moves at all, so a rule that ran anyway would switch the
    camera off thirty seconds after boot and never have a reason to switch it on
    again.
    """
    try:
        import rover_daemon                                 # noqa: F401
        import rover_depth                                  # noqa: F401
    except ImportError as exc:
        SKIP.append(f"depth camera rule without driving ({type(exc).__name__})")
        return

    rover, fake = _parked_rover()
    rover.nav = None

    rover.depth_tick()
    check("a rover that cannot drive does not touch the switch", fake.switches, [])
    check("...and starts no thread to keep asking", rover.start_depth_rule(), "")


def test_the_rule_never_raises_at_its_own_thread():
    """A depth client that throws must not take the loop down with it.

    Nothing is waiting on this thread and nothing would report its traceback, so
    a camera that fell off the bus would end the rule silently and the console
    would show a lamp that had simply stopped changing.
    """
    try:
        import rover_daemon                                 # noqa: F401
        import rover_depth
    except ImportError as exc:
        SKIP.append(f"depth camera rule raising ({type(exc).__name__})")
        return

    rover, _ = _parked_rover()

    class Angry:
        def power(self):
            raise RuntimeError("the camera fell off the bus")

        def set_power(self, on):
            raise RuntimeError("the camera fell off the bus")

    rover._world_ranger = lambda: Angry()

    answer = rover.call("get_depth_power", {})
    check("get_depth_power answers a browser when the client throws",
          answer["ok"], False)
    check("...and says what threw", "fell off the bus" in answer["error"], True)

    rover._depth_stop = threading.Event()
    rover._depth_stop.set()          # so the loop runs its body no times over
    rover._depth_rule_loop()         # and returning at all is the check
    try:
        rover.depth_tick()
    except RuntimeError:
        pass                         # the tick itself may raise; the loop may not
    check("a client that throws leaves the rule able to run again",
          rover._depth_on, None)

    # A service that will not answer is retried on its own clock rather than
    # twice a second for the life of the daemon.
    from world_state import depth_client

    rover, _ = _parked_rover()
    down = depth_client.FakeRanger(fail="Connection refused")
    rover._world_ranger = lambda: down
    rover.depth_tick()
    check("a refusal is one attempt", down.switches, [True])
    rover.depth_tick()
    check("...and is not attempted again straight away", down.switches, [True])
    rover._depth_tried_at -= rover_depth.DEPTH_RETRY_S + 1.0
    rover.depth_tick()
    check("...but is attempted again later", down.switches, [True, True])


TESTS = (
    test_the_depth_camera_reports_in_every_state,
    test_the_camera_follows_the_wheels,
    test_a_rover_that_cannot_drive_keeps_its_camera,
    test_the_rule_never_raises_at_its_own_thread,
)
