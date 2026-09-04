"""The depth camera's switch, as the console sees it: three states and no raise.

What matters here is not that a flag flips. It is that the two control calls
answer a browser in every state the camera can be in -- off, waking, on, not
answering, and not fitted at all -- because the console draws a switch from those
answers and a switch that goes blank when the camera is busy waking is a switch
somebody presses twice.
"""
from __future__ import annotations

from test_fakes import FakeLink
from test_harness import SKIP, check


def test_the_depth_camera_switch_answers_in_every_state():
    """Off, on, waking, unreachable, and never fitted, through the daemon's calls.

    The client is the fake ranger from `world_state.depth_client`, which answers
    `waking` to a switch-on exactly as the real service does. That is the case
    worth writing a test around: the honest answer to "switch it on" is not "on",
    because the firmware upload to a VPU with no flash has only just started.
    """
    try:
        import rover_daemon
        from world_state import depth_client
    except ImportError as exc:
        SKIP.append(f"depth camera switch ({type(exc).__name__})")
        return

    rover = rover_daemon.Rover(FakeLink(), "unused", device="/dev/null")
    fake = depth_client.FakeRanger()
    rover._world_ranger = lambda: fake

    reading = rover.call("get_depth_power", {})
    check("a camera that is on says so", reading["power"], "on")
    check("...and is offered as a control", reading["supported"], True)

    switched = rover.call("set_depth_power", {"on": False})
    check("switching it off answers off", switched["power"], "off")
    check("...and the switch reached the camera", fake.switches, [False])
    check("...and asking again agrees",
          rover.call("get_depth_power", {})["power"], "off")

    # The state the console exists to show. Switching on is a firmware upload, so
    # the honest answer at the moment of the call is that it has started.
    woken = rover.call("set_depth_power", {"on": True})
    check("switching it on answers waking, not on", woken["power"], "waking")

    check("a switch with no on/off is refused rather than guessed",
          rover.call("set_depth_power", {})["ok"], False)

    # A depth service that is not answering. Worth telling apart from a camera
    # that is off: this one is asked again, and the sentence goes on the panel.
    down = depth_client.FakeRanger(fail="Connection refused")
    rover._world_ranger = lambda: down
    refused = rover.call("get_depth_power", {})
    check("a service that is down is not a camera that is off", refused["ok"], False)
    check("...and it stays a thing worth asking about", refused["supported"], True)
    check("...and says what happened", refused["error"], "Connection refused")

    # And a rover with no depth camera at all, which is a control to hide rather
    # than an error to show every ten seconds.
    rover._world_ranger = lambda: None
    missing = rover.call("get_depth_power", {})
    check("a rover with no depth camera says so once", missing["supported"], False)
    check("...and switching it is refused in a sentence",
          "no depth camera" in rover.call("set_depth_power", {"on": True})["error"],
          True)


def test_switching_the_camera_never_raises_at_the_daemon():
    """Every call here goes through `Rover.call`, which must answer, not raise.

    The console asks for this on the same connection as the status poll, so an
    exception escaping one of these would take out the map, the lights and the
    tracking panel along with the switch.
    """
    try:
        import rover_daemon
    except ImportError as exc:
        SKIP.append(f"depth camera switch raising ({type(exc).__name__})")
        return

    rover = rover_daemon.Rover(FakeLink(), "unused", device="/dev/null")

    class Angry:
        def power(self):
            raise RuntimeError("the camera fell off the bus")

        def set_power(self, on):
            raise RuntimeError("the camera fell off the bus")

    rover._world_ranger = lambda: Angry()
    for name, arguments in (("get_depth_power", {}),
                            ("set_depth_power", {"on": True})):
        answer = rover.call(name, arguments)
        check(f"{name} answers a browser when the client throws",
              answer["ok"], False)
        check(f"...and {name} says what threw",
              "fell off the bus" in answer["error"], True)


TESTS = (
    test_the_depth_camera_switch_answers_in_every_state,
    test_switching_the_camera_never_raises_at_the_daemon,
)
