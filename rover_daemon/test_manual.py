"""Manual driving: the controller read right, and the rover handed back after.

The joystick is fed the kernel's own 8-byte events and navigation's bridge is a
list of the commands it was sent. What matters is that the triggers and sticks
come out as the speeds and rates the mapping says, that a button held while the
controller connects is not a press, that nothing else may drive while it is
connected, and that its going away gives everything back.
"""
from __future__ import annotations

import math
import struct
import time

from test_fakes import FakeLink
from test_harness import check

#: The Xbox controller as it came up on the rover on 2026-11-12, through the
#: kernel's generic HID driver: eight axes and fifteen buttons.
XBOX_AXES = [0x00, 0x01, 0x02, 0x05, 0x09, 0x0A, 0x10, 0x11]
XBOX_BUTTONS = list(range(0x130, 0x13F))


def event(kind, number, value, init=False):
    return struct.pack("IhBB", 0, value, kind | (0x80 if init else 0), number)


def axis(number, value, init=False):
    return event(0x02, number, value, init)


def button(number, value, init=False):
    return event(0x01, number, value, init)


def opened():
    """What the driver says on opening: every axis and button as it stands."""
    return b"".join([axis(n, -32767 if n in (4, 5) else 0, True) for n in range(8)]
                    + [button(n, 0, True) for n in range(15)])


def test_the_controller_is_read_as_it_reports_itself():
    import rover_manual as m

    check("the generic driver's Xbox layout: stick on Z/RZ, triggers on GAS/BRAKE",
          m.layout(XBOX_AXES), {"rs_x": 0x02, "rs_y": 0x05, "lt": 0x0A, "rt": 0x09})
    check("...and xpad's: stick on RX/RY, triggers on Z/RZ",
          m.layout([0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x10, 0x11]),
          {"rs_x": 0x03, "rs_y": 0x04, "lt": 0x02, "rt": 0x05})

    pad = m.Pad("Xbox Wireless Controller", XBOX_AXES, XBOX_BUTTONS)
    check("opening reports no presses", pad.feed(opened()), [])
    check("...and at rest nothing moves", pad.command(), (0.0, 0.0, 0.0, 0.0))
    held = m.Pad("held", XBOX_AXES, XBOX_BUTTONS)
    check("a button held while the controller connects is not a press",
          held.feed(button(4, 1, True)), [])

    check("Y is a press of BTN_Y", pad.feed(button(4, 1)), [m.BTN_Y])
    check("...once, however long it is held", pad.feed(button(4, 1)), [])
    pad.feed(button(4, 0))
    check("X and B", pad.feed(button(3, 1) + button(1, 1)), [m.BTN_X, m.BTN_B])
    check("half an event waits for the rest of it",
          (pad.feed(button(0, 1)[:5]), pad.feed(button(0, 1)[5:])), ([], [m.BTN_A]))

    pad.feed(axis(4, 32767))
    check("RT pulled right in is full speed forward",
          round(pad.command()[0], 3), m.MAX_LINEAR_MS)
    pad.feed(axis(5, 32767))
    check("...and LT with it cancels it", pad.command()[0], 0.0)
    pad.feed(axis(4, -32767))
    check("LT alone is full speed backward", round(pad.command()[0], 3), -m.MAX_LINEAR_MS)
    pad.feed(axis(5, -32767 + 1000))
    check("a trigger barely touched is nothing", pad.command()[0], 0.0)

    pad.feed(axis(0, -32767))
    check("the left stick left is a left turn at full rate",
          round(math.degrees(pad.command()[1]), 1), m.MAX_TURN_DPS)
    pad.feed(axis(0, 3000))
    check("...and a stick resting slightly off centre is no turn", pad.command()[1], 0.0)
    pad.feed(axis(2, 32767) + axis(3, -32767))
    _, _, pan, tilt = pad.command()
    check("the right stick right and up pans right and tilts up",
          (round(pan), round(tilt)), (round(m.PAN_DPS), round(m.TILT_DPS)))


class FakePad:
    """A controller the test pushes events into, and can take away."""

    def __init__(self):
        import rover_manual as m

        self.pad = m.Pad("Xbox Wireless Controller", XBOX_AXES, XBOX_BUTTONS)
        self.name = self.pad.name
        self.queue = [opened()]
        self.gone = False
        self.closed = False

    def read(self):
        if self.gone:
            raise OSError("No such device")
        data, self.queue = b"".join(self.queue), []
        return self.pad.feed(data)

    def command(self):
        return self.pad.command()

    def close(self):
        self.closed = True


class FakeLine:
    def __init__(self):
        self.asked = []

    def ask(self, request):
        self.asked.append(request)
        blocked = "ahead" if request.get("linear", 0) > 0.4 else ""
        return {"ok": True, "manual": request.get("on"), "front_m": 0.35,
                "rear_m": None, "scan_age_s": 0.04, "blocked": blocked,
                "linear": 0.0 if blocked else request.get("linear", 0.0),
                "angular": request.get("angular", 0.0)}


class FakeNav:
    def __init__(self):
        self.held_by = None
        self.stops = 0

    def stop(self, latch=False):
        self.stops += 1
        return {"stopped": True, "latched": False}

    def close(self):
        pass


def _wait(until, seconds=3.0):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if until():
            return True
        time.sleep(0.02)
    return False


def test_a_controller_takes_the_wheels_and_gives_them_back():
    import rover_daemon
    import rover_manual as m

    link = FakeLink()
    rover = rover_daemon.Rover(link, "unused", device="/dev/null")
    rover.nav = FakeNav()
    pad, line = FakePad(), FakeLine()
    found = []

    def finder():
        if found:
            return None
        found.append(pad)
        return pad

    rover.start_manual(8773, finder=finder, line=line)
    try:
        check("a connected controller is manual mode",
              _wait(rover.manual_on), True)
        check("...which stops whatever was moving", rover.nav.stops, 1)
        check("...and holds every move off at the navigator",
              rover.nav.held_by, m.HELD)
        check("a drive asked for meanwhile is refused, and says why",
              rover.call("drive", {"distance_m": 0.5}),
              {"ok": False, "error": m.HELD})
        check("...and so is an autonomous run",
              rover.call("autonomy_start", {}).get("error"), m.HELD)

        pad.queue.append(axis(4, 16000))
        check("the trigger becomes a forward command to navigation",
              _wait(lambda: any(0 < (a.get("linear") or 0) < 0.4 for a in line.asked)),
              True)
        pad.queue.append(axis(4, 32767))
        check("...and navigation's cut is what the status shows",
              _wait(lambda: rover.manual_status().get("blocked") == "ahead"), True)
        pad.queue.append(axis(4, -32767))

        level = rover.level
        pad.queue.append(button(4, 1) + button(4, 0))
        check("Y switches the lights", _wait(lambda: rover.level != level), True)
        before = rover.manual_status()["voice"]
        pad.queue.append(button(3, 1) + button(3, 0))
        check("X is counted for the console to toggle the conversation",
              _wait(lambda: rover.manual_status()["voice"] == before + 1), True)

        sent = len(link.sent)
        pad.queue.append(axis(2, 32767))
        check("the right stick swings the camera",
              _wait(lambda: any(c.get("T") == 133 for c in link.sent[sent:])
                    and rover.pan > 1.0), True)
        pad.queue.append(axis(2, 0))

        check("the status says which controller and what it sees",
              {k: rover.manual_status().get(k) for k in ("on", "controller", "front_m")},
              {"on": True, "controller": "Xbox Wireless Controller", "front_m": 0.35})

        pad.gone = True
        check("the controller going away ends manual mode",
              _wait(lambda: not rover.manual_on()), True)
        check("...navigation is told", line.asked[-1], {"op": "manual", "on": False})
        check("...moves are allowed again", rover.nav.held_by, None)
        check("...and the device is let go of", pad.closed, True)
    finally:
        rover.stop_manual()


def test_nothing_changes_without_a_controller():
    import rover_daemon

    rover = rover_daemon.Rover(FakeLink(), "unused", device="/dev/null")
    rover.nav = FakeNav()
    line = FakeLine()
    rover.start_manual(8773, finder=lambda: None, line=line)
    try:
        time.sleep(0.2)
        check("no controller is no manual mode", rover.manual_on(), False)
        check("...navigation is never asked", line.asked, [])
        check("...and nothing is held", rover.nav.held_by, None)
    finally:
        rover.stop_manual()
    check("a daemon with no navigation does not look for a controller",
          (rover_daemon.Rover(FakeLink(), "unused", device="/dev/null")
           .start_manual(None), True), ("", True))


def test_the_store_waits_while_navigation_holds_the_map():
    """A drive by hand that left the map holds it after the controller has
    gone (ros_nav/off_map.py); the looking loop asks navigation, not the pad."""
    import rover_daemon

    class Holding(FakeNav):
        def __init__(self):
            super().__init__()
            self.off_map = {"since": 1.0, "left_at": None}
            self.map_held = ""
            self.asked = 0

        def status(self):
            self.asked += 1
            if self.off_map == "broken":
                raise OSError("no bridge")
            return {"off_map": self.off_map, "map_held": self.map_held}

    rover = rover_daemon.Rover(FakeLink(), "unused", device="/dev/null")
    rover.nav = nav = Holding()
    check("navigation holding the map stops the store", rover._world_held(10.0), True)
    nav.off_map = None
    check("...asked once a second, not every turn",
          (rover._world_held(10.5), nav.asked), (True, 1))
    check("...and given back when navigation lets go",
          rover._world_held(11.1), False)
    nav.off_map = "broken"
    check("a navigation that does not answer is not a hold",
          rover._world_held(12.2), False)
    nav.off_map = None
    nav.map_held = "position unconfirmed"
    check("a kept map the rover's place on is not confirmed stops it too",
          rover._world_held(12.2 + 1.1), True)
    rover.nav = None
    check("...nor is a rover with none", rover._world_held(14.5), False)


TESTS = (
    test_the_controller_is_read_as_it_reports_itself,
    test_a_controller_takes_the_wheels_and_gives_them_back,
    test_nothing_changes_without_a_controller,
    test_the_store_waits_while_navigation_holds_the_map,
)
