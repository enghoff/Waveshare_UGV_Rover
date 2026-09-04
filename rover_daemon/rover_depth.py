"""The depth camera's switch, and the rule that works it: the wheels.

Nobody presses this any more. The camera is on while the rover is driving and
for half a minute after it stops, and off the rest of the time, decided here on
a thread of its own. **The console used to carry a tick box for it and no longer
does**, because the question it asked -- is this rover doing anything that needs
a depth camera? -- is one the rover can answer for itself and a person at a
screen usually cannot: the camera earns its draw while the rover is moving
through a room it has not measured, and a parked rover photographing the same
wall over and over is the case the box existed to end.

What is left for a caller is `get_depth_power`, which reports and does not set.
It is not a model tool -- `tools()` builds its list from `tool_schemas.py` and
this is not in it -- for the same reason the world state's calls are not: what
the rover can afford is a decision about the rover rather than about the
conversation. Forcing the switch by hand is `POST /power` on the depth service
itself, documented in `oak_depth/README.md`, and this rule will move it back at
its next tick.

**Why the daemon owns the rule**, when the depth service runs on the same
machine and could watch a clock itself: only the daemon knows whether the wheels
are turning. The move mutex in `ros_navigator.py` is the fact -- every drive,
turn, tap on the map and explore run holds it -- so `driving` cannot drift out
of step with what the rover is actually doing, and a rover with no navigator at
all is left alone rather than guessed at.

The client is `world_state.depth_client`, reached through the ranger the world
state already keeps, so there is one place in this daemon that knows where the
depth service lives. What that means when the component is missing is that this
answers `supported: false` rather than failing -- see `_tool_get_depth_power`.
"""
from __future__ import annotations

import threading
import time
from typing import Any

#: How long the wheels stay still before the camera is switched off.
#:
#: Long enough that the pauses inside a drive are not spent waking up. A move
#: that has arrived lets go of the mutex while the next one is being decided --
#: a tap on the map answered, a leg of an explore finished and the next frontier
#: being chosen -- and switching off across those gaps would spend the whole of
#: the following leg in a four-to-six second firmware upload with no ranges.
DEPTH_IDLE_OFF_S = 30.0
#: How often the rule looks at the wheels. It reads a mutex and nothing else
#: unless the switch has to move, so this is paced by how late the camera may
#: start waking rather than by what asking costs.
DEPTH_TICK_S = 0.5
#: And how long to leave a depth service that would not answer before trying it
#: again, so a stopped service is not connected to twice a second for the life
#: of the daemon.
DEPTH_RETRY_S = 30.0


class RoverDepth:
    """Switching the depth camera off while the rover stands still, and on again."""

    #: When the wheels were last turning, and what the switch was last set to.
    #: Class attributes so that a bench `Rover` which never started the rule can
    #: still have `depth_tick` called at it -- the first tick starts the clock.
    _depth_moved_at: float | None = None
    _depth_on: bool | None = None
    _depth_tried_at: float = 0.0
    _depth_error: str = ""

    def _depth_ranger(self):
        """The depth camera's client, or None on a rover without the component.

        Borrowed from the world state rather than made a second time: two clients
        would be two copies of where the service listens, and this repository has
        the rule that the thing which owns a fact states it once.
        """
        borrow = getattr(self, "_world_ranger", None)
        return borrow() if callable(borrow) else None

    def start_depth_rule(self) -> str:
        """Put the camera's switch on the wheels, and answer with the line to log.

        Nothing on a rover that cannot drive. `driving` is false for the whole
        life of such a daemon, so the rule would switch the camera off half a
        minute after boot and never switch it on again -- which is a sensor
        quietly lost rather than a saving, and the honest answer to "is this
        rover moving?" from something with no navigator is that it does not know.
        """
        if getattr(self, "nav", None) is None:
            return ""
        self._depth_stop = threading.Event()
        thread = threading.Thread(target=self._depth_rule_loop, name="depth-power",
                                  daemon=True)
        self._depth_thread = thread
        thread.start()
        return (f"[rover] the depth camera follows the wheels -- off after "
                f"{DEPTH_IDLE_OFF_S:.0f} s standing still, on again when it drives")

    def _depth_rule_loop(self) -> None:
        """Never raises: it is a loop, and there is nobody to raise at."""
        while not self._depth_stop.wait(DEPTH_TICK_S):
            try:
                self.depth_tick()
            except Exception as error:          # never past here
                self._depth_error = f"{type(error).__name__}: {error}"

    def depth_tick(self) -> None:
        """One look at the wheels, and the switch if they have changed their mind.

        The switch is only touched when the answer differs from what was last
        asked for, so a parked rover costs one mutex read twice a second and a
        driving one the same. The first tick of all does send a switch-on, which
        is a no-op at the service and is worth the round trip: it is how this
        finds out what state the camera is actually in rather than assuming the
        one a fresh process starts in.

        A service that will not answer leaves `_depth_on` alone, so the decision
        stands and is made again at the next attempt; the attempts themselves are
        spaced out, because a depth service that has been stopped for the evening
        should not be connected to twice a second until morning.
        """
        if getattr(self, "nav", None) is None:
            return
        now = time.monotonic()
        if self._depth_moved_at is None:
            self._depth_moved_at = now
        if self.driving:
            self._depth_moved_at = now
            want = True
        else:
            want = now - self._depth_moved_at < DEPTH_IDLE_OFF_S
        if want is self._depth_on:
            return
        if self._depth_error and now - self._depth_tried_at < DEPTH_RETRY_S:
            return
        ranger = self._depth_ranger()
        if ranger is None:
            return
        self._depth_tried_at = now
        power = ranger.set_power(want)
        self._depth_error = power.error
        if not power.error:
            self._depth_on = want

    def _tool_get_depth_power(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        """Whether the depth camera is on, off, or still waking up.

        Three states rather than two, because waking is not a detail: the Myriad
        has no flash, so switching it on uploads the firmware and builds the
        stereo pipeline over USB every time, and that is four to six seconds
        measured on this rover, during which the camera is on and answering
        nothing. Anything drawing this as a lamp has to show that, or the first
        seconds of every drive will look like a camera that failed to come back.

        `supported` is the difference between a rover that has no depth camera
        and one whose depth camera is not answering this second. The first is
        permanent and worth taking the lamp off the screen for; the second is
        worth asking about again.
        """
        ranger = self._depth_ranger()
        if ranger is None:
            return {"ok": False, "supported": False,
                    "error": "this rover has no depth camera component installed"}
        power = ranger.power()
        if power.error:
            return {"ok": False, "supported": True, "error": power.error}
        return {"ok": True, "supported": True, "power": power.state,
                "since_s": round(power.since_s, 1)}
