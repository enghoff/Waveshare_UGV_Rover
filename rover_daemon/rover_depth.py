"""The daemon's side of the depth camera's switch: on, off, and how long it takes.

Two control calls and nothing else. They are not model tools -- `tools()` builds
its list from `tool_schemas.py` and neither of these is in it -- for the same
reason the world state's calls are not: switching a sensor off is a decision
about what the rover can afford, and a voice model that turned the depth camera
off mid-sentence would be a fault nobody could see. The person at the console
owns that call.

**Why the daemon is in the middle at all**, when the console runs on the same
machine as the depth service and could reach loopback 8770 itself: it does not
always run there. A console started on a desk against a rover across the LAN has
its own 127.0.0.1, and on that host 8770 is the console's own web server -- so a
toggle wired straight to loopback would, on a desk, be switching the wrong thing
entirely. The daemon is the one process that is on the rover by definition.

The client is `world_state.depth_client`, reached through the ranger the world
state already keeps, so there is one place in this daemon that knows where the
depth service lives. What that means when the component is missing is that this
answers `supported: false` rather than failing -- see `_tool_get_depth_power`.
"""
from __future__ import annotations

from typing import Any


class RoverDepth:
    """Switching the depth camera off to save what it draws, and on again."""

    def _depth_ranger(self):
        """The depth camera's client, or None on a rover without the component.

        Borrowed from the world state rather than made a second time: two clients
        would be two copies of where the service listens, and this repository has
        the rule that the thing which owns a fact states it once.
        """
        borrow = getattr(self, "_world_ranger", None)
        return borrow() if callable(borrow) else None

    def _tool_get_depth_power(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        """Whether the depth camera is on, off, or still waking up.

        Three states rather than two, because waking is not a detail: the Myriad
        has no flash, so switching it on uploads the firmware and builds the
        stereo pipeline over USB every time, and that is four to six seconds
        measured on this rover, during which the camera is on and answering
        nothing. Anything drawing this as a switch has to show that, or it will
        read as a switch that did not work.

        `supported` is the difference between a rover that has no depth camera
        and one whose depth camera is not answering this second. The first is
        permanent and worth hiding the control for; the second is worth asking
        about again.
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

    def _tool_set_depth_power(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """Switch it, and answer with what the camera is doing now.

        The answer to switching on is `waking` and not `on`, which is the honest
        one: the call returns as soon as the upload has been started, because
        holding it open across the wake would hold the console's status
        connection open for those seconds, and the lights, the map and the
        tracking panel all wait behind it.
        """
        if "on" not in arguments:
            return {"ok": False, "error": "say on: true or on: false"}
        ranger = self._depth_ranger()
        if ranger is None:
            return {"ok": False, "supported": False,
                    "error": "this rover has no depth camera component installed"}
        power = ranger.set_power(bool(arguments["on"]))
        if power.error:
            return {"ok": False, "supported": True, "error": power.error}
        return {"ok": True, "supported": True, "power": power.state,
                "since_s": round(power.since_s, 1)}
