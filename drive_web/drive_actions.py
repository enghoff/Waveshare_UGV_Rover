"""What the console's buttons ask the rover for.

One posted action at a time, run on the pump thread so that nothing else is. The
buttons are the only place a person reaches the rover directly, so the rules that
matter are here: a move takes the target from whoever held it, a stop is allowed
to interrupt one, and a clear has to be pressed twice because it throws away a
map that took a drive to build.

Split from `drive_session.py` for the reason `SessionShow` and `SessionWorld`
were: the session is one object with several concerns, and this is the concern a
reader looking for "what happens when I press this" wants on its own.
"""
from __future__ import annotations

import time
from typing import Any

from console_model import (
    CLEAR_ARM_S, MAP_EXTENTS_M, WIFI_REJOIN_S, rung, tap_to_point,
)
from drive_show import _number

# How long a click that interrupted a move waits for the wheels to come free. The
# handover itself is inside a second, so this is only ever reached by a stop that
# did not land -- see mind_the_target for why waiting out the move channel's own
# four minutes instead would be worse than giving up.
TARGET_HANDOVER_S = 6.0


class SessionActions:
    """The half of `Session` that a button press lands in."""

    def act(self, action: dict[str, Any]) -> None:
        """One posted action, run on the pump thread so that nothing else is."""
        what = action.get("do")
        if what == "connect":
            self.wanted_address = str(action.get("address") or "")
            self.connect()
        elif what == "stop":
            self.stop()
        elif what == "drive":
            arguments: dict[str, Any] = {"distance_m": _number(
                action.get("distance_m"), 0.5)}
            speed = _number(action.get("speed_ms"), None)
            if speed is not None:
                arguments["speed_ms"] = speed
            self.move("drive", arguments)
        elif what == "turn":
            self.move("turn_in_place",
                      {"angle_deg": _number(action.get("angle_deg"), 90.0)})
        elif what == "explore":
            # Not `move`, which is for calls that hold the wheels until they
            # answer. This one answers in a moment and leaves the rover driving,
            # so it goes out on the status connection like any other button --
            # and what the header toggle then shows is the rover's own
            # `exploring`, not a call this console is waiting for.
            #
            # No arguments: the budget is the rover's default, because the one
            # thing a console is for is watching, and somebody watching can stop
            # it whenever they like. A box to type minutes into would be a
            # setting nobody has a reason to change with the STOP button in view.
            self.watch_call("explore")
        elif what == "tap":
            self.tap(action)
        elif what == "describe":
            self.watch_call("describe_surroundings")
        elif what == "map":
            self.map_settings(action)
        elif what == "track":
            name = action.get("name")
            if name in ("start_tracking", "stop_tracking"):
                self.watch_call(name)
                self.track_at = 0.0
        elif what == "lights":
            self.watch_call("set_lights",
                            {"level": int(_number(action.get("level"), 0))})
        elif what == "reset_lidar":
            self.reset_lidar()
        elif what == "clear_map":
            self.clear_map()
        elif what == "wifi_scan":
            self.wifi_scan()
        elif what == "wifi_join":
            self.wifi_join(str(action.get("ssid") or ""))
        elif what == "world":
            self.world_act(action)

    def watch_call(self, name: str, arguments: dict[str, Any] | None = None) -> None:
        if self.watch is None:
            self.say(f"not connected, so {name} was not sent", "bad")
            return
        self.watch.submit(name, arguments)

    def move(self, name: str, arguments: dict[str, Any]) -> None:
        """A bounded move, one at a time.

        Refused here rather than sent and refused by the daemon. The daemon's answer
        would be `busy`, which is correct and tells you nothing, and it would arrive
        as the notice on a move that is running perfectly well, where it reads like
        that move having failed.
        """
        if self.moves is None or not self.can_drive:
            self.say(f"no driving tools on this rover, so {name} was not sent", "bad")
            return
        if self.busy_since is not None:
            self.say(f"{self.busy_name} is still running; stop it or wait", "quiet")
            return
        self.busy_since = time.monotonic()
        self.busy_name = name
        self.moves.submit(name, arguments)

    def tap(self, action: dict[str, Any]) -> None:
        """A click on the picture is a place in the room, not a pixel -- and it
        outranks whatever the rover is doing when it lands.

        The page sends the pixel in the picture's own coordinates -- it divides out
        whatever CSS scaling the panel applied, which is the one piece of arithmetic
        it does -- and the conversion into metres happens here, in the renderer's own
        code. A browser that worked that out for itself would be a third copy of the
        map's geometry.

        **The place is asked for in map coordinates rather than as an offset from
        the rover, and that is what makes interrupting possible at all.** An offset
        is measured from wherever the rover has got to when the call arrives, and a
        click that interrupts a move arrives late by construction: the running move
        has to be stopped first, and the rover keeps driving until the stop lands.
        Sent as an offset, the click would mean a place most of a metre from the one
        under the cursor, and further out the faster the rover was going. Sent as a
        point on the map it means the same place however late it arrives.

        A click while something is already running is therefore not refused. It
        stops what is running and takes its place, because somebody clicking a
        second time is saying the rover is going to the wrong place, and "stop it or
        wait" is a console arguing with the only instruction it has.
        """
        if self.map_view is None:
            return
        if "drive_to" not in self.tools:
            self.say("this rover has no drive_to tool, so the tap was not sent", "quiet")
            return
        where = tap_to_point(_number(action.get("col"), 0.0),
                             _number(action.get("row"), 0.0), self.map_view)
        if where is None:
            self.say("cannot convert a tap without mapimg", "bad")
            return
        x_m, y_m = where
        arguments: dict[str, Any] = {"x_m": round(x_m, 2), "y_m": round(y_m, 2)}
        speed = _number(action.get("speed_ms"), None)
        if speed is not None:
            arguments["speed_ms"] = speed
        if self.busy_since is None:
            self.move("drive_to", arguments)
            return
        # Something is running. Stop it, and hold this until the wheels are free:
        # the running call occupies the move connection and cannot be overtaken on
        # it, and the daemon would refuse a second move as "busy" in any case. The
        # stop goes out on the connection that carries nothing else, and the move it
        # cancels answers within a control cycle of it landing, which is where the
        # waiting target is picked up. See `handle`.
        replacing = self.pending_target is not None
        self.pending_target = arguments
        self.pending_until = time.monotonic() + TARGET_HANDOVER_S
        if replacing:
            # The stop from the first click is already in flight, so a second one
            # would only say the same thing again.
            self.say(f"{self.new_target()} instead", "note")
            return
        self.say(f"{self.new_target()}, so the {self.busy_name} in flight is being "
                 f"stopped first", "note")
        self.stop(keep_target=True)

    def new_target(self) -> str:
        """The waiting click as a phrase, for the notice line."""
        target = self.pending_target or {}
        return ("a new target at x {:+.2f}, y {:+.2f}".format(
            float(target.get("x_m") or 0.0), float(target.get("y_m") or 0.0)))

    def hand_over(self) -> None:
        """Send the click that was waiting for the wheels, now that they are free."""
        arguments, self.pending_target = self.pending_target, None
        self.pending_until = 0.0
        if arguments is not None:
            self.move("drive_to", arguments)

    def forget_target(self, why: str) -> None:
        """Drop a waiting click, and say so. Silence is the bad outcome here: a
        click that quietly evaporated looks exactly like a console that ignores
        clicks."""
        if self.pending_target is None:
            return
        self.say(f"{self.new_target()} was dropped: {why}", "quiet")
        self.pending_target = None
        self.pending_until = 0.0

    def stop(self, keep_target: bool = False) -> None:
        """Always allowed, and on the connection that carries nothing else.

        A stop throws away a waiting click along with the move in flight, unless it
        *is* that click's own stop. Pressing STOP after clicking somewhere and then
        watching the rover set off for that place is the one behaviour nobody would
        forgive -- and the same goes for the stop that follows the last browser
        leaving, where the target was queued by a tab that has since been closed.
        """
        if not keep_target:
            self.forget_target("the rover was stopped")
        if self.halt is None:
            self.say("not connected, so there was nothing to stop", "quiet")
            return
        self.halt.submit("stop_driving")

    def take_picture(self) -> None:
        """On its own connection, because it is the slowest call here: a camera that
        has to be opened takes the rover up to four seconds to deliver a first
        buffer, and while it is doing that nothing else on that socket is answered."""
        if self.camera is None or self.frame_outstanding:
            return
        self.frame_outstanding = True
        self.frame_asked_at = time.monotonic()
        # The note is left saying what the last picture was. Replacing it with
        # "taking one..." for the second each capture takes was a change of state
        # twice a picture, and the panel it wrote to already has the picture on it.
        self.camera.submit("camera_jpeg")

    def reset_lidar(self) -> None:
        """Ask the rover to replug its own lidar.

        On the watch connection rather than the move one, because the point of it is
        to work when the rover is otherwise doing nothing useful -- and because it
        answers immediately: the reset is issued and the device takes a second or
        two to come back on its own, which the scan age will show.
        """
        if self.watch is None:
            return
        self.say("resetting the lidar's USB device", "note")
        self.watch_call("reset_lidar")

    def clear_map(self) -> None:
        """Two presses, and no dialog between them.

        `confirm()` blocks the page's script, which is the script receiving status
        and holding the stop button -- the same objection a desktop window has to a
        modal dialog sitting on its event loop, for the same reason. Arming the
        button costs one extra press and takes nothing away. It disarms itself
        after CLEAR_ARM_S, so a press forgotten about does not lie in wait.
        """
        now = time.monotonic()
        if now > self.clear_armed_until:
            self.clear_armed_until = now + CLEAR_ARM_S
            return
        self.clear_armed_until = 0.0
        if self.picture is None:
            self.say("not connected, so the map was not cleared", "bad")
            return
        # On the map's connection, so that a picture already being drawn comes back
        # before the clear rather than after it. The other way round shows an empty
        # map and then replaces it with the old one, which reads as the clear having
        # failed.
        self.picture.submit("clear_map")

    def map_settings(self, action: dict[str, Any]) -> None:
        """Zoom and which way is up, from the two controls left under the map.

        An extent, never a magnification: the picture is always the same number of
        pixels and the rover derives pixels per cell from the extent, which is what
        keeps the picture the same size when the view widens. Asking for a
        magnification instead resized the picture on every zoom, which is not
        zooming.
        """
        if "zoom" in action:
            index = rung(MAP_EXTENTS_M, self.half_extent) + int(action["zoom"])
            self.half_extent = MAP_EXTENTS_M[
                max(0, min(len(MAP_EXTENTS_M) - 1, index))]
        if "rover_up" in action:
            self.rover_up = bool(action["rover_up"])
        self.refresh_map()

    def refresh_map(self) -> None:
        """On its own connection, because the map is the slowest thing here.

        It shared the status connection at first, which was wrong once the cost was
        measured: a map at the default settings takes a couple of seconds on the rover,
        and `RoverClient` serialises, so every refresh held up a status poll that is
        meant to arrive three times a second. The numbers went stale exactly while
        the picture was being drawn.
        """
        if self.picture is None:
            return
        if self.map_outstanding:
            # One at a time, but do not lose the request: the map takes seconds, and
            # a zoom pressed while one is in flight would otherwise be dropped on
            # the floor and the next picture would come back at the old extent.
            # Remember that the settings moved and ask again as soon as this lands.
            self.map_wanted = True
            return
        self.map_wanted = False
        self.map_outstanding = True
        self.map_asked_at = time.monotonic()
        self.picture.submit("map_png", {"half_extent_m": self.half_extent,
                                        "pixels": self.map_size,
                                        "rover_up": self.rover_up})

    def wifi_scan(self) -> None:
        """Ask the radio to look around, which costs the rover the link for a moment.

        Said out loud in the panel before it happens, because it is fifteen seconds
        of a rover that answers nothing -- including a stop -- and a button that
        appears to have done nothing for that long is a button people press again.
        Which is also why it goes out until the answer arrives: pressing it twice
        buys two scans and half a minute off channel, not a quicker one.
        """
        if self.scanner is None:
            self.say("not connected, so no scan was sent", "bad")
            return
        self.wifi["note"] = "scanning -- the rover is off channel for a few seconds"
        self.wifi["scanning"] = True
        self.wifi_outstanding = True
        self.wifi_at = time.monotonic()
        self.scanner.submit("wifi_status", {"scan": True})

    def wifi_join(self, ssid: str) -> None:
        """Move the rover onto another network.

        The rover has one radio, so this costs the link. The daemon answers
        before it acts -- the reply arriving means the request was accepted and
        nothing more -- and what follows is the link going down under all six of
        this page's connections. The reconnect is therefore scheduled rather
        than waited for, because there is nothing left to be told on.
        """
        if not ssid or self.watch is None:
            return
        self.wifi["joining"] = ssid
        self.wifi_joining = ssid
        self.wifi["note"] = (f"joining {ssid}; the rover will be unreachable "
                             f"for a few seconds")
        self.watch_call("wifi_join", {"ssid": ssid})
        self.rejoin_at = time.monotonic() + WIFI_REJOIN_S

    def rejoined(self) -> None:
        """Reconnect after a join, whatever became of the request.

        Unconditional on purpose. The switch may have worked, may have failed and
        left the rover where it was, or may have left it on a network this desk
        cannot reach -- and the first two are indistinguishable from here until
        something reconnects and asks.
        """
        asked, self.wifi_joining = self.wifi_joining, None
        self.wifi["joining"] = None
        self.wifi["note"] = f"reconnecting after asking for {asked}"
        self.connect()
