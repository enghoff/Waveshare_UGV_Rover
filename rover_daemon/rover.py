"""Rover state: board, tools, and the mixins that own camera / wifi / nav."""
from __future__ import annotations

import threading
import time
from typing import Any

from board_link import (
    BATTERY_CELLS, BATTERY_MAX_AGE_S, CMD_LIGHTS, CMD_PROBE, PROBE_WAIT_S,
    _battery_percent, _battery_state, _battery_summary,
)
from rover_camera import RoverCamera, VisionLink
from rover_nav import CAMERA_FOV_DEG, RoverNav
from rover_util import _flag, _level, _number  # noqa: F401
from rover_wifi import RoverWifi
from tool_schemas import (
    LIGHT_MAX, LOOK_TOOL, MAP_POINT_TOOL, MAP_TOOL, NAV_TOOLS, SCRIPT_TOOL,
    START_SCRIPT_TOOL, STOP_SCRIPT_TOOL, TOOLS,
)


class Rover(RoverCamera, RoverWifi, RoverNav):
    """The rover's state and everything that may be done to it.

    One lock covers the board and the model of where things are pointed. The
    tracking loop runs on its own thread and takes the same lock for as long as
    it takes to send a servo command, so a tool call arriving mid-sweep is
    ordered against it rather than interleaved with it.
    """

    def __init__(self, link, service: str, device: str | None, size=(640, 480),
                 vision: str | None = None,
                 camera_fov_deg: float = CAMERA_FOV_DEG) -> None:
        self.link = link
        self.service = service
        self.device = device
        self.camera_fov_deg = camera_fov_deg
        self.size = size
        self.vision = VisionLink(vision) if vision and device else None

        self._lock = threading.RLock()
        self.level = 0
        self.pan = 0.0
        self.tilt = 0.0

        self._camera = None
        self._camera_used = 0.0
        self._detector = None
        self._loop_fps = 0.0
        # Where a turn of the loop goes, besides the detector: waiting for a
        # fresh frame, and telling the servos. Smoothed the same way and for the
        # same reason -- one slow turn is not news.
        self._wait_ms = 0.0
        self._aim_ms = 0.0
        self._aim: list[dict[str, Any]] = []
        # Overridden by whichever detector is opened, since the bar for
        # starting a lock is a property of that detector's scores.
        self._acquire_score = None
        # The tracking loop's newest frame, kept so that `look` has something to
        # send while the loop owns the camera. One 35kB JPEG, replaced in place.
        self._frame: tuple[bytes, float] | None = None

        self._tracking = threading.Event()
        self._thread: threading.Thread | None = None
        self.nav = None
        # The map picture the model was last shown, and what it takes to read a
        # place on it back out as a place in the room. None until it has looked.
        # See :meth:`_remember_map` in [rover_nav.py](rover_nav.py).
        self._map_shown: dict[str, Any] | None = None
        self._skip_centre = None
        self._skip_until = 0.0
        # What the loop last saw, for tracking_status and count_faces while it
        # is running -- the loop owns the camera then, so nothing else may look.
        self._seen = {"faces": 0, "locked": False, "at": 0.0, "where": []}
        # The detector is one kept-open connection that takes strictly one request
        # at a time, so two `count_faces` arriving together must not both be in it.
        # Its own lock rather than the board's: waiting on a detector on another
        # host is no reason a light cannot be switched or the wheels stopped, and
        # this call used to hold the board lock for exactly that wait.
        self._detector_lock = threading.Lock()
        # Set by main() once the port is known, since a script reaches the rover
        # by connecting back to this daemon like any other client. None on a
        # daemon that is not running scripts, which is what every call checks.
        self.scripts = None
        # The three scripting schemas, once something has asked for them. See
        # :meth:`script_tools` for why they are built rather than constant and
        # why they are worth keeping afterwards.
        self._script_tools: list[dict[str, Any]] | None = None
        # The last pack voltage and when it was read, because a console polls this
        # and every fresh sample is a read of the UART. Its own lock, so that two
        # clients asking at once are one read of the board rather than two.
        self._battery: float | None = None
        self._battery_at = 0.0
        self._battery_lock = threading.Lock()

        # The list of access points, for the same reason and at a longer interval:
        # a console polls this too, and one nmcli here costs a third of a second of
        # a core that is also running SLAM. What is not cached is the signal
        # strength, which is a file read, and so the panel stays live while the
        # thing behind it is answered three times a minute at most.
        self._wifi: list[dict[str, Any]] | None = None
        self._wifi_at = 0.0
        self._wifi_profiles: set[str] | None = None
        self._wifi_profiles_at = 0.0
        self._wifi_lock = threading.Lock()
        # What the last switch did, since the caller who asked for it cannot be
        # told: the switch takes the link down, and the reply would have gone out
        # over it. See :meth:`_tool_wifi_join`.
        self._wifi_join: dict[str, Any] | None = None

    # --- the board ----------------------------------------------------------

    def tools(self, local: bool = False) -> list[dict[str, Any]]:
        """What this rover can do, as this rover is currently configured.

        Built rather than constant, because `look` exists only when there is
        somewhere to send a picture. A tool that cannot reach the hardware it
        describes is worse than a missing one -- the model says it has done the
        thing, and nothing happens -- and the same is true of one that cannot
        reach the model's own host.

        The same rule covers driving, which needs a lidar, and the map, which needs
        both a lidar to build it and somewhere to send the picture.

        `local` is that same rule applied to the three tools whose condition is
        the caller rather than the hardware. Running code is refused on anything
        but loopback -- see `LOCAL_ONLY` in [rover_daemon.py](rover_daemon.py) --
        so a client across the LAN must not be shown any of them: the model would
        be offered tools that answer "reach it through an ssh tunnel" every time,
        which is exactly the lying schema this method exists to avoid. They come
        last because order is not cosmetic here; a tool is read against its
        neighbours, and everything measured about this list was measured with the
        others in this order.
        """
        tools = list(TOOLS)
        if self.vision is not None:
            tools.append(LOOK_TOOL)
        if self.nav is not None:
            tools += NAV_TOOLS
            if self.vision is not None:
                # Both of these need a picture on its way to the model rather
                # than only a lidar: one takes the map and the other is how a
                # place on that map is named, so a rover with nowhere to send a
                # picture would be offering a model a way to point at nothing.
                tools.append(MAP_TOOL)
                tools.append(MAP_POINT_TOOL)
        if local and self.scripts is not None:
            tools += self.script_tools()
        return tools

    def script_tools(self) -> list[dict[str, Any]]:
        """Writing a program, starting one that keeps going, and stopping it.

        `run_script`'s description names every primitive a program may call,
        because the alternative is a model guessing at them: a voice model asked
        for a behaviour in the middle of a conversation has one turn to write it,
        and a program against `lights.on()` -- which does not exist -- fails as a
        `NameError` several seconds later with nothing to show for it. Handing it
        `list_api` instead would need the model to ask before it writes, and a
        catalogue a model has only read is not reliably one it uses; see
        [docs/scripting.md](../docs/scripting.md).

        So the surface is generated from `rover_api` and pasted in, which keeps
        the one rule this repository has about descriptions: the thing that owns a
        fact is the thing that states it. The other two are literals that point at
        it rather than repeating it, since all three arrive together.

        In this order, and it is the order of a sentence rather than of a
        catalogue: the one a model reaches for most often first, then the same
        thing for something that has no end in it, then the way to end it. Built
        once and kept, because the first costs an `inspect` import and a walk of
        six namespaces, and `list_tools` is asked on every connection a console
        makes.
        """
        if self._script_tools is None:
            import copy

            import rover_api
            import scripting

            schema = copy.deepcopy(SCRIPT_TOOL)  # the literal is not ours to edit
            schema["function"]["description"] = (
                schema["function"]["description"].format(
                    api=rover_api.signatures(), limit_s=scripting.RUN_LIMIT_S))
            self._script_tools = [schema, START_SCRIPT_TOOL, STOP_SCRIPT_TOOL]
        return self._script_tools

    def describe(self) -> str:
        return self.link.describe()

    def probe(self, wait_s: float = PROBE_WAIT_S) -> bool:
        """Is the driver board actually there? Ask, then wait to be answered.

        **The waiting is the whole point, and leaving it out cost an afternoon.**
        This used to be `link.send(...)` alone, which reports whether the *write*
        succeeded -- and a write to a serial port succeeds whether or not anything
        is listening at the other end. So it returned True against a board that
        was unplugged, unpowered or not yet booted, and the daemon came up
        believing it had one.

        That mattered far more than a wrong startup message, because
        [run_daemon.sh](run_daemon.sh) is built on this returning False: the
        daemon is meant to exit when the board does not answer, and the supervisor
        retries every 15 s precisely because the ESP32 boots on its own schedule
        and the `@reboot` daemon can start first. With the check vacuous that loop
        never fired once. What actually happened at boot was a daemon holding a
        port the board was not yet talking on, for ever -- no telemetry, so no
        odometry, so no `odom -> base_link`, so slam_toolbox dropped every scan
        and there was no map. Nothing anywhere reported an error.

        `CMD_PROBE` is answered with the board's ordinary telemetry, so a fresh
        line is the proof. One `telemetry()` call is a 0.4 s wait; a board that
        needs longer than that is one this should exit over and let the supervisor
        come back to.
        """
        self.link.send({"T": CMD_PROBE})
        deadline = time.monotonic() + max(0.0, wait_s)
        while True:
            if self.link.telemetry() is not None:
                return True
            if time.monotonic() >= deadline:
                return False
            self.link.send({"T": CMD_PROBE})

    def _send_lights(self, level: int) -> bool:
        return self.link.send({"T": CMD_LIGHTS, "IO4": level, "IO5": level})

    def _send_gimbal(self) -> bool:
        # T:133 is the simple absolute form, and deliberately does not feed the
        # firmware's heartbeat: aiming is not driving, and must not be mistaken
        # for it by a base that stops itself when commands stop arriving.
        return self.link.send({"T": 133, "X": round(self.pan), "Y": round(self.tilt),
                               "SPD": 0, "ACC": 0})

    def centre_gimbal(self) -> bool:
        with self._lock:
            self.pan = self.tilt = 0.0
            return self._send_gimbal()

    def call(self, name: str, arguments: dict[str, Any]) -> dict[str, Any]:
        """Perform one tool call. Never raises: an error is a result too.

        What comes back goes into the model's context verbatim, so a failure has
        to read as an explanation rather than a traceback -- the model repeats
        the gist of it out loud.
        """
        handler = getattr(self, f"_tool_{name}", None)
        if handler is None:
            return {"ok": False, "error": f"no such tool: {name}"}
        try:
            return handler(arguments)
        except (TypeError, ValueError) as error:
            return {"ok": False, "error": str(error)}
        except Exception as error:  # a bug here must not take the daemon down
            return {"ok": False, "error": f"{type(error).__name__}: {error}"}

    def _tool_set_lights(self, arguments: dict[str, Any]) -> dict[str, Any]:
        level = _level(arguments.get("level"))
        with self._lock:
            if not self._send_lights(level):
                return {"ok": False, "error": "the driver board did not answer"}
            self.level = level
        return self._tool_get_lights({})

    def _tool_get_lights(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        return {"ok": True, "level": self.level, "on": self.level > 0}

    def _sample_battery(self) -> tuple[float | None, float]:
        """The pack voltage, and how many seconds old that reading is.

        Cached for BATTERY_MAX_AGE_S, which is what keeps a console polling every
        few seconds from reading the UART every few seconds. A battery is the
        slowest-moving thing on this rover -- the pack takes hours to go flat -- so
        the staleness costs nothing, and the age goes out alongside the number, so
        that a board which has stopped answering shows up as a reading getting old
        rather than as a reading.
        """
        with self._battery_lock:
            if (self._battery is None
                    or time.monotonic() - self._battery_at > BATTERY_MAX_AGE_S):
                # Asked of the link rather than assumed of it: everything that
                # embeds this daemon in a test brings its own, and a link that
                # cannot be read should come back as a sentence rather than as an
                # AttributeError.
                read = getattr(self.link, "telemetry", None)
                message = read() if read is not None else None
                volts = message.get("v") if isinstance(message, dict) else None
                if isinstance(volts, (int, float)) and not isinstance(volts, bool):
                    self._battery = float(volts) / 100.0
                    self._battery_at = time.monotonic()
            if self._battery is None:
                return None, 0.0
            return self._battery, time.monotonic() - self._battery_at

    def _tool_battery(self, _arguments: dict[str, Any]) -> dict[str, Any]:
        """How much charge is left, from the one thing here that measures anything.

        There is no fuel gauge on this rover, no coulomb counter and no current
        sense. The driver board reports the pack voltage and that is the whole of
        the evidence, so this is that voltage, read under whatever load the rover
        happens to be under and put through a discharge curve. It answers the
        question people actually ask -- whether to keep going -- and it will not
        tell two runs apart.
        """
        volts, age = self._sample_battery()
        if volts is None:
            return {"ok": False,
                    "error": "the driver board did not report a battery voltage"}
        state = _battery_state(volts)
        reading = {"ok": True, "volts": round(volts, 2), "state": state,
                   "cells": BATTERY_CELLS,
                   "volts_per_cell": round(volts / BATTERY_CELLS, 2),
                   "reading_age_s": round(age, 1),
                   "summary": _battery_summary(volts, state)}
        if state != "absent":
            reading["percent"] = _battery_percent(volts)
        return reading

    def close(self) -> None:
        # A running script first of all, because it is the only thing here that
        # will otherwise go on issuing calls into a daemon that is shutting down.
        if self.scripts is not None:
            self.scripts.close()
        # The navigator next, and outside the lock: it stops the wheels and joins
        # its own loop, and nothing else here matters until the rover is still.
        if self.nav is not None:
            self.nav.close()
            self.nav = None
        self.stop_tracking()
        with self._lock:
            self._close_camera()
            if self._detector is not None:
                self._detector.close()
            if self.vision is not None:
                self.vision.close()
            self.link.close()
