"""Manual driving with a game controller.

**A connected controller is manual mode.** When a joystick appears under
`/dev/input` -- a paired controller switched on (rover_bluetooth.py), or one on
USB -- the rover stops whatever it was doing and takes its orders from it, until
the controller goes away. Nothing else changes while there is no controller.

    RT / LT          forward / backward, by how far the trigger is pulled
    left stick       turn, left and right
    right stick      the camera: pan and tilt
    Y                headlights on or off
    B                face tracking on or off
    X                the voice conversation on or off (the console starts it)

**What manual mode switches off.** No move, exploration or autonomous run may
start, and one running is stopped; the mapper stops drawing, the change watch
stops counting and the world store records nothing: the daemon's own looking
loop here, and the mapper, the change watch and the world model's scan log
through navigation's `/map_hold` (ros_nav/nav_manual.py). Localization goes
on, so the rover still knows where it is when the controller goes away -- and
if a drive took it off its map, navigation goes on holding the map until it is
back on it (ros_nav/off_map.py).

**The wheels are driven through navigation, not around it.** Each tick the
command goes to the bridge as `op: manual`, which checks it against the newest
lidar scan -- nothing within 20 cm in the way it is driving, front or back,
turning always allowed (ros_nav/manual_gate.py) -- and publishes what survives
on `/cmd_vel`, so the measured drive curves and the half-second timeouts that
stop a rover nobody is commanding apply here as everywhere else. That makes the
bridge a condition of driving: with navigation down, the stick does nothing,
because nothing could check the way. The bridge's manual mode is a lease, so a
daemon that dies leaves the rover in it for two seconds at most.

The joystick is read through the kernel's `js` interface, which this user can
read without being in the `input` group. Which axis is which comes from the
device itself (JSIOCGAXMAP): an Xbox controller over Bluetooth with the kernel's
generic HID driver puts the right stick on Z/RZ and the triggers on GAS/BRAKE,
the `xpad` driver puts the stick on RX/RY and the triggers on Z/RZ.
"""
from __future__ import annotations

import array
import glob
import json
import math
import os
import socket
import struct
import threading
import time
from typing import Any

from tool_schemas import LIGHT_MAX

#: Where joysticks appear.
JS_GLOB = "/dev/input/js*"
#: The loop's period, and how often a missing controller is looked for.
TICK_S = 0.05
FIND_EVERY_S = 1.0
#: The fastest the triggers drive and the left stick turns. 0.5 m/s is well under
#: what the chassis can do (0.68 m/s at PWM 140, drive_mixer.py) and is what the
#: 20 cm stop was sized against (manual_gate.REACTION_S).
MAX_LINEAR_MS = 0.5
MAX_TURN_DPS = 80.0
#: How fast the right stick swings the camera at full deflection.
PAN_DPS = 120.0
TILT_DPS = 90.0
#: Stick and trigger travel ignored around rest.
STICK_DEAD = 0.15
TRIGGER_DEAD = 0.05
#: The bridge, and how long one manual command may take to answer.
BRIDGE_TIMEOUT_S = 0.3
BRIDGE_RETRY_S = 1.0
#: What a move refused in manual mode is told.
HELD = "somebody is driving with the game controller"

# The kernel's numbers (linux/input-event-codes.h, linux/joystick.h).
ABS_X, ABS_Y, ABS_Z, ABS_RX, ABS_RY, ABS_RZ = 0x00, 0x01, 0x02, 0x03, 0x04, 0x05
ABS_GAS, ABS_BRAKE = 0x09, 0x0A
BTN_A, BTN_B, BTN_X, BTN_Y = 0x130, 0x131, 0x133, 0x134
JS_EVENT_BUTTON, JS_EVENT_AXIS, JS_EVENT_INIT = 0x01, 0x02, 0x80
JSIOCGAXES, JSIOCGBUTTONS = 0x80016A11, 0x80016A12
JSIOCGAXMAP, JSIOCGBTNMAP = 0x80406A32, 0x84006A34
JSIOCGNAME_128 = 0x80006A13 | (128 << 16)
EVENT = struct.Struct("IhBB")


def layout(axis_codes: list[int]) -> dict[str, int]:
    """Which axis is the right stick and which are the triggers."""
    if ABS_RX in axis_codes and ABS_RY in axis_codes:
        return {"rs_x": ABS_RX, "rs_y": ABS_RY, "lt": ABS_Z, "rt": ABS_RZ}
    return {"rs_x": ABS_Z, "rs_y": ABS_RZ, "lt": ABS_BRAKE, "rt": ABS_GAS}


def dead(value: float, zone: float) -> float:
    """A stick or trigger with its rest band taken out, rescaled to the full range."""
    if abs(value) <= zone:
        return 0.0
    return math.copysign((abs(value) - zone) / (1.0 - zone), value)


class Pad:
    """One joystick: its layout, its axes and buttons as they stand, and presses."""

    def __init__(self, name: str, axis_codes: list[int], button_codes: list[int],
                 fd: int | None = None) -> None:
        self.name = name
        self.fd = fd
        self.axis_codes = list(axis_codes)
        self.button_codes = list(button_codes)
        self.values = [0] * len(self.axis_codes)
        self.buttons = [0] * len(self.button_codes)
        self.roles = layout(self.axis_codes)
        self._pending = b""

    @classmethod
    def open(cls, path: str) -> "Pad":
        import fcntl

        fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        try:
            count = array.array("B", [0])
            fcntl.ioctl(fd, JSIOCGAXES, count)
            axes = count[0]
            fcntl.ioctl(fd, JSIOCGBUTTONS, count)
            buttons = count[0]
            name = array.array("B", [0] * 128)
            fcntl.ioctl(fd, JSIOCGNAME_128, name)
            axis_map = array.array("B", [0] * 64)
            fcntl.ioctl(fd, JSIOCGAXMAP, axis_map)
            button_map = array.array("H", [0] * 512)
            fcntl.ioctl(fd, JSIOCGBTNMAP, button_map)
        except OSError:
            os.close(fd)
            raise
        label = name.tobytes().split(b"\0")[0].decode("utf-8", "replace") or path
        return cls(label, list(axis_map[:axes]), list(button_map[:buttons]), fd)

    def read(self) -> list[int]:
        """Everything the device has said since last time; the buttons pressed.

        Raises OSError when the device has gone, which is how a controller
        switched off or out of range is noticed.
        """
        chunks = []
        while self.fd is not None:
            try:
                chunk = os.read(self.fd, EVENT.size * 64)
            except BlockingIOError:
                break
            if not chunk:
                raise OSError("the joystick closed")
            chunks.append(chunk)
        return self.feed(b"".join(chunks))

    def feed(self, data: bytes) -> list[int]:
        data = self._pending + data
        whole = len(data) - len(data) % EVENT.size
        self._pending = data[whole:]
        pressed = []
        for offset in range(0, whole, EVENT.size):
            _time, value, kind, number = EVENT.unpack_from(data, offset)
            if kind & JS_EVENT_AXIS and number < len(self.values):
                self.values[number] = value
            elif kind & JS_EVENT_BUTTON and number < len(self.buttons):
                # A press, and not the state the driver reports on opening:
                # a button held while the controller connects is not a press.
                if value and not self.buttons[number] and not kind & JS_EVENT_INIT:
                    pressed.append(self.button_codes[number])
                self.buttons[number] = value
        return pressed

    def axis(self, code: int) -> float:
        try:
            return self.values[self.axis_codes.index(code)] / 32767.0
        except ValueError:
            return 0.0

    def trigger(self, code: int) -> float:
        """A trigger, 0 at rest and 1 pulled right in. The kernel reports one
        as an axis from -32767 to 32767, resting at the bottom."""
        if code not in self.axis_codes:
            return 0.0
        return min(1.0, max(0.0, (self.axis(code) + 1.0) / 2.0))

    def command(self) -> tuple[float, float, float, float]:
        """m/s, rad/s, and the camera's pan and tilt in degrees a second."""
        throttle = (dead(self.trigger(self.roles["rt"]), TRIGGER_DEAD)
                    - dead(self.trigger(self.roles["lt"]), TRIGGER_DEAD))
        steer = dead(self.axis(ABS_X), STICK_DEAD)
        # Stick left is negative and a left turn is a positive angular rate.
        angular = -steer * math.radians(MAX_TURN_DPS)
        # Pan is positive to the right (memory of the gimbal: aiming.py), and
        # stick up is negative, while tilt is positive up.
        pan = dead(self.axis(self.roles["rs_x"]), STICK_DEAD) * PAN_DPS
        tilt = -dead(self.axis(self.roles["rs_y"]), STICK_DEAD) * TILT_DPS
        return throttle * MAX_LINEAR_MS, angular, pan, tilt

    def close(self) -> None:
        if self.fd is not None:
            try:
                os.close(self.fd)
            except OSError:
                pass
            self.fd = None


class BridgeLine:
    """One held connection to navigation's bridge, for a command every tick."""

    def __init__(self, port: int, host: str = "127.0.0.1") -> None:
        self.address = (host, port)
        self._file = None
        self._socket = None
        self._retry_at = 0.0

    def ask(self, request: dict[str, Any]) -> dict[str, Any]:
        if self._file is None:
            if time.monotonic() < self._retry_at:
                return {"ok": False, "error": "navigation is not answering"}
            try:
                self._socket = socket.create_connection(self.address, BRIDGE_TIMEOUT_S)
                self._socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self._socket.settimeout(BRIDGE_TIMEOUT_S)
                self._file = self._socket.makefile("rwb")
            except OSError as error:
                self._retry_at = time.monotonic() + BRIDGE_RETRY_S
                self.close()
                return {"ok": False, "error": f"navigation is not answering: {error}"}
        try:
            self._file.write(json.dumps(request).encode() + b"\n")
            self._file.flush()
            line = self._file.readline()
            if not line:
                raise OSError("the bridge closed the connection")
            reply = json.loads(line)
        except (OSError, ValueError) as error:
            self._retry_at = time.monotonic() + BRIDGE_RETRY_S
            self.close()
            return {"ok": False, "error": f"navigation stopped answering: {error}"}
        return reply if isinstance(reply, dict) else {"ok": False, "error": "not an object"}

    def close(self) -> None:
        for thing in (self._file, self._socket):
            try:
                if thing is not None:
                    thing.close()
            except OSError:
                pass
        self._file = self._socket = None


class RoverManual:
    """Manual driving, mixed into Rover. Started by `start_manual`."""

    def _manual_init(self) -> None:
        self._manual_lock = threading.Lock()
        self._manual: dict[str, Any] = {"on": False}
        self._manual_stop = threading.Event()
        self._manual_thread: threading.Thread | None = None
        #: How many times X has been pressed, ever: the console toggles the
        #: conversation each time this goes up.
        self._manual_voice = 0

    def manual_on(self) -> bool:
        with self._manual_lock:
            return bool(self._manual.get("on"))

    def manual_status(self) -> dict[str, Any]:
        with self._manual_lock:
            return {**self._manual, "voice": self._manual_voice}

    def start_manual(self, bridge_port: int | None,
                     finder=None, line=None) -> str:
        """Watch for a controller. Nothing to do without navigation to drive by."""
        if not bridge_port:
            return ""
        self._manual_finder = finder or _find_pad
        self._manual_line = line or BridgeLine(bridge_port)
        self._manual_thread = threading.Thread(target=self._manual_loop,
                                               name="manual", daemon=True)
        self._manual_thread.start()
        return ""

    def stop_manual(self) -> None:
        self._manual_stop.set()
        thread = self._manual_thread
        if thread is not None:
            thread.join(timeout=2.0)

    # --- the loop ---------------------------------------------------------------

    def _manual_loop(self) -> None:
        pad: Pad | None = None
        look_at = 0.0
        last = time.monotonic()
        try:
            while not self._manual_stop.wait(TICK_S):
                now = time.monotonic()
                dt, last = min(now - last, 0.2), now
                if pad is None:
                    if now >= look_at:
                        look_at = now + FIND_EVERY_S
                        pad = self._manual_finder()
                        if pad is not None:
                            self._manual_begin(pad)
                    continue
                try:
                    self._manual_tick(pad, dt)
                except OSError:
                    self._manual_end("the controller went away")
                    pad.close()
                    pad = None
        finally:
            if pad is not None:
                self._manual_end("the daemon is stopping")
                pad.close()

    def _manual_tick(self, pad: Pad, dt: float) -> None:
        for code in pad.read():
            self._manual_button(code)
        linear, angular, pan, tilt = pad.command()
        reply = self._manual_line.ask({"op": "manual", "on": True, "linear": linear,
                                       "angular": angular, "who": pad.name})
        self._manual_camera(pan, tilt, dt)
        with self._manual_lock:
            if reply.get("ok"):
                self._manual.update({
                    "blocked": reply.get("blocked") or "",
                    "front_m": reply.get("front_m"), "rear_m": reply.get("rear_m"),
                    "scan_age_s": reply.get("scan_age_s"),
                    "linear": reply.get("linear"), "angular": reply.get("angular"),
                    "error": ""})
            else:
                self._manual.update({"blocked": "", "linear": 0.0, "angular": 0.0,
                                     "error": str(reply.get("error") or "no answer")})

    def _manual_begin(self, pad: Pad) -> None:
        """A controller appeared: whatever the rover was doing ends here."""
        with self._manual_lock:
            self._manual = {"on": True, "controller": pad.name, "since": time.time(),
                            "blocked": "", "front_m": None, "rear_m": None,
                            "scan_age_s": None, "linear": 0.0, "angular": 0.0,
                            "error": "", "note": ""}
        nav = getattr(self, "nav", None)
        if nav is not None:
            nav.held_by = HELD
        # Through `call`, so that an autonomous run is ended as a person
        # stopping the rover ends one.
        self.call("stop_driving", {})
        print(f"[rover] manual driving: {pad.name}", flush=True)

    def _manual_end(self, why: str) -> None:
        self._manual_line.ask({"op": "manual", "on": False})
        nav = getattr(self, "nav", None)
        if nav is not None:
            nav.held_by = None
        with self._manual_lock:
            self._manual = {"on": False, "ended": why, "at": time.time()}
        print(f"[rover] manual driving ended: {why}", flush=True)

    def _manual_note(self, text: str) -> None:
        with self._manual_lock:
            self._manual["note"] = text

    def _manual_button(self, code: int) -> None:
        if code == BTN_Y:
            answer = self._tool_set_lights({"level": 0 if self.level else LIGHT_MAX})
            self._manual_note(("lights on" if answer.get("on") else "lights off")
                              if answer.get("ok") else str(answer.get("error")))
        elif code == BTN_B:
            if self._tracking.is_set():
                self._tool_stop_tracking({})
                self._manual_note("face tracking off")
            else:
                answer = self._tool_start_tracking({})
                self._manual_note("face tracking on" if answer.get("ok")
                                  else str(answer.get("error")))
        elif code == BTN_X:
            with self._manual_lock:
                self._manual_voice += 1
                self._manual["note"] = "talk pressed"

    def _manual_camera(self, pan_dps: float, tilt_dps: float, dt: float) -> None:
        """The right stick swings the camera, unless face tracking has it."""
        if (not pan_dps and not tilt_dps) or self._tracking.is_set():
            return
        from aiming import PAN_LIMIT, TILT_LIMITS

        with self._lock:
            pan = min(max(self.pan + pan_dps * dt, -PAN_LIMIT), PAN_LIMIT)
            tilt = min(max(self.tilt + tilt_dps * dt, TILT_LIMITS[0]), TILT_LIMITS[1])
            moved = (round(pan) != round(self.pan) or round(tilt) != round(self.tilt))
            self.pan, self.tilt = pan, tilt
            if moved:
                self._send_gimbal()


def _find_pad() -> Pad | None:
    for path in sorted(glob.glob(JS_GLOB)):
        try:
            return Pad.open(path)
        except OSError:
            continue
    return None
