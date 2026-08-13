"""Drive the Waveshare UGV Rover from an Xbox controller, over WiFi or USB.

Talks to the ESP32 on the rover's General Driver board -- the only thing on the
robot wired to the motors -- by sending it the same JSON commands its own web UI
sends. No Raspberry Pi, no ROS, nothing else on the rover need be running.

    python drive_xbox.py                 # over WiFi, to the rover's own AP
    python drive_xbox.py --host 192.168.1.9   # ...or wherever it joined your LAN
    python drive_xbox.py --serial        # over USB, auto-detecting the port
    python drive_xbox.py --serial COM7

The triggers drive and the right stick steers: right trigger forward, left trigger
back, both proportional, and the stick's left/right is the turn. Left stick aims
the pan/tilt camera and it stays where you leave it; click the stick to recentre.
Y switches the white LEDs, and the D-pad sides dim them -- a tap is one notch, a
hold fades. Hold RB for full speed, LB to creep; D-pad up/down moves the speed cap
in steps. Back quits, and so does Ctrl-C -- both stop the motors and douse the
lights on the way out.

Throttle sits on the triggers rather than the left stick because they are the one
control on the pad with travel to spare in a single direction: a stick shares its
range between forward and back and springs to the middle of it, while a trigger
gives all of its travel to going forwards and rests at zero. It also puts the two
on separate controls, so a turn can no longer pull the throttle round with it.

The camera is rate-controlled, not position-controlled: the stick is spring
loaded, so treating its deflection as an angle would snap the camera back to
centre every time you let go. Deflection is a speed instead, and the angle is
integrated from it. The camera is centred once at startup, because the firmware
offers no way to read back where the servos are pointing.

Unlike the rest of the suite this opens no window: it reads the pad through
XInput, which does not need focus, so you can watch the rover instead of the
screen. It is therefore Windows-only, and the status line is the terminal's.

The command is CMD_PWM_INPUT (`{"T":11,"L":..,"R":..}`), open-loop PWM in
+-255, because this rover has no wheel encoders -- `{"T":1,...}` and the ROS
`{"T":13,...}` want encoder feedback to close the loop against. Two consequences
worth knowing: equal PWM on both sides is not equal speed, so it will drift on a
straight, and PWM below roughly MIN_PWM only makes the motors buzz.

Safety comes from the firmware's own heartbeat: this sets it to HEARTBEAT_MS on
connect, so if this script dies, the WiFi drops or the USB is pulled, the base
stops itself within half a second rather than driving on with the last command.
That is the failsafe -- it is not a substitute for the rover's power switch.
"""

import argparse
import ctypes
import json
import math
import sys
import time

DEFAULT_HOST = "192.168.4.1"  # the ESP32's own AP: SSID "UGV", password "12345678"
BAUD = 115200

# The firmware stops the base if it hears nothing for this long. Its own default
# is 3000 ms, restored on a clean exit -- and by a reboot in any case, since the
# setting lives in RAM.
HEARTBEAT_MS = 500
FIRMWARE_HEARTBEAT_MS = 3000
STOP_REPEATS = 3  # a dropped stop is the one packet that matters

SEND_HZ = 20
# Commands go out only when the PWM pair actually changes, so a held stick is
# quiet and the ESP32's single-threaded web server is left alone. This is the
# floor that keeps the heartbeat fed regardless.
KEEPALIVE_S = HEARTBEAT_MS / 3000

TOP_PWM_DEFAULT = 160
TOP_PWM_STEP = 20
BOOST_PWM = 255  # RB held
CREEP_SCALE = 0.4  # LB held
# Below this the motors buzz without turning. Stiction, not a firmware limit, so
# it is a starting point to tune by feel rather than a measured constant -- the
# firmware's own THRESHOLD_PWM of 23 applies only to its encoder PID path.
MIN_PWM = 40

# How much of the stick's travel is spent on the slow end. The sticks are
# analogue and the whole point is proportional control, but a straight linear map
# puts useful crawling speed in the first few millimetres of travel. This is the
# usual RC expo -- a blend of cube and linear, still smooth and still reaching
# full output at full deflection, just finer around centre. 0 turns it off.
EXPO = 0.4

# Flip these if the rover disagrees with the stick. Motor polarity depends on how
# the leads went on, and the firmware has its own SET_MOTOR_DIR as well.
FORWARD_SIGN = 1
STEER_SIGN = 1

# The pan/tilt is two ST3215 bus servos. These limits are the firmware's own, out
# of gimbalCtrlSimple, which clamps to them regardless of what it is sent.
PAN_LIMIT = 180
TILT_LIMITS = (-30, 90)
# Degrees per second at full stick. Pan faster than tilt: there is four times as
# much of it to cross, and much less to hit.
PAN_RATE = 90
TILT_RATE = 60
PAN_SIGN = 1  # flip if the camera pans away from the stick
TILT_SIGN = 1

# The white LEDs, on GPIO 4 and 5, are PWM rather than on/off -- so the pad dims
# them as well as switching them. Both are driven together as one headlight.
LIGHT_STEP = 32  # one tap of the D-pad
LIGHT_FULL = 255
# Held, the D-pad fades instead of stepping. The delay is what keeps a tap a tap;
# the rate crosses the whole range in about two seconds, slow enough to stop where
# you meant to and quick enough not to be a chore.
LIGHT_HOLD_S = 0.35
LIGHT_FADE_RATE = 128

# XInput's own recommended deadzones, in raw stick units -- named for the stick
# rather than the job, since they are the hardware's numbers and the smaller right
# stick genuinely needs the larger one. The camera is on the left, steering right.
DEADZONE_LEFT = 7849
DEADZONE_RIGHT = 8689
STICK_MAX = 32767

# The triggers are a byte each, and rest at zero -- but not reliably at zero, so
# XInput names a threshold below which one is not pressed at all.
TRIGGER_THRESHOLD = 30
TRIGGER_MAX = 255

BUTTON_DPAD_UP = 0x0001
BUTTON_DPAD_DOWN = 0x0002
BUTTON_DPAD_LEFT = 0x0004
BUTTON_DPAD_RIGHT = 0x0008
BUTTON_BACK = 0x0020
BUTTON_LEFT_STICK = 0x0040
BUTTON_LB = 0x0100
BUTTON_RB = 0x0200
BUTTON_Y = 0x8000

ERROR_DEVICE_NOT_CONNECTED = 1167


class XInputGamepad(ctypes.Structure):
    _fields_ = [
        ("wButtons", ctypes.c_ushort),
        ("bLeftTrigger", ctypes.c_ubyte),
        ("bRightTrigger", ctypes.c_ubyte),
        ("sThumbLX", ctypes.c_short),
        ("sThumbLY", ctypes.c_short),
        ("sThumbRX", ctypes.c_short),
        ("sThumbRY", ctypes.c_short),
    ]


class XInputState(ctypes.Structure):
    _fields_ = [("dwPacketNumber", ctypes.c_ulong), ("Gamepad", XInputGamepad)]


class Pad:
    """One XInput controller, polled. Nothing to install: an Xbox pad is XInput.

    xinput1_4 ships with Windows 8 and later; the older names are there for a
    machine carrying only the DirectX redistributable's copy.
    """

    def __init__(self, slot=None):
        self.dll = None
        for name in ("xinput1_4", "xinput1_3", "xinput9_1_0"):
            try:
                self.dll = ctypes.WinDLL(name)
                break
            except OSError:
                continue
        if self.dll is None:
            sys.exit("No XInput DLL found -- this script is Windows-only.")
        self.slot = slot
        self.state = XInputState()

    def find(self):
        """The lowest slot with a pad in it, or None."""
        for slot in range(4):
            if self.dll.XInputGetState(slot, ctypes.byref(self.state)) == 0:
                return slot
        return None

    def poll(self):
        """The pad's current state, or None if it is not there.

        A pad that vanishes mid-drive costs one poll to notice; the caller sends
        a stop and keeps looking for it, so a knocked-out USB plug or a flat
        battery halts the rover rather than freezing the last command.
        """
        if self.slot is None:
            self.slot = self.find()
            if self.slot is None:
                return None
        if self.dll.XInputGetState(self.slot, ctypes.byref(self.state)) != 0:
            self.slot = None
            return None
        return self.state.Gamepad


def axis(value, deadzone):
    """One raw stick axis -> -1..1, with a deadzone taken out.

    For the steering stick, where only the one axis is read: the other is free to
    flop about without a round deadzone reading its magnitude as steering.

    What is left of the travel is rescaled to a full 0..1, without which the
    control would jump to a quarter output the instant it left the deadzone.
    """
    if abs(value) <= deadzone:
        return 0.0
    span = (abs(value) - deadzone) / (STICK_MAX - deadzone)
    return math.copysign(min(span, 1.0), value)  # the raw axis reaches -32768


def trigger(value):
    """One raw trigger -> 0..1, past the point where it counts as pressed."""
    if value <= TRIGGER_THRESHOLD:
        return 0.0
    return (value - TRIGGER_THRESHOLD) / (TRIGGER_MAX - TRIGGER_THRESHOLD)


def stick(x, y, deadzone):
    """Raw stick pair -> (x, y) in -1..1, with a round deadzone taken out.

    Radial, not per axis, because the camera stick is worked in both at once and
    a square deadzone lets a stick pushed straight up register a little sideways
    as well -- here, a camera that drifts round as it tilts. Taking the magnitude
    first means the dead region is a disc, so pure tilt stays pure tilt.

    What is left of the travel is rescaled to a full 0..1, as above.
    """
    magnitude = math.hypot(x, y)
    if magnitude <= deadzone:
        return 0.0, 0.0
    # Diagonals reach ~1.41x full scale, so the rescale is clamped rather than
    # trusted, and the direction is preserved by scaling both axes together.
    scale = min((magnitude - deadzone) / (STICK_MAX - deadzone), 1.0) / magnitude
    return x * scale, y * scale


def expo(value):
    """Soften an analogue axis around centre, without losing its full range."""
    return EXPO * value ** 3 + (1 - EXPO) * value


def mix(throttle, steer):
    """Throttle and steer in -1..1 -> left and right track in -1..1.

    Skid steer: turning is a difference between the sides. The pair is scaled
    down together when it would clip, so a hard turn under full throttle keeps
    its shape instead of flattening into a straight line.
    """
    left = throttle + steer
    right = throttle - steer
    peak = max(abs(left), abs(right))
    if peak > 1.0:
        left /= peak
        right /= peak
    return left, right


def to_pwm(value, top):
    """-1..1 -> a motor PWM, skipping the range where the motors only buzz."""
    if abs(value) < 1e-3:
        return 0
    magnitude = MIN_PWM + abs(value) * (top - MIN_PWM)
    return int(round(magnitude if value > 0 else -magnitude))


class Lights:
    """The headlights' brightness, and how the pad moves it.

    Assumed dark at startup rather than read back -- like the gimbal, the LEDs
    have no feedback path. The exit turns them off, which makes the assumption
    true for the next run as long as this was the last thing to touch them.

    A tap of the D-pad is one notch, so the useful settings stay repeatable;
    holding fades instead, so crossing the whole range is not eight taps. The
    level is kept as a float and only rounded on the way to the board, or a fade
    of a few units per tick would never accumulate into a whole one.
    """

    def __init__(self):
        self.level = 0.0
        self.held_since = None

    @property
    def value(self):
        return int(round(self.level))

    def toggle(self):
        # Off goes back to full, remembering nothing: Y stays a headlight switch
        # rather than a way to get stuck at one notch of brightness.
        self.level = 0.0 if self.level else float(LIGHT_FULL)

    def dim(self, direction, now, dt):
        """One tick of the D-pad sides. direction is -1, 0 or +1."""
        if direction == 0:  # released, or both sides at once
            self.held_since = None
            return
        if self.held_since is None:
            self.held_since = now
            self.level += direction * LIGHT_STEP
        elif now - self.held_since > LIGHT_HOLD_S:
            self.level += direction * LIGHT_FADE_RATE * dt
        self.level = min(max(self.level, 0.0), float(LIGHT_FULL))

    def command(self):
        return {"T": 132, "IO4": self.value, "IO5": self.value}


class Gimbal:
    """Where the camera is pointed, in degrees, integrated from the right stick.

    Nothing can be read back: the firmware has a getGimbalFeedback() but no JSON
    command reaches it, so there is no asking the servos where they are. The
    angles here are therefore a model, kept true by centring the camera once at
    startup and being the only thing commanding it thereafter.
    """

    def __init__(self):
        self.pan = 0.0
        self.tilt = 0.0

    def aim(self, x, y, dt, recentre):
        """Move the target by one tick of stick. True if it moved a whole degree.

        Sub-degree changes are not worth a command: the servos take integers,
        and the bus is shared with everything else the board is doing.
        """
        before = self.command()
        if recentre:
            self.pan = self.tilt = 0.0
        else:
            self.pan += PAN_SIGN * expo(x) * PAN_RATE * dt
            self.tilt += TILT_SIGN * expo(y) * TILT_RATE * dt
            self.pan = min(max(self.pan, -PAN_LIMIT), PAN_LIMIT)
            self.tilt = min(max(self.tilt, TILT_LIMITS[0]), TILT_LIMITS[1])
        return self.command() != before

    def command(self):
        # SPD 0 is the servo's own maximum. The rate limiting has already
        # happened here, in how fast the target moves, so the servo's job is to
        # chase that target as promptly as it can rather than add a second limit.
        return {"T": 133, "X": round(self.pan), "Y": round(self.tilt), "SPD": 0, "ACC": 0}


class HttpLink:
    """JSON commands over the ESP32's own `/js` endpoint.

    A fresh connection per command would mean a TCP handshake 20 times a second
    at an ESP32; this keeps one open and rebuilds it only when it breaks, which
    also makes a lost link visible as an error rather than a silent stall.
    """

    def __init__(self, host, timeout=0.5):
        import http.client

        self._client = http.client
        self.host = host
        self.timeout = timeout
        self.connection = None

    def describe(self):
        return f"http://{self.host}/js"

    def send(self, command):
        from urllib.parse import quote

        payload = quote(json.dumps(command, separators=(",", ":")), safe="")
        for attempt in (1, 2):  # a stale keep-alive costs one retry, not a command
            if self.connection is None:
                self.connection = self._client.HTTPConnection(self.host, timeout=self.timeout)
            try:
                self.connection.request("GET", f"/js?json={payload}")
                self.connection.getresponse().read()
                return True
            except Exception:
                self.close()
                if attempt == 2:
                    return False
        return False

    def close(self):
        if self.connection is not None:
            try:
                self.connection.close()
            except Exception:
                pass
            self.connection = None


class SerialLink:
    """JSON commands over the ESP32's Type-C port -- the one *not* labelled LIDAR."""

    def __init__(self, port):
        import serial

        self.port = port
        self.link = serial.Serial(port, BAUD, timeout=0.1)

    def describe(self):
        return f"{self.port} at {BAUD}"

    def send(self, command):
        try:
            self.link.write(json.dumps(command, separators=(",", ":")).encode() + b"\n")
            self.link.reset_input_buffer()  # the board chatters; nothing here reads it
            return True
        except Exception:
            return False

    def close(self):
        try:
            self.link.close()
        except Exception:
            pass


def find_serial_port():
    """Find the ESP32's port by asking it something only it will answer.

    Both Type-C ports on this board revision sit behind the same hub and can
    enumerate as the same CH343 VID/PID, so unlike lidar_view.py this cannot go
    by USB identity -- the lidar port would match just as well. Instead each
    candidate is asked for base feedback: the ESP32 replies with a JSON line,
    while the lidar port only ever streams its own binary packets and the wrong
    port stays silent.

    Only USB ports are probed. Windows offers a "Standard Serial over Bluetooth
    link" COM port for every paired device that has ever advertised SPP, and
    merely opening one blocks for as long as Windows spends trying to raise the
    radio link -- which on a device that is not there is effectively forever.
    Those report no VID, which is what excludes them here.
    """
    import serial
    from serial.tools import list_ports

    for port in list_ports.comports():
        if port.vid is None:
            continue
        try:
            with serial.Serial(port.device, BAUD, timeout=0.1) as link:
                link.reset_input_buffer()
                link.write(b'{"T":130}\n')
                deadline = time.monotonic() + 0.5
                while time.monotonic() < deadline:
                    line = link.readline()
                    if line.startswith(b"{"):
                        return port.device
        except Exception:
            continue
    return None


def open_link(args):
    if args.serial is None:
        return HttpLink(args.host)
    port = args.serial if args.serial != "auto" else find_serial_port()
    if port is None:
        sys.exit("No driver board found on any serial port. Name it, e.g. --serial COM7")
    try:
        return SerialLink(port)
    except Exception as error:  # a wrong port name, or one already in use
        sys.exit(f"Cannot open {port}: {error}")


def main():
    parser = argparse.ArgumentParser(description="Drive the UGV Rover with an Xbox controller.")
    parser.add_argument("--host", default=DEFAULT_HOST, help="the ESP32's address over WiFi")
    parser.add_argument(
        "--serial", nargs="?", const="auto", default=None, metavar="PORT",
        help="drive over USB instead; bare, or with a port such as COM7",
    )
    parser.add_argument("--top", type=int, default=TOP_PWM_DEFAULT, help="initial speed cap, 0-255")
    parser.add_argument(
        "--no-gimbal", dest="gimbal", action="store_false",
        help="leave the pan/tilt camera alone, for a rover without one fitted",
    )
    args = parser.parse_args()

    pad = Pad()
    if pad.find() is None:
        sys.exit("No controller found. Plug in the Xbox pad, or wake it with its Guide button.")

    link = open_link(args)
    top_pwm = min(max(args.top, MIN_PWM + 1), 255)
    print(f"driving over {link.describe()}; Back or Ctrl-C to stop")

    # Do this before anything moves: it is what stops the rover if this process
    # is not here to send the next command.
    if not link.send({"T": 136, "cmd": HEARTBEAT_MS}):
        link.close()
        sys.exit(f"No answer from the driver board on {link.describe()}. Is it powered?")

    # The one command that moves something before the stick is touched: it puts
    # the camera where this thinks it is, since it cannot ask.
    gimbal = Gimbal() if args.gimbal else None
    if gimbal is not None:
        link.send(gimbal.command())

    sent = None
    last_send = 0.0
    last_tick = time.monotonic()
    buttons_before = 0
    failures = 0
    lights = Lights()
    lights_sent = 0

    try:
        while True:
            time.sleep(1 / SEND_HZ)
            now = time.monotonic()
            dt, last_tick = now - last_tick, now  # measured, not assumed: the
            # camera's speed should not depend on how promptly the loop ran.
            state = pad.poll()

            if state is None:
                left = right = 0
                status = "no pad"
            else:
                if state.wButtons & BUTTON_BACK:
                    break
                pressed = state.wButtons & ~buttons_before  # edges, not held keys
                buttons_before = state.wButtons
                if pressed & BUTTON_DPAD_UP:
                    top_pwm = min(top_pwm + TOP_PWM_STEP, 255)
                elif pressed & BUTTON_DPAD_DOWN:
                    top_pwm = max(top_pwm - TOP_PWM_STEP, MIN_PWM + 1)

                # Y is the switch, the D-pad sides are the dimmer. The dimmer is
                # read held rather than on the edge, since holding is a fade.
                if pressed & BUTTON_Y:
                    lights.toggle()
                lights.dim(
                    bool(state.wButtons & BUTTON_DPAD_RIGHT)
                    - bool(state.wButtons & BUTTON_DPAD_LEFT), now, dt)
                if lights.value != lights_sent:
                    link.send(lights.command())
                    lights_sent = lights.value

                # Triggers drive, stick steers. The two triggers subtract, so
                # both at once is a stop rather than an argument.
                throttle = FORWARD_SIGN * expo(
                    trigger(state.bRightTrigger) - trigger(state.bLeftTrigger))
                steer = STEER_SIGN * expo(axis(state.sThumbRX, DEADZONE_RIGHT))
                cap = BOOST_PWM if state.wButtons & BUTTON_RB else top_pwm
                if state.wButtons & BUTTON_LB:
                    cap = max(int(cap * CREEP_SCALE), MIN_PWM + 1)
                left, right = (to_pwm(v, cap) for v in mix(throttle, steer))
                status = f"cap {cap:3d}"

                # The camera goes on its own command, and deliberately not on
                # the keepalive: T:133 does not touch the firmware's heartbeat
                # timer, so aiming alone must not be mistaken for driving.
                if gimbal is not None:
                    aim_x, aim_y = stick(state.sThumbLX, state.sThumbLY, DEADZONE_LEFT)
                    if gimbal.aim(aim_x, aim_y, dt, pressed & BUTTON_LEFT_STICK):
                        link.send(gimbal.command())

            # Quiet when nothing changes, but never quiet for long enough to trip
            # the heartbeat the rover is relying on.
            if (left, right) != sent or now - last_send > KEEPALIVE_S:
                failures = 0 if link.send({"T": 11, "L": left, "R": right}) else failures + 1
                sent = (left, right)
                last_send = now

            health = "ok" if not failures else f"{failures} lost"
            aim = "" if gimbal is None else f"pan {gimbal.pan:+4.0f}  tilt {gimbal.tilt:+3.0f}   "
            print(f"\r L {left:+4d}   R {right:+4d}   {status}   {aim}"
                  f"led {lights.value:3d}   link {health}   ", end="", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        for _ in range(STOP_REPEATS):
            link.send({"T": 11, "L": 0, "R": 0})
        # Headlights off too: nothing is left drawing from the pack once the pad
        # is put down, and the next run's assumed-dark starting state holds.
        link.send({"T": 132, "IO4": 0, "IO5": 0})
        link.send({"T": 136, "cmd": FIRMWARE_HEARTBEAT_MS})
        link.close()
        print("stopped")


if __name__ == "__main__":
    main()
