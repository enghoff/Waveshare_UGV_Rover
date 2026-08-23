"""Drive the Waveshare UGV Rover from a game controller, on the rover's host.

The Linux sibling of drive_gamepad.py. Same controls, same feel, same commands
to the same ESP32 -- what differs is both ends of the pipe: the pad arrives over
Bluetooth as a joystick device instead of through XInput, and the board is
reached down the GPIO UART instead of over WiFi or a USB cable.

    python3 drive_gamepad_pi.py                    # the GPIO UART, /dev/ttyS4
    python3 drive_gamepad_pi.py --serial /dev/ttyUSB0
    python3 drive_gamepad_pi.py --host 192.168.1.22  # over the network instead
    python3 drive_gamepad_pi.py --device /dev/input/js1

The triggers drive and the right stick steers: right trigger forward, left
trigger back, both proportional, and the stick's left/right is the turn. Left
stick aims the pan/tilt camera and it stays where you leave it; click the stick
to recentre. Y switches the white LEDs, and the D-pad sides dim them -- a tap is
one notch, a hold fades. Hold RB for full speed, LB to creep; D-pad up/down moves
the speed cap in steps. View quits, and so does Ctrl-C -- both stop the motors
and douse the lights on the way out.

Why a separate script rather than a flag on the other one: nothing of the pad
layer survives the crossing. XInput hands over a whole struct per poll with the
triggers as bytes and the D-pad as buttons; the joystick API pushes one event
per control change, rests its triggers at -32767, and puts the D-pad on a pair of
axes. The parts that *are* shared -- the expo curve, the skid-steer mix, the PWM
floor, the gimbal rates -- are duplicated below with their values kept in step.
Change a tuning constant in one and change it in the other.

Running on the rover's host buys directness at the cost of reach: the UART is a
wire, so there is no WiFi to drop between pad and motors, and the whole loop is
on one board that rides the rover. The price is that the pad's Bluetooth link is
now the only radio in the chain -- hence a 20 Hz loop that
does nothing expensive, and a heartbeat that stops the base if it falls behind.

The pad is read through the joystick API (`/dev/input/js*`), not evdev, for the
same reason check_gamepad.py is -- eight fixed bytes per event and no python
package to install. Use that script first if the question is whether the pad
works at all; this one assumes it does and gets on with driving.

A pad that drops out mid-drive stops the rover and is then waited for: Bluetooth
reconnects on its own, the device node comes back, and this picks it up without
being restarted. That is the common failure out on the floor -- walking out of
range -- and it should not need a trip back to the terminal.

Safety comes from the firmware's own heartbeat: this sets it to HEARTBEAT_MS on
connect, so if this script dies, the pad vanishes or the UART is unplugged, the
base stops itself within half a second rather than driving on with the last
command. That is the failsafe -- it is not a substitute for the rover's power
switch.
"""

import argparse
import glob
import json
import math
import os
import select
import struct
import sys
import time

# The GPIO UART, which is where this board's ESP32 is wired -- there is no USB
# serial to find. ttyS4 is UART4 on the Banana Pi M4 Zero's 40-pin header;
# ttyAMA0 is the same pins on the Pi 1.
DEFAULT_SERIAL = "/dev/ttyS4"
BAUD = 115200

# Freeing that port took disabling the serial console: a getty on it will fight
# for every byte. See the host notes -- serial-getty@ttyAMA0 masked, and
# console=serial0,115200 out of /boot/firmware/cmdline.txt.
SERIAL_CONSOLE_HINT = ("Is a getty still on it? `systemctl status serial-getty@"
                       + DEFAULT_SERIAL.rsplit('/', 1)[-1] + "`")

# The firmware stops the base if it hears nothing for this long. Its own default
# is 3000 ms, restored on a clean exit -- and by a reboot in any case, since the
# setting lives in RAM.
HEARTBEAT_MS = 500
FIRMWARE_HEARTBEAT_MS = 3000
STOP_REPEATS = 3  # a dropped stop is the one packet that matters

SEND_HZ = 20
# Commands go out only when the PWM pair actually changes, so a held stick is
# quiet and the ESP32's single-threaded server is left alone. This is the floor
# that keeps the heartbeat fed regardless.
KEEPALIVE_S = HEARTBEAT_MS / 3000

# --- kept in step with drive_gamepad.py -------------------------------------
TOP_PWM_DEFAULT = 160
TOP_PWM_STEP = 20
BOOST_PWM = 255  # RB held
CREEP_SCALE = 0.4  # LB held
MIN_PWM = 40  # below this the motors buzz without turning
EXPO = 0.4
FORWARD_SIGN = 1
STEER_SIGN = 1

PAN_LIMIT = 180
TILT_LIMITS = (-30, 90)
PAN_RATE = 90
TILT_RATE = 60
PAN_SIGN = 1
TILT_SIGN = 1

LIGHT_STEP = 32
LIGHT_FULL = 255
LIGHT_HOLD_S = 0.35
LIGHT_FADE_RATE = 128
# ----------------------------------------------------------------------------

# joydev normalises every axis to this, whatever the hardware's own range.
AXIS_MAX = 32767

# XInput's recommended deadzones, reused because joydev reports on the same
# scale. The kernel has already applied the device's own fuzz and flat, but a
# resting stick still wanders further than either accounts for.
DEADZONE_LEFT = 7849
DEADZONE_RIGHT = 8689

# The joystick event: milliseconds, value, kind, control number.
EVENT_FORMAT = "IhBB"
EVENT_SIZE = struct.calcsize(EVENT_FORMAT)
EVENT_BUTTON = 0x01
EVENT_AXIS = 0x02
EVENT_INIT = 0x80  # the synthetic opening burst; state, not a press

# An Xbox Wireless Controller over Bluetooth, through hid-generic and joydev.
# The gaps are real: joydev numbers BTN_* in kernel order, and BTN_C and BTN_Z
# sit between B and X and between Y and LB on a pad that has neither.
AXIS_LEFT_X, AXIS_LEFT_Y = 0, 1
AXIS_RIGHT_X, AXIS_RIGHT_Y = 2, 3
AXIS_LEFT_TRIGGER, AXIS_RIGHT_TRIGGER = 4, 5
AXIS_DPAD_X, AXIS_DPAD_Y = 6, 7

BUTTON_A, BUTTON_B, BUTTON_X, BUTTON_Y = 0, 1, 3, 4
BUTTON_LB, BUTTON_RB = 6, 7
BUTTON_VIEW, BUTTON_MENU, BUTTON_XBOX = 10, 11, 12
BUTTON_LEFT_STICK, BUTTON_RIGHT_STICK = 13, 14

# The triggers rest at -AXIS_MAX and reach +AXIS_MAX, so a released trigger is
# not near zero -- it is at one end of a full-scale axis. Anything below this
# much of the travel counts as not pressed.
TRIGGER_THRESHOLD = 0.05

# A D-pad axis only ever reads -AXIS_MAX, 0 or +AXIS_MAX, so this only has to
# clear zero.
DPAD_THRESHOLD = AXIS_MAX // 2


class Pad:
    """One joystick device, read as a stream of events into a current state.

    The joystick API is edge driven where XInput is level driven: it says what
    changed, not what everything is. So the state is accumulated here and the
    control loop reads it like a poll, which keeps the driving code the same
    shape as the Windows script's.

    The device is opened lazily and reopened after a drop, because a Bluetooth
    pad that goes out of range takes its device node with it and brings it back
    unannounced when it returns.
    """

    def __init__(self, path=None):
        self.wanted = path  # None means "whichever one turns up"
        self.path = None
        self.fd = None
        self.axes = {}
        self.buttons = {}
        self.pressed = set()  # buttons that went down since the last poll
        self._retry_after = 0.0

    def _find(self):
        if self.wanted:
            return self.wanted if os.path.exists(self.wanted) else None
        return min(glob.glob("/dev/input/js*"), default=None)

    def _open(self):
        """Try to attach to the device. True if there is one to read."""
        if self.fd is not None:
            return True
        now = time.monotonic()
        if now < self._retry_after:  # do not stat the filesystem 20 times a second
            return False
        self._retry_after = now + 0.5
        path = self._find()
        if path is None:
            return False
        try:
            self.fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        except PermissionError:
            sys.exit(f"No permission to read {path}. Over SSH the session gets no ACL "
                     f"on it: add yourself to the `input` group and log in again -- "
                     f"sudo usermod -aG input $USER")
        except OSError:
            return False
        self.path = path
        # Whatever the pad was holding before this attached is not a press. The
        # opening burst fills in the state; only later events count as input.
        self.axes.clear()
        self.buttons.clear()
        self.pressed.clear()
        return True

    def _drop(self):
        if self.fd is not None:
            os.close(self.fd)
        self.fd = None
        self.path = None
        self.axes.clear()
        self.buttons.clear()

    def poll(self):
        """Drain every pending event. True if a pad is attached and readable.

        Non-blocking: the control loop sets the pace, and a pad that has nothing
        to say must not hold up the heartbeat.
        """
        self.pressed.clear()
        if not self._open():
            return False
        while True:
            if not select.select([self.fd], [], [], 0)[0]:
                return True
            try:
                data = os.read(self.fd, EVENT_SIZE)
            except OSError:  # the link went down between select and read
                self._drop()
                return False
            if len(data) < EVENT_SIZE:  # end of file: the device is gone
                self._drop()
                return False

            _, value, kind, number = struct.unpack(EVENT_FORMAT, data)
            synthetic = bool(kind & EVENT_INIT)
            kind &= ~EVENT_INIT
            if kind == EVENT_AXIS:
                self.axes[number] = value
            elif kind == EVENT_BUTTON:
                was = self.buttons.get(number, 0)
                self.buttons[number] = value
                if value and not was and not synthetic:
                    self.pressed.add(number)

    def axis_raw(self, number):
        return self.axes.get(number, 0)

    def held(self, number):
        return bool(self.buttons.get(number))

    def just_pressed(self, number):
        return number in self.pressed


def axis(value, deadzone):
    """One raw stick axis -> -1..1, with a deadzone taken out.

    For the steering stick, where only the one axis is read: the other is free
    to flop about without a round deadzone reading its magnitude as steering.

    What is left of the travel is rescaled to a full 0..1, without which the
    control would jump to a quarter output the instant it left the deadzone.
    """
    if abs(value) <= deadzone:
        return 0.0
    span = (abs(value) - deadzone) / (AXIS_MAX - deadzone)
    return math.copysign(min(span, 1.0), value)


def trigger(value):
    """One raw trigger -> 0..1, from an axis that rests at -AXIS_MAX."""
    span = (value + AXIS_MAX) / (2 * AXIS_MAX)
    return 0.0 if span <= TRIGGER_THRESHOLD else min(span, 1.0)


def stick(x, y, deadzone):
    """Raw stick pair -> (x, y) in -1..1, with a round deadzone taken out.

    Radial, not per axis, because the camera stick is worked in both at once and
    a square deadzone lets a stick pushed straight up register a little sideways
    as well -- here, a camera that drifts round as it tilts.
    """
    magnitude = math.hypot(x, y)
    if magnitude <= deadzone:
        return 0.0, 0.0
    scale = min((magnitude - deadzone) / (AXIS_MAX - deadzone), 1.0) / magnitude
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
    """

    def __init__(self):
        self.level = 0.0
        self.held_since = None

    @property
    def value(self):
        return int(round(self.level))

    def toggle(self):
        self.level = 0.0 if self.level else float(LIGHT_FULL)

    def dim(self, direction, now, dt):
        """One tick of the D-pad sides. direction is -1, 0 or +1."""
        if direction == 0:
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
    """Where the camera is pointed, in degrees, integrated from the left stick.

    Nothing can be read back, so the angles here are a model, kept true by
    centring the camera once at startup and being the only thing commanding it
    thereafter. The stick is spring loaded, so its deflection is a speed rather
    than an angle -- otherwise the camera would snap back on every release.
    """

    def __init__(self):
        self.pan = 0.0
        self.tilt = 0.0

    def aim(self, x, y, dt, recentre):
        """Move the target by one tick of stick. True if it moved a whole degree."""
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
        return {"T": 133, "X": round(self.pan), "Y": round(self.tilt), "SPD": 0, "ACC": 0}


class SerialLink:
    """JSON commands down the GPIO UART to the ESP32."""

    def __init__(self, port):
        import serial

        self.port = port
        self.link = serial.Serial(port, BAUD, timeout=0.1)

    def describe(self):
        return f"{self.port} at {BAUD}"

    def send(self, command):
        try:
            self.link.write(json.dumps(command, separators=(",", ":")).encode() + b"\n")
            # The board streams T:1001 telemetry continuously and nothing here
            # reads it; left alone it would fill the buffer within seconds.
            self.link.reset_input_buffer()
            return True
        except Exception:
            return False

    def close(self):
        try:
            self.link.close()
        except Exception:
            pass


class HttpLink:
    """JSON commands over the ESP32's own `/js` endpoint, for a rover on WiFi.

    One connection kept open rather than a handshake 20 times a second, rebuilt
    only when it breaks -- which also makes a lost link visible as an error
    rather than a silent stall.
    """

    def __init__(self, host, timeout=0.5):
        import http.client
        from urllib.parse import quote

        self._client = http.client
        self._quote = quote
        self.host = host
        self.timeout = timeout
        self.connection = None

    def describe(self):
        return f"http://{self.host}/js"

    def send(self, command):
        path = "/js?json=" + self._quote(
            json.dumps(command, separators=(",", ":")), safe="")
        for attempt in (1, 2):  # a stale keep-alive costs one retry, not a command
            if self.connection is None:
                self.connection = self._client.HTTPConnection(self.host, timeout=self.timeout)
            try:
                self.connection.request("GET", path)
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


def open_link(args):
    if args.host:
        return HttpLink(args.host)
    try:
        return SerialLink(args.serial)
    except Exception as error:
        sys.exit(f"Cannot open {args.serial}: {error}\n{SERIAL_CONSOLE_HINT}")


def main():
    parser = argparse.ArgumentParser(
        description="Drive the UGV Rover with a Bluetooth game controller, from the rover's host.")
    parser.add_argument("--serial", default=DEFAULT_SERIAL, metavar="PORT",
                        help=f"the ESP32's serial port (default {DEFAULT_SERIAL})")
    parser.add_argument("--host", default=None, metavar="ADDRESS",
                        help="talk to the board over WiFi at this address instead")
    parser.add_argument("--device", default=None, metavar="PATH",
                        help="the pad to read; by default the first /dev/input/js*")
    parser.add_argument("--top", type=int, default=TOP_PWM_DEFAULT, help="initial speed cap, 0-255")
    parser.add_argument("--no-gimbal", dest="gimbal", action="store_false",
                        help="leave the pan/tilt camera alone, for a rover without one fitted")
    args = parser.parse_args()

    pad = Pad(args.device)
    if not pad.poll():
        sys.exit("No controller found. Wake it with its Xbox button, and check it is "
                 "still paired: bluetoothctl info")

    link = open_link(args)
    top_pwm = min(max(args.top, MIN_PWM + 1), 255)
    print(f"pad on {pad.path}, driving over {link.describe()}; View or Ctrl-C to stop")

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
    failures = 0
    lights = Lights()
    lights_sent = 0
    dpad_up_before = dpad_down_before = False

    try:
        while True:
            time.sleep(1 / SEND_HZ)
            now = time.monotonic()
            # Measured, not assumed: the camera's speed should not depend on how
            # promptly a Pi 1 got round to running the loop.
            dt, last_tick = now - last_tick, now

            if not pad.poll():
                left = right = 0
                status = "no pad"
            else:
                if pad.just_pressed(BUTTON_VIEW):
                    break
                if pad.just_pressed(BUTTON_Y):
                    lights.toggle()
                dpad_x = pad.axis_raw(AXIS_DPAD_X)
                dpad_y = pad.axis_raw(AXIS_DPAD_Y)
                # The D-pad is an axis pair here, not four buttons, so there are
                # no button edges to read: up and down are one axis at its two
                # ends -- negative is up, joydev's Y being positive downwards --
                # and the edge has to be found by comparing against last tick.
                dpad_up = dpad_y < -DPAD_THRESHOLD
                dpad_down = dpad_y > DPAD_THRESHOLD
                if dpad_up and not dpad_up_before:
                    top_pwm = min(top_pwm + TOP_PWM_STEP, 255)
                elif dpad_down and not dpad_down_before:
                    top_pwm = max(top_pwm - TOP_PWM_STEP, MIN_PWM + 1)
                dpad_up_before, dpad_down_before = dpad_up, dpad_down

                # The sides are read held rather than on the edge, since holding
                # one is a fade rather than a second notch.
                lights.dim((dpad_x > DPAD_THRESHOLD) - (dpad_x < -DPAD_THRESHOLD), now, dt)
                if lights.value != lights_sent:
                    link.send(lights.command())
                    lights_sent = lights.value

                # Triggers drive, stick steers. The two triggers subtract, so
                # both at once is a stop rather than an argument.
                throttle = FORWARD_SIGN * expo(
                    trigger(pad.axis_raw(AXIS_RIGHT_TRIGGER))
                    - trigger(pad.axis_raw(AXIS_LEFT_TRIGGER)))
                steer = STEER_SIGN * expo(
                    axis(pad.axis_raw(AXIS_RIGHT_X), DEADZONE_RIGHT))
                cap = BOOST_PWM if pad.held(BUTTON_RB) else top_pwm
                if pad.held(BUTTON_LB):
                    cap = max(int(cap * CREEP_SCALE), MIN_PWM + 1)
                left, right = (to_pwm(v, cap) for v in mix(throttle, steer))
                status = f"cap {cap:3d}"

                # The camera goes on its own command, and deliberately not on
                # the keepalive: T:133 does not touch the firmware's heartbeat
                # timer, so aiming alone must not be mistaken for driving.
                if gimbal is not None:
                    aim_x, aim_y = stick(pad.axis_raw(AXIS_LEFT_X),
                                         pad.axis_raw(AXIS_LEFT_Y), DEADZONE_LEFT)
                    if gimbal.aim(aim_x, -aim_y, dt,
                                  pad.just_pressed(BUTTON_LEFT_STICK)):
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
        link.send({"T": 132, "IO4": 0, "IO5": 0})
        link.send({"T": 136, "cmd": FIRMWARE_HEARTBEAT_MS})
        link.close()
        print("stopped")


if __name__ == "__main__":
    main()
