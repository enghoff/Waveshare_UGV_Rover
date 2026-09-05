"""Platform-independent gamepad mixing, lights and gimbal controls."""

EXPO = 0.4

PAN_LIMIT = 180

TILT_LIMITS = (-30, 90)

PAN_RATE = 90

TILT_RATE = 60

PAN_SIGN = 1  # flip if the camera pans away from the stick

TILT_SIGN = 1

LIGHT_STEP = 32  # one tap of the D-pad

LIGHT_FULL = 255

LIGHT_HOLD_S = 0.35

LIGHT_FADE_RATE = 128


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
