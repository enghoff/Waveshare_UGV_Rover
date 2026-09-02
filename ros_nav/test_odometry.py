"""Odometry and the IMU: integrating what the board reports.

A tick difference taken across a counter reset and a gyro bias that never
settles are both silent faults -- the rover simply believes it is somewhere it
is not -- so they are checked here as arithmetic on recorded samples.
"""
import math

from test_harness import check, section


# --- odometry -----------------------------------------------------------------
def integrate(samples, gyro_lsb_per_dps, ticks_per_metre):
    """base_node.BaseNode.integrate's arithmetic, standing alone.

    Same midpoint-heading rule and same treatment of `breaks`, so a change to
    either here is a change that has to be made in both places on purpose.
    """
    x = y = yaw = 0.0
    last_gz = last_ticks = last_breaks = None
    for gz, ticks, breaks in samples:
        broken = last_breaks is not None and breaks != last_breaks
        last_breaks = breaks
        d_yaw = 0.0
        if last_gz is not None and not broken:
            d_yaw = math.radians((gz - last_gz) / gyro_lsb_per_dps)
        last_gz = gz
        d_s = 0.0
        if last_ticks is not None and not broken:
            d_s = (ticks - last_ticks) / ticks_per_metre
        last_ticks = ticks
        heading = yaw + d_yaw / 2.0
        x += d_s * math.cos(heading)
        y += d_s * math.sin(heading)
        yaw = (yaw + d_yaw + math.pi) % (2 * math.pi) - math.pi
    return x, y, yaw


def idle_sends(commanded, seconds_since_cmd, ever_commanded=True):
    """base_node.BaseNode.drive's decision about whether to say anything at all.

    Its own arithmetic, standing alone, because the rule is easy to get backwards
    in the safe-looking direction and the consequence is a rover nobody else can
    drive.
    """
    live = ever_commanded and seconds_since_cmd <= 0.5
    if not live:
        return commanded not in (None, (0, 0))
    return True


def test_idle_behaviour():
    section("what the base node does when nobody is navigating")
    check("having never been commanded, it says nothing to the board",
          idle_sends(None, 999.0, ever_commanded=False), False)
    check("a live command is always sent", idle_sends((0, 0), 0.1), True)
    check("when a command goes stale, one stop is sent",
          idle_sends((80, 80), 1.0), True)
    check("...and then it goes quiet rather than repeating the stop",
          idle_sends((0, 0), 1.0), False)
    # This is the one that matters. If it were True, ROS would be commanding
    # zero several times a second for ever, and a person driving the rover with
    # the game pad would find the board braking under them with nothing in any
    # log to explain it.
    check("so a game pad can drive the rover while ROS is only mapping",
          idle_sends((0, 0), 30.0), False)


def debias_run(samples, gain=0.001, settle=1.0, still_ticks=0.5):
    """base_node.BaseNode.debias, standing alone.

    Each sample is (d_yaw, dt, ticks, commanded). Returns the corrected total and
    the bias it settled on.
    """
    bias = None
    still_for = 0.0
    last_ticks = None
    total = 0.0
    for d_yaw, dt, ticks, commanded in samples:
        rate = d_yaw / dt
        moving = bool(commanded)
        if ticks is not None and last_ticks is not None:
            moving = moving or abs(ticks - last_ticks) > still_ticks
        if not moving:
            still_for += dt
            if still_for > settle:
                bias = rate if bias is None else bias + gain * (rate - bias)
        else:
            still_for = 0.0
        total += d_yaw if bias is None else d_yaw - bias * dt
        last_ticks = ticks
    return total, bias


def test_gyro_bias():
    section("the gyro's zero-offset")
    dt = 1.0 / 18.0
    drift = math.radians(0.46)          # measured on this rover, standing still

    # Standing still for two minutes with a 0.46 deg/s offset. Uncorrected that is
    # 55 degrees of rotation that never happened.
    still = [(drift * dt, dt, 100.0, False) for _ in range(18 * 120)]
    raw = sum(d for d, _, _, _ in still)
    check("uncorrected, a still rover invents %.0f degrees in two minutes"
          % math.degrees(raw), math.degrees(raw) > 40, True)
    total, bias = debias_run(still)
    check("the offset is found", bias is not None, True)
    check("...to within a twentieth of a degree per second",
          math.degrees(bias), math.degrees(drift), tolerance=0.05)
    check("...and most of the invented rotation is removed",
          abs(math.degrees(total)) < abs(math.degrees(raw)) * 0.25, True)

    # A real turn must survive. The rover is commanded and the wheels are moving,
    # so nothing is learned during it and the rotation passes through.
    warmup = [(drift * dt, dt, 100.0, False) for _ in range(18 * 60)]
    turning = [(math.radians(25.0) * dt, dt, 100.0 + i, True) for i in range(18 * 4)]
    # Both totals corrected, and differenced. Subtracting the *raw* warmup from a
    # corrected total would charge the turn with the bias the warmup removed.
    settled, _ = debias_run(warmup)
    total, _ = debias_run(warmup + turning)
    turned = math.degrees(total - settled)
    check("a commanded 25 deg/s turn for 4 s still reads about 100 degrees",
          turned, 100.0, tolerance=6.0)

    # Wheels turning with nothing commanded -- somebody pushing the rover, or a
    # game pad driving it -- must not be mistaken for stillness.
    pushed = [(math.radians(20.0) * dt, dt, 100.0 + i * 3, False)
              for i in range(18 * 30)]
    _, bias = debias_run(pushed)
    check("a pushed rover is not treated as a still one", bias, None)

    # And the settle window keeps the coast out of the estimate.
    coasting = [(math.radians(15.0) * dt, dt, 100.0, False) for _ in range(9)]
    _, bias = debias_run(coasting)
    check("half a second of coasting teaches it nothing", bias, None)


def test_odometry():
    section("board counters -> pose")
    lsb, tpm = 15.0, 100.0

    straight = [(0.0, 0.0, 0), (0.0, 100.0, 0), (0.0, 200.0, 0)]
    x, y, yaw = integrate(straight, lsb, tpm)
    check("200 ticks at 100/m is 2 m straight ahead", x, 2.0, tolerance=1e-9)
    check("...with no sideways drift", y, 0.0, tolerance=1e-9)
    check("...and no heading change", yaw, 0.0, tolerance=1e-9)

    # 15 LSB per degree-per-second, so 1350 LSB-seconds is 90 degrees.
    turning = [(0.0, 0.0, 0), (1350.0, 0.0, 0)]
    _, _, yaw = integrate(turning, lsb, tpm)
    check("1350 LSB-s at 15 LSB/dps is a quarter turn", math.degrees(yaw), 90.0,
          tolerance=1e-6)

    # Driving and turning together must trace an arc, not a corner. The midpoint
    # heading is what makes the difference, and getting it wrong is a bias that
    # only shows up as a map that curls.
    arc = [(0.0, 0.0, 0), (1350.0, 100.0, 0)]
    x, y, _ = integrate(arc, lsb, tpm)
    check("a metre while turning 90 degrees goes diagonally, not straight up",
          abs(x - y) < 1e-6 and x > 0.6, True)

    # A counter break is a hole, and integrating across it invents a jump.
    broken = [(0.0, 0.0, 0), (0.0, 100.0, 0), (0.0, 5.0, 1), (0.0, 105.0, 1)]
    x, _, _ = integrate(broken, lsb, tpm)
    check("a reset counter does not become a 95 cm leap backwards", x, 2.0,
          tolerance=1e-9)


TESTS = (
    test_idle_behaviour,
    test_gyro_bias,
    test_odometry,
)
