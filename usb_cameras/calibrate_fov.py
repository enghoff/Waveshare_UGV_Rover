"""Measure a camera's field of view by turning it a known amount.

    python usb_cameras/calibrate_fov.py --selftest
    python usb_cameras/calibrate_fov.py sweep/            # gimbal, horizontal
    python usb_cameras/calibrate_fov.py sweep/ --by rover
    python usb_cameras/calibrate_fov.py sweep/ --axis tilt

No chart to print, no tape measure, nobody holding a chessboard: the rover already
owns two things that turn its camera by a known angle, so the room it is standing in
is the calibration target. Point the camera at the room, turn it a little, and every
feature in the picture slides across the frame. How far each one slides for a given
turn is the lens's angular scale, and adding that up across the width is the field of
view.

Which of the two turns is doing the work matters, so both are offered. `--by gimbal`
pans the camera and trusts the pan servo's degrees; `--by rover` turns the whole
chassis and takes the angle from the lidar's scan match, which is measured against
the walls rather than asked for. Running both is the point of having both -- they
share no mechanism, so agreement means the servo is honest as well. (What vouches for
the scan match itself is a separate measurement in the same spirit:
[ros_nav/calibrate_chassis.py](../ros_nav/calibrate_chassis.py) cross-correlates the
range profile before and after a turn, which the matcher takes no part in.)

**Why this fits a lens model rather than averaging pixel shifts.** The obvious
version -- shift in pixels, divided into the angle turned -- is wrong on a wide lens
in a way that flatters it. Under a pan, a feature near the top of the frame slides
*less* than one on the centreline, for the same reason that a degree of longitude is
shorter away from the equator, so averaging over the whole frame overstates the
degrees per pixel and so overstates the field. Measured on this rover that error was
about 6 degrees. So this fits the projection itself -- angular scale, one distortion
term and the principal point -- to the full two-dimensional motion of every tracked
point, and reads the field of view off the fitted model.

`--selftest` needs no rover and no camera. It renders a synthetic room through a lens
of known field of view, measures it back and checks the answer, which is the only way
to know that the model, the sign conventions and the fit are right rather than merely
self-consistent.
"""

from __future__ import annotations

import argparse
import base64
import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

# The daemon's own client, rather than a fourth socket in this repository. It lives
# under voice_chat/ because that is what needed it first; nothing in it is about
# voice, and reaching for it here is cheaper than a second copy of the reconnect and
# discovery logic. See voice_chat/rover_tools.py.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "voice_chat"))
from rover_tools import RoverClient, discover                    # noqa: E402

# A step small enough that nearly every feature is still in the picture afterwards
# and large enough that its motion is many pixels rather than a few. On a lens this
# wide 5 degrees is about 24 px, which optical flow measures to a small fraction of a
# pixel, so the angle is the limiting error and not the tracking.
STEP_DEG = 5.0
# How far either side of centre to sweep. Not the whole range: the point is to
# measure the lens, and a feature is most useful while it is in shot, so the sweep
# only needs to be long enough to carry features right across the frame.
SPAN_DEG = 50.0
# The servo is given this long to arrive. Measured by walking the sweep with a
# stopwatch: 5 degrees is done well inside half a second, and the rest is the
# chassis settling on its springs after a turn.
SETTLE_S = 1.2
# turn_in_place blocks until it arrives, and a refused turn waits out its own
# timeout, so this client waits longer than the voice one would.
TIMEOUT_S = 60.0

# Points are tracked in the upper part of the frame by default. The floor is the
# nearest thing in any picture a rover takes -- half a metre at the bottom edge --
# and swinging the lens sideways slides a close surface by parallax as well as by
# rotation, which is motion this model does not describe and should not be asked to
# absorb. Walls and what is hung on them are metres away, where the difference does
# not survive the fit's own noise.
BAND = (0.05, 0.55)
# Enough points to pin four parameters down thoroughly, few enough that the fit is a
# second rather than a minute.
MAX_POINTS = 6000


# --------------------------------------------------------------------------- lens

def _theta(radius, scale, bend, normal):
    """Angle off the lens axis, in radians, for a point this many pixels out.

    An equidistant fisheye puts angle in proportion to radius -- which is what this
    camera turned out to be, near enough -- and `bend` is the one term that lets the
    fit say otherwise. It is written against a normalised radius so that it comes
    out around a hundredth rather than around 1e-9, which is the difference between a
    fit that converges and one that wanders.
    """
    return radius * scale * (1.0 + bend * (radius / normal) ** 2)


def _radius(theta, scale, bend, normal):
    """The inverse of :func:`_theta`, by Newton. Monotone, so this is safe."""
    radius = theta / scale
    for _ in range(20):
        guess = _theta(radius, scale, bend, normal)
        slope = scale * (1.0 + 3.0 * bend * (radius / normal) ** 2)
        radius = radius - (guess - theta) / slope
    return radius


def directions(points, scale, bend, centre, normal):
    """Unit vectors for image points: x right, y down, z out of the lens."""
    offset = points - np.asarray(centre)
    radius = np.hypot(offset[:, 0], offset[:, 1])
    theta = _theta(radius, scale, bend, normal)
    # A point exactly on the axis has no direction to be off-axis in. Its sine is
    # zero anyway, so the division only has to not raise.
    safe = np.where(radius > 1e-9, radius, 1.0)
    sin = np.sin(theta)
    return np.stack([sin * offset[:, 0] / safe,
                     sin * offset[:, 1] / safe,
                     np.cos(theta)], axis=1)


def project(vectors, scale, bend, centre, normal):
    """Image points for unit vectors -- :func:`directions` the other way round."""
    flat = np.hypot(vectors[:, 0], vectors[:, 1])
    theta = np.arctan2(flat, vectors[:, 2])
    radius = _radius(theta, scale, bend, normal)
    safe = np.where(flat > 1e-9, flat, 1.0)
    return np.stack([centre[0] + radius * vectors[:, 0] / safe,
                     centre[1] + radius * vectors[:, 1] / safe], axis=1)


def turned(vectors, axis, degrees):
    """The same fixed directions, seen from a camera that has turned.

    Positive is right for `pan` and up for `tilt`, matching what `look_at` means by
    them -- and a sign error here draws a perfectly ordinary-looking answer, so
    `--selftest` renders its synthetic sweep through this same function and would
    fail if the two disagreed.
    """
    angle = math.radians(degrees)
    cos, sin = math.cos(angle), math.sin(angle)
    if axis == "pan":                       # about the y axis, which points down
        matrix = np.array([[cos, 0.0, -sin], [0.0, 1.0, 0.0], [sin, 0.0, cos]])
    else:                                   # about the x axis, which points right
        matrix = np.array([[1.0, 0.0, 0.0], [0.0, cos, sin], [0.0, -sin, cos]])
    return vectors @ matrix.T


# ---------------------------------------------------------------------------- fit

def _predict(params, pairs, axis, normal):
    """Where each tracked point should have gone, under these lens parameters."""
    scale, bend, cx, cy = params
    out = []
    for before, after, degrees in pairs:
        rays = directions(before, scale, bend, (cx, cy), normal)
        out.append(project(turned(rays, axis, degrees), scale, bend, (cx, cy), normal))
    return np.concatenate(out)


def fit(pairs, axis, size, rounds=40):
    """Angular scale, distortion and principal point, from tracked motion.

    Gauss-Newton on four parameters with a numerical Jacobian, which is plenty at
    this size and spares the repository a dependency on scipy for one solve. The
    reweighting is what makes it usable on a room with a person in it: somebody
    walking across the frame moves in a way no rotation explains, and so does a
    mistracked corner on a reflective floor, and both are pushed out of the fit
    rather than argued with.
    """
    width, height = size
    normal = width / 2.0
    observed = np.concatenate([after for _, after, _ in pairs])
    params = np.array([math.radians(90.0 / width), 0.0, width / 2.0, height / 2.0])
    weight = np.ones(len(observed) * 2)
    spread = float("nan")
    for _ in range(rounds):
        residual = (_predict(params, pairs, axis, normal) - observed).ravel()
        jacobian = np.empty((len(residual), len(params)))
        for index in range(len(params)):
            nudge = max(abs(params[index]), 1.0) * 1e-6
            moved = params.copy()
            moved[index] += nudge
            jacobian[:, index] = (
                (_predict(moved, pairs, axis, normal) - observed).ravel() - residual
            ) / nudge
        middle = np.median(np.abs(residual - np.median(residual)))
        spread = 1.4826 * middle or 1e-6
        weight = 1.0 / np.sqrt(1.0 + (residual / (2.0 * spread)) ** 2)
        step, *_ = np.linalg.lstsq(jacobian * weight[:, None], -residual * weight,
                                   rcond=None)
        params = params + step
        if np.linalg.norm(step / np.maximum(np.abs(params), 1e-9)) < 1e-9:
            break
    return params, normal, spread


def field_of_view(params, normal, size):
    """Degrees across the whole picture, both ways, from the fitted lens.

    Measured out to the edge from wherever the axis actually falls rather than from
    the middle of the sensor, because those are not the same point and the wedge on
    the map is a claim about the picture's edges.
    """
    scale, bend, cx, cy = params
    width, height = size

    def angle(pixels):
        return math.degrees(_theta(pixels, scale, bend, normal))

    return (angle(cx) + angle(width - cx), angle(cy) + angle(height - cy))


# ------------------------------------------------------------------------ tracking

def matches(before, after, band, axis):
    """Point pairs from one frame to the next, forward-backward checked.

    Both directions are tracked and the disagreement thrown away, which costs a
    second pass and removes most of what a plain flow gets wrong: a corner that
    slides along an edge, a specular highlight that is not attached to anything, the
    inside of a window. What survives is filtered once more on the sign of its
    motion, since a rotation moves the entire room one way and anything travelling
    against it is a person or a mistake.
    """
    first = cv2.cvtColor(before, cv2.COLOR_BGR2GRAY)
    second = cv2.cvtColor(after, cv2.COLOR_BGR2GRAY)
    height = first.shape[0]
    mask = np.zeros_like(first)
    mask[int(band[0] * height):int(band[1] * height), :] = 255
    seeds = cv2.goodFeaturesToTrack(first, maxCorners=1500, qualityLevel=0.01,
                                    minDistance=6, blockSize=7, mask=mask)
    if seeds is None:
        return np.empty((0, 2), np.float32), np.empty((0, 2), np.float32)
    flow = dict(winSize=(31, 31), maxLevel=4,
                criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 40, 0.01))
    ahead, found, _ = cv2.calcOpticalFlowPyrLK(first, second, seeds, None, **flow)
    back, _, _ = cv2.calcOpticalFlowPyrLK(second, first, ahead, None, **flow)
    keep = (found.ravel() == 1)
    keep &= np.linalg.norm(seeds - back, axis=2).ravel() < 1.0
    start, end = seeds[keep].reshape(-1, 2), ahead[keep].reshape(-1, 2)
    if not len(start):
        return start, end
    which = 0 if axis == "pan" else 1
    moved = end[:, which] - start[:, which]
    same = np.sign(np.median(moved)) or 1.0
    keep = moved * same > 0
    return start[keep], end[keep]


def pairs_from(frames, folder, axis, band):
    """Consecutive frames, tracked, as (before, after, degrees turned right/up)."""
    out = []
    for first, second in zip(frames, frames[1:]):
        before = cv2.imread(str(folder / first["file"]))
        after = cv2.imread(str(folder / second["file"]))
        start, end = matches(before, after, band, axis)
        if len(start) < 20:
            print(f"  {first['file']} -> {second['file']}: only {len(start)} points, "
                  f"skipped")
            continue
        if len(start) > MAX_POINTS // max(1, len(frames) - 1):
            step = len(start) // (MAX_POINTS // max(1, len(frames) - 1))
            start, end = start[::step], end[::step]
        out.append((start.astype(float), end.astype(float),
                    second["turned_deg"] - first["turned_deg"]))
    return out


# --------------------------------------------------------------------------- sweep

def sweep(rover, folder, axis, by, span, step, settle):
    """Turn the camera in steps, keeping one frame and one angle at each.

    Always from the same side, never back and forth. The gimbal has a little
    backlash -- about two pixels' worth, measured by returning to centre from either
    direction -- and a sweep that reverses puts that straight into the angles.
    """
    folder.mkdir(parents=True, exist_ok=True)
    frames = []

    def picture(index, turned_deg, extra):
        # Twice, and the second one kept. The first can predate the turn: the frame
        # in the daemon's slot is whatever the camera last delivered, which on a
        # camera that has just been opened is also whatever it saw while its
        # exposure was still settling.
        for _ in range(2):
            shot = rover.call("camera_jpeg", {})
        if not shot.get("ok"):
            raise SystemExit(f"no picture from the rover: {shot.get('error')}")
        name = f"{axis}_{index:03d}.jpg"
        (folder / name).write_bytes(base64.b64decode(shot["jpeg_base64"]))
        frames.append({"file": name, "turned_deg": turned_deg,
                       "size": [shot["width"], shot["height"]], **extra})
        return shot

    def heading():
        status = rover.call("nav_status", {})
        if not status.get("ok"):
            raise SystemExit("this rover has no lidar, so --by rover cannot measure "
                             "its own turn: " + str(status.get("error")))
        return status["pose"]["heading_deg"], status["match_score"]

    if by == "gimbal":
        angles = np.arange(-span, span + 1e-6, step)
        # Approached from outside the sweep so that the first step is going the same
        # way as all the others.
        rover.call("look_at", {axis: float(angles[0] - 3 * step),
                               "tilt" if axis == "pan" else "pan": 0.0})
        time.sleep(3.0)
        for index, angle in enumerate(angles):
            rover.call("look_at", {axis: float(angle),
                                   "tilt" if axis == "pan" else "pan": 0.0})
            time.sleep(settle)
            picture(index, float(angle), {"commanded": float(angle)})
            print(f"  {axis} {angle:+6.1f}")
    else:
        if axis != "pan":
            raise SystemExit("--by rover turns the chassis, which only pans")
        rover.call("look_at", {"pan": 0.0, "tilt": 0.0})
        time.sleep(3.0)
        # The turn the chassis actually made, off the lidar, rather than the one it
        # was asked for -- this controller overshoots a 5 degree request by half again
        # as much as often as not, and every one of those overshoots is measured.
        # Heading counts positive to the left and this file counts turns positive to
        # the right, hence the minus.
        start, score = heading()
        picture(0, 0.0, {"heading_deg": start, "match_score": score})
        print(f"  heading {start:+7.1f}")
        try:
            for index in range(1, int(2 * span / step) + 1):
                outcome = rover.call("turn_in_place", {"angle_deg": -step})
                if not outcome.get("ok"):
                    print(f"  the rover would not turn: {outcome.get('reason')}")
                    break
                time.sleep(settle)
                now, score = heading()
                picture(index, -(now - start), {"heading_deg": now,
                                               "match_score": score})
                print(f"  heading {now:+7.1f}  match {score:.3f}")
        finally:
            # Put it back. Measuring a lens is not a reason to leave the rover
            # facing a different way than it was found in, and a sweep of this
            # length ends a hundred degrees round from where it started.
            now, _ = heading()
            if abs(now - start) > 1.0:
                rover.call("turn_in_place", {"angle_deg": start - now})
                print(f"  turned back to {heading()[0]:+7.1f}")

    meta = {"axis": axis, "by": by, "step": step, "frames": frames}
    (folder / "sweep.json").write_text(json.dumps(meta, indent=1))
    return meta


def measure(folder, band=BAND):
    """Read a captured sweep and report what lens took it."""
    meta = json.loads((folder / "sweep.json").read_text())
    size = tuple(meta["frames"][0]["size"])
    turns = [b["turned_deg"] - a["turned_deg"]
             for a, b in zip(meta["frames"], meta["frames"][1:])]
    print(f"{len(meta['frames'])} frames at {size[0]}x{size[1]}, turned by "
          f"{meta['by']}, {min(turns):+.1f} to {max(turns):+.1f} deg a step")
    pairs = pairs_from(meta["frames"], folder, meta["axis"], band)
    if len(pairs) < 3:
        raise SystemExit("too few usable frame pairs to fit a lens")
    params, normal, spread = fit(pairs, meta["axis"], size)
    horizontal, vertical = field_of_view(params, normal, size)
    scale, bend, cx, cy = params
    points = sum(len(before) for before, _, _ in pairs)
    print(f"{points} tracked points over {len(pairs)} pairs, "
          f"residual {spread:.2f} px")
    print(f"centre of the lens {cx:.1f}, {cy:.1f} px "
          f"(the middle of the picture is {size[0] / 2:.0f}, {size[1] / 2:.0f})")
    print(f"{math.degrees(scale) * 60:.2f} arcmin per pixel on the axis, "
          f"distortion term {bend:+.4f}")
    print(f"\nfield of view: {horizontal:.1f} deg across, {vertical:.1f} deg down")
    print(f"the map's cone wants the horizontal one: --camera-fov {horizontal:.0f}")
    return horizontal, vertical


# ------------------------------------------------------------------------ selftest

def _room(width=2048, height=1024, seed=7):
    """A synthetic room to look at: smooth noise, all round, with corners in it."""
    generator = np.random.default_rng(seed)
    coarse = generator.random((height // 16, width // 16)).astype(np.float32)
    grown = cv2.resize(coarse, (width, height), interpolation=cv2.INTER_CUBIC)
    fine = generator.random((height // 4, width // 4)).astype(np.float32)
    grown = 0.6 * grown + 0.4 * cv2.resize(fine, (width, height),
                                           interpolation=cv2.INTER_CUBIC)
    picture = np.clip(grown * 255.0, 0, 255).astype(np.uint8)
    return cv2.cvtColor(picture, cv2.COLOR_GRAY2BGR)


def _render(room, size, params, normal, axis, degrees):
    """The room as this lens would see it, having turned by `degrees`."""
    width, height = size
    grid = np.stack(np.meshgrid(np.arange(width, dtype=np.float64),
                                np.arange(height, dtype=np.float64)), axis=-1)
    scale, bend, cx, cy = params
    rays = directions(grid.reshape(-1, 2), scale, bend, (cx, cy), normal)
    # The frames are what the camera sees having turned; the room is fixed. So each
    # ray goes back the other way to find the part of the room it lands on.
    world = turned(rays, axis, -degrees)
    longitude = np.arctan2(world[:, 0], world[:, 2])
    latitude = np.arcsin(np.clip(-world[:, 1], -1.0, 1.0))
    rows, columns = room.shape[:2]
    mx = ((longitude / (2 * math.pi) + 0.5) * columns).astype(np.float32)
    my = ((0.5 - latitude / math.pi) * rows).astype(np.float32)
    return cv2.remap(room, mx.reshape(height, width), my.reshape(height, width),
                     cv2.INTER_LINEAR, borderMode=cv2.BORDER_WRAP)


def selftest(folder=None):
    """Measure a lens whose field of view is known, because it was rendered.

    The whole method is here rather than in the numbers it produced on one rover:
    render a room through a lens of a stated field of view, sweep it, hand the
    frames back to the same code that reads a real sweep, and see whether the answer
    comes out. Nothing else checks the sign conventions -- a pan that is fitted
    backwards, or an axis swapped for the other one, gives a residual that looks fine
    and a field of view that is quietly wrong.
    """
    room = _room()
    size = (640, 480)
    ok = True
    for axis, want, bend in (("pan", 136.0, 0.0), ("pan", 100.0, -0.05),
                             ("tilt", 136.0, 0.0)):
        normal = size[0] / 2.0
        # Solve for the scale that gives the field of view asked for, so that the
        # thing being recovered is a number this test chose rather than one it read.
        across = size[0] / 2.0
        scale = math.radians(want / 2.0) / (across * (1.0 + bend))
        truth = np.array([scale, bend, size[0] / 2.0, size[1] / 2.0])
        stated = field_of_view(truth, normal, size)[0 if axis == "pan" else 1]
        frames, pairs = [], []
        angles = np.arange(-25.0, 25.1, 5.0)
        rendered = [_render(room, size, truth, normal, axis, float(a)) for a in angles]
        for index, (before, after) in enumerate(zip(rendered, rendered[1:])):
            start, end = matches(before, after, (0.05, 0.95), axis)
            pairs.append((start.astype(float), end.astype(float),
                          float(angles[index + 1] - angles[index])))
            frames.append(len(start))
        params, got_normal, spread = fit(pairs, axis, size)
        horizontal, vertical = field_of_view(params, got_normal, size)
        got = horizontal if axis == "pan" else vertical
        off = abs(got - stated)
        print(f"{axis:4s} bend {bend:+.2f}: rendered {stated:6.2f} deg, "
              f"measured {got:6.2f} deg, out by {off:.2f}  "
              f"({sum(frames)} points, residual {spread:.3f} px)")
        ok &= off < 1.0 and spread < 0.5

    # What this exists to catch: averaging the pixel shifts instead of fitting the
    # lens. The naive number is quoted here so that the size of the error it makes is
    # recorded rather than remembered -- it reads high, and on a wide lens it reads
    # high by more than any real disagreement between the two ways of turning.
    normal = size[0] / 2.0
    scale = math.radians(136.0 / 2.0) / (size[0] / 2.0)
    truth = np.array([scale, 0.0, size[0] / 2.0, size[1] / 2.0])
    rendered = [_render(room, size, truth, normal, "pan", float(a))
                for a in np.arange(-25.0, 25.1, 5.0)]
    shifts = []
    for before, after in zip(rendered, rendered[1:]):
        start, end = matches(before, after, (0.05, 0.55), "pan")
        shifts.append(np.median(5.0 / np.abs(end[:, 0] - start[:, 0])))
    naive = float(np.mean(shifts)) * size[0]
    print(f"averaging shifts over the upper band instead: {naive:.1f} deg, "
          f"{naive - 136.0:+.1f} out")
    ok &= naive > 136.0                       # high, as the docstring claims

    if folder is not None:                    # leave the synthetic sweep to look at
        folder.mkdir(parents=True, exist_ok=True)
        for index, frame in enumerate(rendered):
            cv2.imwrite(str(folder / f"synthetic_{index:03d}.jpg"), frame)

    print("selftest ok" if ok else "SELFTEST FAILED")
    return 0 if ok else 1


def main() -> int | str:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("folder", nargs="?", type=Path,
                        help="where the sweep's frames go, or already are")
    parser.add_argument("--rover", default=None, metavar="HOST[:PORT]",
                        help="the rover daemon (default: find it)")
    parser.add_argument("--by", choices=("gimbal", "rover"), default="gimbal",
                        help="turn the camera with the pan servo, or turn the whole "
                             "rover and take the angle from the lidar")
    parser.add_argument("--axis", choices=("pan", "tilt"), default="pan",
                        help="which way to sweep (default pan)")
    parser.add_argument("--span", type=float, default=SPAN_DEG, metavar="DEG")
    parser.add_argument("--step", type=float, default=STEP_DEG, metavar="DEG")
    parser.add_argument("--settle", type=float, default=SETTLE_S, metavar="SECONDS")
    parser.add_argument("--band", type=float, nargs=2, default=BAND, metavar="FRACTION",
                        help="which rows to track, as fractions of the height; the "
                             f"default {BAND} avoids the floor")
    parser.add_argument("--fit-only", action="store_true",
                        help="measure frames already captured, without a rover")
    parser.add_argument("--selftest", action="store_true",
                        help="render a lens of known field of view and measure it back")
    args = parser.parse_args()

    if args.selftest:
        return selftest(args.folder)
    if args.folder is None:
        parser.error("a folder to put the sweep in, or --selftest")
    if not args.fit_only:
        # Named or found, the same way talk.py does it -- and then given a longer
        # patience than a voice client would, because `turn_in_place` does not answer
        # until the rover has arrived.
        rover = RoverClient(args.rover) if args.rover else discover()
        if rover is None:
            return "no rover daemon found; name one with --rover"
        rover.timeout = TIMEOUT_S
        if not rover.probe():
            return f"no rover daemon at {rover.describe()}"
        print(f"sweeping {args.axis} by {args.by}, on {rover.describe()}")
        try:
            sweep(rover, args.folder, args.axis, args.by, args.span, args.step,
                  args.settle)
        finally:
            rover.call("center_camera", {})
            rover.close()
    measure(args.folder, tuple(args.band))
    return 0


if __name__ == "__main__":
    sys.exit(main())
