"""Can the camera be pointed at a thing in one move? Measure it, per point in frame.

    python usb_cameras/calibrate_aim.py --selftest
    python usb_cameras/calibrate_aim.py aim/                 # on the rover
    python usb_cameras/calibrate_aim.py aim/ --fit-only      # re-measure saved frames

[calibrate_fov.py](calibrate_fov.py) asks how wide the lens is. This asks the
question the tracking loop actually depends on: given a face at some pixel, are the
degrees [aiming.py](../face_tracking/aiming.py) works out the degrees that put it in
the middle -- **in one move, from anywhere in the frame**, rather than after a series
of ever-smaller nudges. A loop whose one-shot answer is wrong still converges, because
it re-measures every frame, but it converges by walking there; at the two and a bit
frames a second this rover's own detector manages, walking there looks exactly like
hunting.

The test needs no chart and nobody holding anything. Pick a textured patch at a known
pixel, ask the model under test what would centre it, command precisely that, and see
where the patch actually ended up. What is left over is the model's error, and it is
reported in degrees rather than pixels because degrees are what was commanded.

**Two models are tried on the same targets, back to back.** `--model sphere` is what
the rover flies: the pixel as a direction through the fitted fisheye, and the
pan-then-tilt geometry solved for it, by calling `aiming.solve` itself rather than a
copy of it -- so this is a test of the deployed answer and not of agreement between
two files. `--model separable` is what it flew until 2026-08-19: the horizontal error
times a degrees-per-half-frame constant, the vertical error times another, the two
axes independent. It is kept because a comparison needs something to compare against,
and because the size of the error it made is the reason the other one exists. The two
agree on the centre lines and part company towards the corners, so a run that only
probed the middle of the frame would find nothing.

**Measuring the residual is where the care goes.** A patch at the edge of a 132 degree
lens is squeezed across its short way by a quarter, so matching it against its own
centred self is matching two different shapes -- and that mismatch would be read as
aiming error. Neither frame is therefore matched raw: both are resampled through the
fitted lens onto a flat tangent plane, the before-frame around the target's direction
and the after-frame around the axis, at one shared angular scale. What is compared is
then two views of the same thing at the same size, and the answer comes out in degrees
off axis with no pixels-per-degree conversion anywhere in it.
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

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "voice_chat"))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "face_tracking"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from rover_tools import RoverClient, discover                      # noqa: E402
from calibrate_fov import directions, project, turned, _theta      # noqa: E402
import aiming                                                      # noqa: E402

# The lens this rover wears, as calibrate_fov.py measures it: near enough an
# equidistant fisheye, about 130 degrees across. Kept here as a default so that a run
# costs one trip to the hardware rather than two, and overridable with --lens because
# a freshly measured sweep should always be believed over a remembered one.
#
# Measured 2026-08-19 by two sweeps that share no motion: a pan sweep gave 11.85
# arcmin per pixel with a distortion term of +0.025, a tilt sweep 11.79 and +0.035.
# They agree on the scale to half a percent, which is the part this file leans on.
# The centre is taken one axis from each, because a sweep pins the coordinate it
# moves along and says almost nothing about the other: cx from the pan run, cy from
# the tilt run. See docs/usb-cameras.md.
LENS = {"arcmin_per_px": 11.82, "bend": 0.030, "centre": (315.9, 227.4)}

# Where in the frame to try, as fractions of a half frame from the middle: +1 is the
# right edge and the top. Chosen to walk out along the centreline, up the middle, and
# only then diagonally, because the two models agree exactly on the first two and the
# whole of their disagreement lives in the third.
# Kept clear of the edges by more than the patch's own radius: a patch cut around a
# target too near the border is half off the picture, and what the matcher then finds
# is somewhere else in the room. `landed` says so rather than being trusted, but a
# target list that provokes it is simply a worse experiment.
TARGETS = [
    (0.25, 0.0), (0.5, 0.0), (0.7, 0.0), (0.85, 0.0),
    (0.0, 0.4), (0.0, 0.75),
    (0.5, 0.45), (0.7, 0.55), (0.85, 0.75), (-0.7, 0.55), (-0.5, 0.45),
    (-0.85, -0.45),
]
# Where the camera starts. Not necessarily straight ahead: everything here is
# relative to wherever it began, and the room a rover happens to be standing in is
# textured on some headings and a blank white floor on others. Starting tilted is
# worth doing on purpose as well -- the coupling between the two axes is a function
# of the tilt already on, so a probe run only from level tests the one case where
# there is least of it.
HOME = (0.0, 0.0)
# How big a patch is cut out to be found again, in degrees of the tangent plane, and
# how far around the axis the after-frame is searched. The search has to cover the
# error being looked for: the separable model is out by up to 20 degrees in the
# corner, so a window covering only a few would find nothing and report a miss.
PATCH_DEG = 7.0
SEARCH_DEG = 24.0
# Room to nudge a target onto something worth matching. A blank wall is not a
# measurement, and the exact pixel does not matter as long as it is *recorded*.
NUDGE_PX = 45
# The servo is given this long to arrive, and a second frame is taken to prove it
# has: look_at sends SPD 0, which is the servo's own ceiling of about 130 deg/s, so
# even the far corner is a third of a second of travel. The rest is the camera's own
# exposure settling after the feed is reopened.
SETTLE_S = 2.5
TIMEOUT_S = 30.0
# Below this the patch was not recognised, and whatever the match landed on is some
# other part of the room. Such a row is printed and then left out of the summary,
# because reading it as a large aiming error is exactly the wrong conclusion. A room
# repeats itself -- four identical dining chairs, a row of framed pictures -- so the
# bar is well above chance: a correct match on a real frame scores 0.85 and up, and
# the mistaken ones seen here clustered around 0.6.
MIN_SCORE = 0.7


# --------------------------------------------------------------------------- lens

def lens_of(size, spec=LENS):
    """(scale, bend, centre, normal) for a frame this size, as calibrate_fov names them."""
    width, height = size
    centre = spec.get("centre") or (width / 2.0, height / 2.0)
    return (math.radians(spec["arcmin_per_px"] / 60.0), spec["bend"],
            tuple(centre), width / 2.0)


def ray(point, lens):
    """The direction a pixel looks along: x right, y down, z out of the lens."""
    scale, bend, centre, normal = lens
    return directions(np.asarray([point], float), scale, bend, centre, normal)[0]


def middle(size, lens):
    """The direction of the middle of the picture, which is what aiming aims at.

    Not the same as the lens's own axis, and the difference is why this is a function
    rather than a forward vector written out: the fit puts the axis about thirteen
    pixels above the middle of the frame, and a controller that drove a face onto the
    axis instead would leave it two and a half degrees high in every picture.
    """
    return ray((size[0] / 2.0, size[1] / 2.0), lens)


# ------------------------------------------------------------------------- aiming

def solve(seen, wanted, tilt_now):
    """Degrees of pan and tilt that move the direction `seen` onto `wanted`.

    One move, exactly, and no iteration. The gimbal pans about the world's vertical
    and tilts about its own horizontal, in that order, so the two axes are not
    independent and cannot be solved one at a time. Undo the tilt first, which puts
    the direction back in the frame the pan turns within. A pan cannot change how far
    a direction sits out of that frame's forward plane, so the pan is the one leaving
    it exactly as far out as the destination is; the tilt that remains then follows
    outright.

    That coupling is the whole difference between this and the separable model, and it
    is invisible on the centreline: with the target level with the camera, or
    directly above it, both terms fall away and the two agree.
    """
    cos, sin = math.cos(math.radians(tilt_now)), math.sin(math.radians(tilt_now))
    ax, ay, az = seen[0], seen[1] * cos - seen[2] * sin, seen[1] * sin + seen[2] * cos
    across = math.hypot(ax, az)
    if across < 1e-12 or abs(wanted[0]) > across:
        return 0.0, 0.0                    # unreachable; the caller sees no move
    pan = math.atan2(ax, az) - math.asin(wanted[0] / across)
    forward = math.sqrt(max(across * across - wanted[0] * wanted[0], 0.0))
    tilt = math.atan2(wanted[1], wanted[2]) - math.atan2(ay, forward)
    return math.degrees(pan), math.degrees(tilt) - tilt_now


def aim_sphere(point, tilt_now, lens, size):
    """Degrees that bring this pixel to the middle of the picture, in one move.

    The rays are built from *this* file's lens, so that --lens can try a freshly
    measured one, but the geometry is aiming.solve -- the code that flies. A local
    reimplementation would pass this test while the rover failed it.
    """
    return aiming.solve(ray(point, lens), middle(size, lens), tilt_now)


# Degrees per half frame, as the separable model had them at 640x480 and 1280x720.
# Written down here rather than read from aiming.py because aiming.py no longer has
# them: they were measured by template-matching one patch across a commanded step,
# and the numbers are kept so that what replaced them can be measured against them.
SEPARABLE_HALF_FRAME = {(640, 480): (61.3, 45.1), (1280, 720): (66.0, 38.0)}


def aim_separable(point, size):
    """What aiming.py commanded before 2026-08-19: two errors, two multiplications."""
    width, height = size
    gains = SEPARABLE_HALF_FRAME.get((width, height))
    if gains is None:                       # scaled the way gains_for used to
        gains = SEPARABLE_HALF_FRAME[(640, 480)]
    error_x = (point[0] - width / 2.0) / (width / 2.0)
    error_y = -(point[1] - height / 2.0) / (height / 2.0)
    return (aiming.PAN_SIGN * error_x * gains[0],
            aiming.TILT_SIGN * error_y * gains[1])


def aim(model, point, size, tilt_now, lens):
    if model == "separable":
        return aim_separable(point, size)
    return aim_sphere(point, tilt_now, lens, size)


# -------------------------------------------------------------------- rectifying

def tangent(frame, lens, direction, half_deg, per_deg, want_basis=False):
    """The frame resampled onto a flat plane touching the sphere at `direction`.

    Two pictures of the same thing taken from different parts of a fisheye are not
    the same shape -- a patch out at 70 degrees is squeezed across its short way by a
    quarter -- and matching one against the other reads that squeeze as displacement.
    Both sides of the comparison are therefore brought here first, where a degree is
    the same number of pixels everywhere and in every direction, and the only thing
    left that can differ between them is where the thing is.
    """
    scale, bend, centre, normal = lens
    forward = np.asarray(direction, float)
    forward = forward / np.linalg.norm(forward)
    right = np.cross(forward, (0.0, -1.0, 0.0))
    right = right / np.linalg.norm(right)
    down = np.cross(forward, right)
    span = math.tan(math.radians(half_deg))
    side = int(round(2 * half_deg * per_deg)) | 1
    offsets = np.linspace(-span, span, side)
    u, v = np.meshgrid(offsets, offsets)
    rays = (forward[None, None, :] + u[..., None] * right[None, None, :]
            + v[..., None] * down[None, None, :]).reshape(-1, 3)
    rays = rays / np.linalg.norm(rays, axis=1, keepdims=True)
    points = project(rays, scale, bend, centre, normal).reshape(side, side, 2)
    height, width = frame.shape[:2]
    inside = float(np.mean((points[..., 0] >= 0) & (points[..., 0] <= width - 1)
                           & (points[..., 1] >= 0) & (points[..., 1] <= height - 1)))
    out = cv2.remap(frame, points[..., 0].astype(np.float32),
                    points[..., 1].astype(np.float32), cv2.INTER_LINEAR,
                    borderValue=0)
    if want_basis:
        return out, offsets, (forward, right, down)
    return out, offsets, inside


def _grey(image):
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image


def landed(before, after, target, lens, size, per_deg=None):
    """Where the thing at `target` in `before` has got to in `after`, as a direction.

    Returns that direction in the after-frame's own coordinates, how well the patch
    was recognised, and how much of the patch was on the picture at all. A low score
    means the answer is about some other part of the room and must not be read as a
    large error; a low coverage means the target was too near the border to cut a
    whole patch around, which is the usual cause of a low score.
    """
    per_deg = per_deg or math.radians(1.0) / lens[0]
    patch, _, inside = tangent(before, lens, ray(target, lens), PATCH_DEG, per_deg)
    window, offsets, basis = tangent(after, lens, middle(size, lens), SEARCH_DEG,
                                     per_deg, want_basis=True)
    result = cv2.matchTemplate(_grey(window), _grey(patch), cv2.TM_CCOEFF_NORMED)
    _, score, _, at = cv2.minMaxLoc(result)
    # Where the patch's own centre landed, in the window's tangent coordinates.
    u = float(offsets[at[0] + patch.shape[1] // 2])
    v = float(offsets[at[1] + patch.shape[0] // 2])
    forward, right, down = basis
    found = forward + u * right + v * down
    return found / np.linalg.norm(found), float(score), inside


def still_needed(direction, tilt_now, lens, size):
    """The pan and tilt still owed, given where the thing actually ended up."""
    return solve(direction, middle(size, lens), tilt_now)


def textured(frame, target, half=32, nudge=NUDGE_PX):
    """A pixel near `target` with something on it. A blank wall is not a measurement."""
    grey = _grey(frame)
    height, width = grey.shape[:2]
    energy = cv2.boxFilter(cv2.Laplacian(grey, cv2.CV_32F) ** 2, -1, (half, half))
    x0, x1 = int(max(half, target[0] - nudge)), int(min(width - half, target[0] + nudge))
    y0, y1 = int(max(half, target[1] - nudge)), int(min(height - half, target[1] + nudge))
    if x1 <= x0 or y1 <= y0:
        return (float(np.clip(target[0], half, width - half)),
                float(np.clip(target[1], half, height - half)))
    box = energy[y0:y1, x0:x1]
    where = np.unravel_index(int(np.argmax(box)), box.shape)
    return float(x0 + where[1]), float(y0 + where[0])


# ------------------------------------------------------------------------ capture

def picture(rover, path):
    """One frame from the rover, kept on disk. The second of two, as calibrate_fov does.

    The daemon hands out whatever the camera last delivered, and on a feed that has
    just been reopened that can predate the move being measured.
    """
    for _ in range(2):
        shot = rover.call("camera_jpeg", {})
    if not shot.get("ok"):
        raise SystemExit(f"no picture from the rover: {shot.get('error')}")
    path.write_bytes(base64.b64decode(shot["jpeg_base64"]))
    return shot["width"], shot["height"]


def probe(rover, folder, models, targets, settle, home=HOME):
    """Command each model's answer for each target, keeping every frame it took."""
    folder.mkdir(parents=True, exist_ok=True)
    rover.call("look_at", {"pan": home[0], "tilt": home[1]})
    time.sleep(settle + 1.0)
    size = picture(rover, folder / "warm.jpg")
    lens = lens_of(size)
    trials = []
    for index, (fx, fy) in enumerate(targets):
        nominal = (size[0] / 2 * (1 + fx), size[1] / 2 * (1 - fy))
        back = rover.call("look_at", {"pan": home[0], "tilt": home[1]})
        time.sleep(settle)
        reference = f"ref_{index:03d}.jpg"
        picture(rover, folder / reference)
        target = textured(cv2.imread(str(folder / reference)), nominal)
        # Both models must be asked to make the same move, or the trial compares two
        # different experiments; and neither may run into a limit, because a servo
        # held against its stop is not a model being tested. So the whole target is
        # dropped if any of it does not fit, which is a thing about where the camera
        # was started rather than about the models.
        steps = {model: aim(model, target, size, float(back["tilt"]), lens)
                 for model in models}
        outside = [f"{model} wants pan {back['pan'] + step[0]:+.0f} "
                   f"tilt {back['tilt'] + step[1]:+.0f}"
                   for model, step in steps.items()
                   if abs(back["pan"] + step[0]) > aiming.PAN_LIMIT - 1
                   or not (aiming.TILT_LIMITS[0] + 1 <= back["tilt"] + step[1]
                           <= aiming.TILT_LIMITS[1] - 1)]
        if outside:
            print(f"  skipped ({target[0]:5.0f},{target[1]:5.0f}) px: "
                  + "; ".join(outside) + ", outside the gimbal's travel")
            continue
        for model in models:
            step = steps[model]
            pan, tilt = back["pan"] + step[0], back["tilt"] + step[1]
            sent = rover.call("look_at", {"pan": pan, "tilt": tilt})
            if not sent.get("ok"):
                raise SystemExit(f"the rover would not look there: {sent}")
            time.sleep(settle)
            name = f"{model}_{index:03d}.jpg"
            picture(rover, folder / name)
            clamped = abs(sent["pan"] - pan) > 1.0 or abs(sent["tilt"] - tilt) > 1.0
            trials.append({"model": model, "target": list(target),
                           "nominal": list(nominal), "wanted": [pan, tilt],
                           "home": [back["pan"], back["tilt"]], "step": list(step),
                           "commanded": [sent["pan"], sent["tilt"]],
                           "clamped": clamped, "reference": reference,
                           "file": name, "size": list(size)})
            print(f"  {model:7s} ({target[0]:5.0f},{target[1]:5.0f}) px  ->  "
                  f"pan {step[0]:+6.1f} tilt {step[1]:+6.1f}"
                  + ("   CLAMPED by the firmware" if clamped else ""))
    meta = {"lens": LENS, "home": list(home), "trials": trials}
    (folder / "aim.json").write_text(json.dumps(meta, indent=1))
    return meta


# ------------------------------------------------------------------------ measure

def measure(folder):
    """Read a probe and say how far each model missed by."""
    meta = json.loads((folder / "aim.json").read_text())
    size = tuple(meta["trials"][0]["size"])
    lens = lens_of(size, meta.get("lens", LENS))
    scale = lens[0]
    rows, by_model = [], {}
    for trial in meta["trials"]:
        before = cv2.imread(str(folder / trial["reference"]))
        after = cv2.imread(str(folder / trial["file"]))
        if before is None or after is None:
            continue
        target = tuple(trial["target"])
        radius = math.hypot(target[0] - size[0] / 2, target[1] - size[1] / 2)
        direction, score, inside = landed(before, after, target, lens, size)
        pan_left, tilt_left = still_needed(direction, trial["commanded"][1], lens, size)
        miss = math.degrees(math.acos(min(1.0, float(direction @ middle(size, lens)))))
        row = {"model": trial["model"], "target": target,
               "off_axis_deg": math.degrees(_theta(radius, lens[0], lens[1], lens[3])),
               "step": trial.get("step", trial["commanded"]),
               "clamped": trial.get("clamped", False),
               "pan_left": pan_left, "tilt_left": tilt_left,
               "miss_deg": miss, "miss_px": miss / math.degrees(scale),
               "score": score, "inside": inside}
        rows.append(row)
        if score >= MIN_SCORE and not row["clamped"]:
            by_model.setdefault(trial["model"], []).append(row)

    print(f"{len(rows)} trials at {size[0]}x{size[1]}, lens "
          f"{math.degrees(scale) * 60:.2f} arcmin per pixel on the axis\n")
    print(f"{'target px':>12} {'off axis':>9} | {'model':7} {'moved by':>15} "
          f"| {'still owed':>16} {'miss':>19} {'match':>6}")
    for row in rows:
        note = "" if row["score"] >= MIN_SCORE else "   (patch not found)"
        note += f"   (patch {1 - row['inside']:.0%} off the picture)"             if row["inside"] < 0.98 else ""
        note += "   (clamped)" if row["clamped"] else ""
        print(f"{row['target'][0]:6.0f},{row['target'][1]:5.0f} "
              f"{row['off_axis_deg']:8.1f} | {row['model']:7} "
              f"{row['step'][0]:+7.1f},{row['step'][1]:+6.1f} | "
              f"{row['pan_left']:+7.1f},{row['tilt_left']:+7.1f} "
              f"{row['miss_deg']:9.1f} deg {row['miss_px']:5.0f} px "
              f"{row['score']:6.2f}{note}")
    print()
    for model, got in by_model.items():
        misses = [row["miss_deg"] for row in got]
        worst = max(got, key=lambda row: row["miss_deg"])
        print(f"{model:7}: {len(got)} usable, median miss {np.median(misses):5.1f} deg, "
              f"worst {worst['miss_deg']:5.1f} deg ({worst['miss_px']:.0f} px) at "
              f"{worst['off_axis_deg']:.0f} deg off axis")
    return rows


# ----------------------------------------------------------------------- selftest

def _room(width=4096, height=2048, seed=11):
    """A synthetic room, textured evenly *on the sphere* rather than in the map of it.

    An equirectangular picture crowds its columns together towards the poles, so a
    texture drawn evenly across it is finer up there than it is at the equator -- and
    the corner targets are precisely the ones the camera looks steepest up at, where
    a patch would come out too fine to survive being sampled at the lens's own five
    pixels per degree. Stretching each row by the secant of its latitude undoes that,
    and without it this test fails at the corners for a reason that has nothing to do
    with aiming.
    """
    generator = np.random.default_rng(seed)
    grown = np.zeros((height, width), np.float32)
    for step, weight in ((64, 0.45), (16, 0.35), (6, 0.20)):
        seed_grid = generator.random((height // step, width // step)).astype(np.float32)
        grown += weight * cv2.resize(seed_grid, (width, height),
                                     interpolation=cv2.INTER_CUBIC)
    latitude = (0.5 - (np.arange(height) + 0.5) / height) * math.pi
    columns = np.arange(width, dtype=np.float32)[None, :] - width / 2.0
    mx = (width / 2.0 + columns * np.cos(latitude, dtype=np.float32)[:, None])
    my = np.repeat(np.arange(height, dtype=np.float32)[:, None], width, axis=1)
    even = cv2.remap(grown, mx.astype(np.float32), my, cv2.INTER_LINEAR,
                     borderMode=cv2.BORDER_WRAP)
    return cv2.cvtColor(np.clip(even * 255, 0, 255).astype(np.uint8),
                        cv2.COLOR_GRAY2BGR)


def _shot(room, size, lens, pan, tilt):
    """The room as this camera sees it from a pose, the pan applied before the tilt."""
    width, height = size
    scale, bend, centre, normal = lens
    grid = np.stack(np.meshgrid(np.arange(width, dtype=np.float64),
                                np.arange(height, dtype=np.float64)), axis=-1)
    rays = directions(grid.reshape(-1, 2), scale, bend, centre, normal)
    world = turned(turned(rays, "tilt", -tilt), "pan", -pan)
    longitude = np.arctan2(world[:, 0], world[:, 2])
    latitude = np.arcsin(np.clip(-world[:, 1], -1.0, 1.0))
    rows, columns = room.shape[:2]
    mx = ((longitude / (2 * math.pi) + 0.5) * columns).astype(np.float32)
    my = ((0.5 - latitude / math.pi) * rows).astype(np.float32)
    return cv2.remap(room, mx.reshape(height, width), my.reshape(height, width),
                     cv2.INTER_LINEAR, borderMode=cv2.BORDER_WRAP)


def _ends_at(target, lens, pan, tilt):
    """The target's direction in the camera frame after moving to this pose."""
    world = np.asarray([ray(target, lens)])
    return turned(turned(world, "pan", pan), "tilt", tilt)[0]


def selftest():
    """Point a camera whose lens and kinematics are known, and check the answer.

    Two things are on trial and only one of them is the rover's. The geometry has to
    be right -- aim_sphere must land a target on the axis to a fraction of a degree,
    from anywhere in the frame, or its arctangents are in the wrong order. And the
    *measurement* has to be right: a residual read off two rendered frames has to
    agree with the residual the render was built to have, or a real run would be
    reporting the rectifier's own error as the rover's.
    """
    size, room = (640, 480), _room()
    lens = lens_of(size)
    ok = True

    print("aiming a camera whose lens is known, from pan 0 tilt 0:\n")
    print(f"{'target px':>12} {'off axis':>9} | {'sphere left over':>18} "
          f"| {'separable left over':>18}")
    for fx, fy in TARGETS:
        target = (size[0] / 2 * (1 + fx), size[1] / 2 * (1 - fy))
        radius = math.hypot(target[0] - size[0] / 2, target[1] - size[1] / 2)
        left = {}
        for model in ("sphere", "separable"):
            pan, tilt = aim(model, target, size, 0.0, lens)
            left[model] = still_needed(_ends_at(target, lens, pan, tilt), tilt,
                                       lens, size)
        print(f"{target[0]:6.0f},{target[1]:5.0f} "
              f"{math.degrees(_theta(radius, lens[0], lens[1], lens[3])):8.1f} | "
              f"{left['sphere'][0]:+8.2f},{left['sphere'][1]:+8.2f} | "
              f"{left['separable'][0]:+8.2f},{left['separable'][1]:+8.2f}")
        ok &= max(abs(left["sphere"][0]), abs(left["sphere"][1])) < 0.05

    print("\nthe same moves rendered and measured back, to test the measurement:\n")
    print(f"{'target px':>12} | {'model':7} {'rendered':>18} {'measured':>18} "
          f"{'out by':>7} {'match':>6}")
    checked = []
    for fx, fy in TARGETS[:6] + TARGETS[7:9]:
        target = (size[0] / 2 * (1 + fx), size[1] / 2 * (1 - fy))
        before = _shot(room, size, lens, 0.0, 0.0)
        for model in ("sphere", "separable"):
            pan, tilt = aim(model, target, size, 0.0, lens)
            pan, tilt = float(round(pan)), float(round(tilt))   # look_at takes whole degrees
            after = _shot(room, size, lens, pan, tilt)
            truth = still_needed(_ends_at(target, lens, pan, tilt), tilt, lens, size)
            direction, score, inside = landed(before, after, target, lens, size)
            got = still_needed(direction, tilt, lens, size)
            out = math.hypot(got[0] - truth[0], got[1] - truth[1])
            found = score >= MIN_SCORE
            print(f"{target[0]:6.0f},{target[1]:5.0f} | {model:7} "
                  f"{truth[0]:+8.2f},{truth[1]:+8.2f} {got[0]:+8.2f},{got[1]:+8.2f} "
                  f"{out:7.2f} {score:6.2f}"
                  + ("" if found else "   (not found, and said so)")
                  + ("" if inside > 0.98 else f"   ({1 - inside:.0%} off the picture)"))
            # A patch that was not recognised is allowed -- being told so is the
            # point of MIN_SCORE -- but one that *was* has to be located correctly,
            # and most of them have to be, or the rectifier is not doing its job.
            ok &= out < 1.5 or not found
            checked.append(found)
    print("\nPASS" if ok else "\nFAIL")
    return 0 if ok else 1


def main() -> int | str:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("folder", nargs="?", type=Path)
    parser.add_argument("--rover", default=None, metavar="HOST[:PORT]")
    parser.add_argument("--model", choices=("separable", "sphere", "both"),
                        default="both")
    parser.add_argument("--mirror", action="store_true",
                        help="try every target on the other side of the frame, for "
                             "when the room's texture or the gimbal's travel is on "
                             "that side")
    parser.add_argument("--settle", type=float, default=SETTLE_S, metavar="SECONDS")
    parser.add_argument("--lens", type=float, nargs=4, default=None,
                        metavar=("ARCMIN", "BEND", "CX", "CY"),
                        help="the lens calibrate_fov.py fitted, if it has been "
                             "measured again since the default above was recorded")
    parser.add_argument("--from", dest="home", type=float, nargs=2, default=HOME,
                        metavar=("PAN", "TILT"),
                        help="the pose to start every trial from, in the degrees "
                             "look_at uses (default straight ahead and level)")
    parser.add_argument("--fit-only", action="store_true",
                        help="measure frames already captured, without a rover")
    parser.add_argument("--selftest", action="store_true",
                        help="render a camera whose aim is known and check it back")
    args = parser.parse_args()

    if args.lens:
        LENS.update(arcmin_per_px=args.lens[0], bend=args.lens[1],
                    centre=(args.lens[2], args.lens[3]))
    if args.selftest:
        return selftest()
    if args.folder is None:
        parser.error("a folder to put the probe in, or --selftest")
    models = ("separable", "sphere") if args.model == "both" else (args.model,)
    if not args.fit_only:
        rover = RoverClient(args.rover) if args.rover else discover()
        if rover is None:
            return "no rover daemon found; name one with --rover"
        rover.timeout = TIMEOUT_S
        if not rover.probe():
            return f"no rover daemon at {rover.describe()}"
        print(f"probing {', '.join(models)} on {rover.describe()}, "
              f"from pan {args.home[0]:+.0f} tilt {args.home[1]:+.0f}")
        try:
            targets = [(-fx, fy) for fx, fy in TARGETS] if args.mirror else TARGETS
            probe(rover, args.folder, models, targets, args.settle, tuple(args.home))
        finally:
            rover.call("center_camera", {})
            rover.close()
    measure(args.folder)
    return 0


if __name__ == "__main__":
    sys.exit(main())
