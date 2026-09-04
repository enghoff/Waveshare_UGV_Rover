#!/usr/bin/env python3
"""Where the OAK is bolted, measured against the camera the rover already trusts.

    ssh orin 'cd ~/ugv/world_state && python3 bench_oak.py'
    ssh orin 'cd ~/ugv/world_state && python3 bench_oak.py --pan -15 0 15'
    ssh orin 'cd ~/ugv/world_state && python3 bench_oak.py --save /tmp/oak-align'

The rover has two cameras that see the room and they have never been in the same
frame. The gimbal camera's optics were swept and fitted on this rover and every
bearing the world state holds was drawn through them; the OAK has sat on the
front measuring millimetres with nothing reading it, and **there are no
extrinsics between it and anything** -- which is the one thing standing between a
range and the world state that wants one. This measures them.

**No target and no tape measure.** Both cameras look at whatever room the rover
is in, and the answer comes out of the one thing they genuinely share: the same
physical points, seen down two known lenses from two places a few centimetres
apart. The OAK says how far away each point is, the gimbal camera says which
direction it lies in, and the pose that reconciles the two **is** the mount --
rotation and offset together, out of one solve, with the points that do not fit
thrown out rather than averaged in.

**Appearance was tried first and it does not work here, measured.** Matching the
two pictures' *regions* by the encoder vectors the resolver uses reads well and
falls apart in a real room: on 2026-09-04 the rover was pointed at a dining
table, and within a single frame six different chairs scored up to 0.913 against
each other -- higher than the same chair scored across the two lenses. Identity
by appearance is exactly what this component gave up on, for exactly this reason,
and a calibration that only works in rooms without repeated furniture is not one.

So the matching is on texture and geometry instead. The OAK's picture is warped
into the fisheye's own geometry first, so that the two images differ by a few
degrees of mount rather than by a lens; ORB matches them; and the solve is a
RANSAC PnP, which is the standard way to get a pose out of points whose
correspondences are partly wrong.

**What it prints goes into `oak.MOUNT` by hand, with the date.** It is not
written back automatically and should not be: a calibration that rewrites the
constant the whole component depends on, from whatever the rover happened to be
looking at, is a calibration nobody can review.

**It wants a textured room with things at a spread of distances.** Pointed at a
blank wall it finds nothing to match and says so. The report says which of the
rotation and the offset it could determine rather than printing a number for both
regardless.
"""
from __future__ import annotations

import argparse
import base64
import json
import math
import os
import socket
import statistics
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
# The package is imported as a package -- `~/ugv` on the rover, the checkout root
# here -- and `face_tracking/` goes on beside it because the gimbal camera's
# swept lens lives there. The same dance `bench_bearing.py` does.
for path in (ROOT, os.path.join(ROOT, "face_tracking")):
    if os.path.isdir(path) and path not in sys.path:
        sys.path.append(path)

from world_state import oak                                       # noqa: E402
from world_state.depth_client import SidecarRanger                # noqa: E402

#: Where the daemon listens, which is what parks the gimbal and hands over its
#: camera's picture. The camera has one owner and this is it.
DAEMON = ("127.0.0.1", 8769)
DAEMON_TIMEOUT_S = 30.0

#: How long to let the gimbal arrive before taking its picture, in seconds.
#: `usb_cameras/calibrate_fov.py` measured what happens with less -- a 25 degree
#: step that has not finished moving reads 20% short -- and this is a bench
#: script with nothing to hurry for.
SETTLE_S = 5.0

#: How many ORB features to look for in each picture. Generous: most will be on
#: the floor and the walls, most of those will be thrown out by the solve, and
#: what is wanted is enough left over on the furniture.
FEATURES = 2000
#: Lowe's ratio. A match whose second-best is nearly as good is a match on a
#: texture that repeats, which is most of a tiled floor.
RATIO = 0.75
#: How far a point may miss and still be counted, as a tangent -- the image
#: points here are directions rather than pixels, so this is an angle. Two
#: degrees, which is comfortably wider than the degree and a half a bearing on
#: this rover is believed to, and tight enough to exclude a mismatch.
INLIER_TAN = 0.035
#: The fewest inliers worth believing a solve from. Four points determine a pose;
#: below about this the answer is describing the noise on two of them.
MIN_INLIERS = 12
#: How big a patch of the depth map a feature's range is taken over, in colour
#: pixels either side. Small enough to be the thing the feature sits on, wide
#: enough to clear the twelve valid pixels the service needs to answer at all --
#: the depth map is half the colour frame's size, so this is a 12x12 box there.
DEPTH_PATCH_PX = 12
#: And how much range spread the fitted points need before the *offset* between
#: the two lenses is determined at all, as a ratio of furthest to nearest.
#: Everything at one distance is a single parallax reading, and any combination
#: of offset and rotation explains one of those.
MIN_RANGE_RATIO = 1.6
#: How coarse a grid the fisheye warp is built on before it is interpolated up.
#: The lens is smooth, so sampling every eighth pixel and resizing is well under
#: a pixel out -- and it means the map is built from `lens.ray_at` itself rather
#: than from a second copy of this camera's optics written out here in numpy.
WARP_STEP = 8


def call(name: str, arguments: dict | None = None) -> dict:
    """One control call on the daemon, or a dictionary saying why not."""
    request = json.dumps({"call": name, "arguments": arguments or {}})
    try:
        with socket.create_connection(DAEMON, DAEMON_TIMEOUT_S) as link:
            stream = link.makefile("rwb")
            stream.write(request.encode() + b"\n")
            stream.flush()
            return json.loads(stream.readline())
    except Exception as error:                                    # noqa: BLE001
        return {"ok": False, "error": f"{type(error).__name__}: {error}"}


def gimbal_frame(pan_deg: float) -> dict:
    """Park the gimbal and take one picture through it.

    The pan and tilt come back from the daemon rather than being assumed. What is
    read back is what it was *told*, so this rover's pan servo arriving about
    three degrees short at the ends of its travel is still in here -- which is
    why the report says which pans it used, and why staying inside +/-20 is worth
    doing.
    """
    aimed = call("look_at", {"pan": pan_deg, "tilt": 0})
    if not aimed.get("ok"):
        return {"ok": False, "error": str(aimed.get("error", "the gimbal refused"))}
    time.sleep(SETTLE_S)
    got = call("camera_jpeg")
    if not got.get("ok") or not got.get("jpeg_base64"):
        return {"ok": False, "error": str(got.get("error", "no picture"))}
    return {"ok": True, "jpeg": base64.b64decode(got["jpeg_base64"]),
            "pan": float(aimed.get("pan", pan_deg)),
            "tilt": float(aimed.get("tilt", 0.0)),
            "size": (int(got.get("width") or 640), int(got.get("height") or 480))}


# --- putting the two pictures in one geometry --------------------------------


def warp_maps(cv2, numpy, size, lens_oak):
    """Where each pixel of the fisheye's picture falls in the OAK's, as cv2 maps.

    **Warping the OAK into the fisheye and not the other way round, because that
    direction needs no inverse.** `lens.ray_at` answers "which way does this pixel
    look", which is exactly what is wanted here: for every pixel of the fisheye
    frame, take its direction and project it into the OAK's pinhole. Going the
    other way would mean inverting the fitted fisheye, which is a cubic and a
    second description of optics this repository deliberately keeps in one place.

    The mount is taken as zero while the map is built, which is the point: the two
    images then differ by *only* the few degrees of mount, at the same scale and
    in the same geometry, which is the easy case for a feature matcher. What the
    solve afterwards recovers is that difference.

    Built on every eighth pixel and interpolated up. The lens is smooth and the
    saving is real -- 4,800 calls into `lens.ray_at` rather than 307,200.
    """
    import lens as fitted                                         # noqa: PLC0415

    width, height = size
    optics = fitted.lens_for(width, height)
    xs = list(range(0, width, WARP_STEP)) + [width - 1]
    ys = list(range(0, height, WARP_STEP)) + [height - 1]
    coarse_x = numpy.zeros((len(ys), len(xs)), numpy.float32)
    coarse_y = numpy.zeros((len(ys), len(xs)), numpy.float32)
    for row, y in enumerate(ys):
        for column, x in enumerate(xs):
            dx, dy, dz = fitted.ray_at(x, y, optics)
            if dz <= 1e-6:
                # Behind the OAK's lens. Sent off the edge of its picture so
                # `cv2.remap` leaves it blank rather than wrapping it round.
                coarse_x[row, column] = -1.0
                coarse_y[row, column] = -1.0
                continue
            coarse_x[row, column] = lens_oak.cx + lens_oak.fx * dx / dz
            coarse_y[row, column] = lens_oak.cy + lens_oak.fy * dy / dz
    map_x = cv2.resize(coarse_x, (width, height), interpolation=cv2.INTER_LINEAR)
    map_y = cv2.resize(coarse_y, (width, height), interpolation=cv2.INTER_LINEAR)
    return map_x, map_y


def matched_points(cv2, gimbal_grey, warped_grey):
    """Feature pairs between the fisheye picture and the OAK's, warped to match.

    `[(gimbal_xy, warped_xy)]`. ORB rather than anything licensed, a ratio test
    against the second-best match because a tiled floor repeats, and a mutual
    check because a corner that resembles every other corner will otherwise be
    somebody's best match in one direction only. Whatever is left wrong after
    that is what the RANSAC in the solve is for.
    """
    orb = cv2.ORB_create(nfeatures=FEATURES)
    here_kp, here_desc = orb.detectAndCompute(gimbal_grey, None)
    there_kp, there_desc = orb.detectAndCompute(warped_grey, None)
    counts = (len(here_kp or []), len(there_kp or []))
    if here_desc is None or there_desc is None or len(here_kp) < 2 \
            or len(there_kp) < 2:
        return [], counts
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    best = {}
    for pair in matcher.knnMatch(here_desc, there_desc, k=2):
        if len(pair) < 2 or pair[0].distance >= RATIO * pair[1].distance:
            continue
        best[pair[0].queryIdx] = pair[0].trainIdx
    mutual = []
    for pair in matcher.knnMatch(there_desc, here_desc, k=2):
        if len(pair) < 2 or pair[0].distance >= RATIO * pair[1].distance:
            continue
        here_index = pair[0].trainIdx
        if best.get(here_index) == pair[0].queryIdx:
            mutual.append((here_kp[here_index].pt,
                           there_kp[pair[0].queryIdx].pt))
    return mutual, counts


def ranges_at(ranger, points, lens_oak):
    """How far away each of these points in the OAK's picture is, or None.

    One request for all of them: `/ranges` batches, and a feature is a small box
    round its own pixel. The service answers out along the ray rather than along
    the axis, which is what a point in space needs.
    """
    boxes = [[(x - DEPTH_PATCH_PX) / lens_oak.width,
              (y - DEPTH_PATCH_PX) / lens_oak.height,
              (x + DEPTH_PATCH_PX) / lens_oak.width,
              (y + DEPTH_PATCH_PX) / lens_oak.height] for x, y in points]
    answers, error = ranger.ranges(boxes)
    if error:
        return None, error
    return [one.range_m if one is not None else None for one in answers], ""


# --- the solve ----------------------------------------------------------------


def solve_pose(cv2, numpy, object_points, image_points):
    """Where the gimbal camera is, relative to the OAK, from points and rays.

    `solvePnPRansac` with the identity for a camera matrix and no distortion,
    because the image points handed to it are already directions rather than
    pixels: the fisheye is not a pinhole and cannot be given to a solver as one,
    but a pixel turned into a unit ray and divided through by its own forward
    component is exactly what a pinhole solver wants.

    What comes back is the pose of the *object* frame -- the OAK's -- in the
    camera's, which is the gimbal's. That is the mount, the right way round, with
    the offset in it rather than fitted separately afterwards.

    RANSAC first and then a refit on the inliers alone, because the pose RANSAC
    returns is the one the best random four points implied; the answer wanted is
    the one all of them agree on.
    """
    if len(object_points) < MIN_INLIERS:
        return None
    objects = numpy.array(object_points, numpy.float64).reshape(-1, 1, 3)
    images = numpy.array(image_points, numpy.float64).reshape(-1, 1, 2)
    identity, none = numpy.eye(3), numpy.zeros(5)
    ok, rvec, tvec, inliers = cv2.solvePnPRansac(
        objects, images, identity, none, reprojectionError=INLIER_TAN,
        iterationsCount=2000, confidence=0.999, flags=cv2.SOLVEPNP_EPNP)
    if not ok or inliers is None or len(inliers) < MIN_INLIERS:
        return None
    kept = [int(one) for one in inliers.ravel()]
    ok, rvec, tvec = cv2.solvePnP(
        objects[kept], images[kept], identity, none, rvec, tvec,
        useExtrinsicGuess=True, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    rotation, _ = cv2.Rodrigues(rvec)
    projected, _ = cv2.projectPoints(objects[kept], rvec, tvec, identity, none)
    residual = []
    for slot, index in enumerate(kept):
        dx = float(projected[slot][0][0]) - image_points[index][0]
        dy = float(projected[slot][0][1]) - image_points[index][1]
        residual.append(math.degrees(math.atan(math.hypot(dx, dy))))
    return rotation, tvec.reshape(3), kept, residual


def chassis_from_optical(numpy, pan_deg: float, tilt_deg: float):
    """The matrix taking a direction in the gimbal camera's optical frame to the
    rover's own: x forward, y left, z up.

    The camera's axes are x right, y down, z out of the lens. Undoing the tilt
    first and then the pan is the same order `view._levelled` and
    `view.chassis_direction` use, and the pan turns the opposite way because the
    gimbal counts it positive to the right while everything on the map counts
    positive to the left.
    """
    tilt = math.radians(tilt_deg or 0.0)
    levelled = numpy.array([[0.0, math.sin(tilt), math.cos(tilt)],
                            [-1.0, 0.0, 0.0],
                            [0.0, -math.cos(tilt), math.sin(tilt)]])
    yaw = math.radians(-(pan_deg or 0.0))
    turn = numpy.array([[math.cos(yaw), -math.sin(yaw), 0.0],
                        [math.sin(yaw), math.cos(yaw), 0.0],
                        [0.0, 0.0, 1.0]])
    return turn @ levelled


def mount_from(numpy, rotation, translation, pan_deg, tilt_deg):
    """One solve, as the numbers `oak.Mount` holds.

    `rotation` and `translation` put a point in the OAK's optical frame into the
    gimbal camera's. The mount wants both in the rover's frame instead, and wants
    the rotation the other way round -- `oak._in_oak` turns chassis directions
    into the OAK's, so what it needs is the transpose.
    """
    to_chassis = chassis_from_optical(numpy, pan_deg, tilt_deg)
    into_oak = (to_chassis @ rotation).T
    pitch = math.asin(max(-1.0, min(1.0, float(into_oak[2][2]))))
    yaw = math.atan2(float(into_oak[2][1]), float(into_oak[2][0]))
    roll = math.atan2(float(into_oak[0][2]), -float(into_oak[1][2]))
    offset = to_chassis @ numpy.asarray(translation)
    return {"yaw_deg": -math.degrees(yaw), "pitch_deg": math.degrees(pitch),
            "roll_deg": math.degrees(roll), "forward_m": float(offset[0]),
            "left_m": float(offset[1]), "up_m": float(offset[2])}


# --- one position, end to end -------------------------------------------------


def at_pan(cv2, numpy, pan, ranger, lens_oak, maps, save):
    """One gimbal position, as a solved mount or as a sentence saying why not."""
    import lens as fitted                                         # noqa: PLC0415

    frame = gimbal_frame(pan)
    if not frame["ok"]:
        return None, f"pan {pan:+.0f}: {frame['error']}"
    theirs = ranger.frame()
    if not theirs.ok:
        return None, f"pan {pan:+.0f}: {theirs.error}"

    gimbal = cv2.imdecode(numpy.frombuffer(frame["jpeg"], numpy.uint8),
                          cv2.IMREAD_GRAYSCALE)
    picture = cv2.imdecode(numpy.frombuffer(theirs.jpeg, numpy.uint8),
                           cv2.IMREAD_GRAYSCALE)
    if gimbal is None or picture is None:
        return None, f"pan {pan:+.0f}: a picture would not decode"
    warped = cv2.remap(picture, maps[0], maps[1], cv2.INTER_LINEAR,
                       borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    if save:
        base = f"{save}-pan{int(round(pan)):+03d}"
        cv2.imwrite(base + "-gimbal.png", gimbal)
        cv2.imwrite(base + "-oak-warped.png", warped)

    pairs, counts = matched_points(cv2, gimbal, warped)
    if not pairs:
        return None, (f"pan {pan:+.0f}: {counts[0]} and {counts[1]} features, "
                      f"none matched")

    # Back out of the warp: the map says which OAK pixel each warped pixel came
    # from, so reading it at the match is the real pixel to ask for a range.
    optics = fitted.lens_for(*frame["size"])
    in_oak, rays = [], []
    for (gx, gy), (wx, wy) in pairs:
        column = int(round(min(max(wx, 0.0), maps[0].shape[1] - 1)))
        row = int(round(min(max(wy, 0.0), maps[0].shape[0] - 1)))
        ox, oy = float(maps[0][row, column]), float(maps[1][row, column])
        if not (0 <= ox < lens_oak.width and 0 <= oy < lens_oak.height):
            continue
        dx, dy, dz = fitted.ray_at(gx, gy, optics)
        if dz <= 1e-3:
            continue
        in_oak.append((ox, oy))
        rays.append((dx / dz, dy / dz))
    if len(in_oak) < MIN_INLIERS:
        return None, (f"pan {pan:+.0f}: {len(pairs)} matched, only "
                      f"{len(in_oak)} of them usable")

    measured, error = ranges_at(ranger, in_oak, lens_oak)
    if measured is None:
        return None, f"pan {pan:+.0f}: no ranges ({error})"

    object_points, image_points, ranged = [], [], []
    for (ox, oy), ray, range_m in zip(in_oak, rays, measured):
        if not range_m:
            continue
        direction = oak.ray_at(ox / lens_oak.width, oy / lens_oak.height, lens_oak)
        object_points.append([one * range_m for one in direction])
        image_points.append(list(ray))
        ranged.append(range_m)
    if len(object_points) < MIN_INLIERS:
        return None, (f"pan {pan:+.0f}: {len(pairs)} matched, only "
                      f"{len(object_points)} had a range")

    solved = solve_pose(cv2, numpy, object_points, image_points)
    if solved is None:
        return None, (f"pan {pan:+.0f}: {len(object_points)} ranged points, "
                      f"no pose survived them")
    rotation, translation, kept, residual = solved
    found = mount_from(numpy, rotation, translation, frame["pan"], frame["tilt"])
    found.update(inliers=len(kept), points=len(object_points),
                 residual_deg=statistics.median(residual),
                 worst_deg=max(residual),
                 ranges=[ranged[i] for i in kept])
    return found, (f"pan {pan:+.0f}: {counts[0]}/{counts[1]} features, "
                   f"{len(pairs)} matched, {len(object_points)} ranged, "
                   f"{len(kept)} fitted")


# --- what it all came to ------------------------------------------------------


def report(found: list, notes: list) -> int:
    for line in notes:
        print("  " + line)
    print()
    if not found:
        print("nothing was solved.")
        print("Point the rover at a textured room with things two to four metres "
              "off and try again -- a blank wall has nothing to match, and a "
              "plain floor matches itself everywhere.")
        return 1

    def spread(key):
        values = [one[key] for one in found]
        return (statistics.median(values),
                (max(values) - min(values)) if len(values) > 1 else 0.0)

    print(f"{len(found)} position(s) solved, "
          f"{sum(one['inliers'] for one in found)} points fitted in all")
    print()
    print("                     median    spread over the positions")
    for key, label, unit in (("yaw_deg", "yaw", "deg"),
                             ("pitch_deg", "pitch", "deg"),
                             ("roll_deg", "roll", "deg"),
                             ("forward_m", "forward", "m"),
                             ("left_m", "left", "m"),
                             ("up_m", "up", "m")):
        middle, apart = spread(key)
        print(f"  {label:<8s} {middle:+10.3f} {unit:<4s} {apart:9.3f}")
    print()
    residuals = [one["residual_deg"] for one in found]
    print(f"  what is left over: median {statistics.median(residuals):.2f} deg, "
          f"worst {max(one['worst_deg'] for one in found):.2f}")
    print(f"  (a bearing on this rover is believed to {_bearing_sigma():.1f} deg, "
          f"which is what that has to beat)")
    ranged = [one for solved in found for one in solved["ranges"]]
    if ranged:
        apart = max(ranged) / max(min(ranged), 0.01)
        print(f"  fitted points {min(ranged):.2f} to {max(ranged):.2f} m out, "
              f"a spread of {apart:.1f}x")
        if apart < MIN_RANGE_RATIO:
            print("  **the offset is not determined**: everything fitted was at "
                  "much the same distance, so the parallax cannot tell an offset "
                  "from a rotation. Take the rotation, and either find a scene "
                  "with near and far things in it or measure the offset with a "
                  "ruler -- two brackets on one chassis, and a ruler is an honest "
                  "instrument there.")
    print()
    _leftover(spread)
    return 0


def _bearing_sigma() -> float:
    try:
        from world_state import locate

        return float(locate.BEARING_SIGMA_DEG)
    except Exception:                                             # noqa: BLE001
        return 1.5


def _leftover(spread) -> None:
    """The block to paste into `oak.py`, so nobody retypes a number."""
    print("to adopt this, put it in oak.py and set MEASURED = True:")
    print()
    print("    MOUNT = Mount(")
    print(f"        yaw_deg={spread('yaw_deg')[0]:.2f},")
    print(f"        pitch_deg={spread('pitch_deg')[0]:.2f},")
    print(f"        forward_m={spread('forward_m')[0]:.3f},")
    print(f"        left_m={spread('left_m')[0]:.3f},")
    print(f"        up_m={spread('up_m')[0]:.3f},")
    print("    )")
    print("    MEASURED = True")
    print()
    roll = spread("roll_deg")[0]
    if abs(roll) > 1.0:
        print(f"and note the roll of {roll:+.2f} deg, which oak.Mount does not "
              f"carry. Above a degree it is worth adding rather than ignoring.")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--pan", type=float, nargs="+", default=[0.0],
                        help="gimbal pan positions to solve at (default: just "
                             "0). Several is a check on the answer rather than "
                             "more of it -- each is solved on its own, and the "
                             "report prints how far apart they came out")
    parser.add_argument("--save", metavar="PREFIX",
                        help="write both pictures per position, the OAK's warped "
                             "into the fisheye's geometry, so a person can see "
                             "what was matched")
    args = parser.parse_args()

    print("bench_oak -- where the OAK is, against the camera the rover trusts")
    print()
    try:
        import cv2
        import numpy
    except ImportError as error:
        print(f"this needs OpenCV and numpy on the host that runs it: {error}")
        return 1

    ranger = SidecarRanger()
    lens_oak = ranger.lens()
    if lens_oak is None:
        print("the depth camera would not say what lens it has; is oak_depth "
              "running, and is it a build that serves the colour half?")
        return 1

    first = gimbal_frame(args.pan[0])
    if not first["ok"]:
        print(f"the gimbal camera would not answer: {first['error']}")
        return 1
    maps = warp_maps(cv2, numpy, first["size"], lens_oak)

    found, notes = [], []
    for pan in args.pan:
        one, note = at_pan(cv2, numpy, pan, ranger, lens_oak, maps, args.save)
        notes.append(note)
        if one is not None:
            found.append(one)
    return report(found, notes)


if __name__ == "__main__":
    raise SystemExit(main())
