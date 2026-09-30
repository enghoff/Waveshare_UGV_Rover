#!/usr/bin/env python3
"""Which way the OAK points, measured against the camera the rover already trusts.

    ssh orin 'cd ~/ugv/world_state && python3 bench_oak.py'
    ssh orin 'cd ~/ugv/world_state && python3 bench_oak.py --offset 0 0 0.05'
    ssh orin 'cd ~/ugv/world_state && python3 bench_oak.py --pan -30 0 30 --tilt 0 20'

The rover has two cameras that see the room and they had never been in the same
frame. The gimbal camera's optics were swept and fitted on this rover and every
bearing the world state holds was drawn through them; the OAK rides the gimbal's
rail beside it and is the only thing that knows how far away anything is. Before
either could check the other's work, somebody had to say **which way the OAK
points** relative to the gimbal camera. That is what this measures.

**Every position should give the same answer now, and that is the check.** The
OAK was bolted to the chassis until 2026-09-30, and what this bench fitted then
moved by 4.5 degrees of yaw depending on where the gimbal was pointed -- the
gimbal's own pointing faults, read as a property of the mount. On the rail the
two cameras turn together, so the rotation between them is one number whatever
the servos did, and positions that disagree mean the mount is not rigid or a
lens model is wrong. `--pan` and `--tilt` exist to ask that question.

**It measures the rotation and it does not measure the offset**, and that
division is the whole design rather than a shortcoming admitted afterwards. Two
lenses a few centimetres apart, looking at a room metres away, see it in almost
the same direction: five centimetres at three metres is one degree of parallax,
which is the size of what the fit leaves over anyway. So a solver handed both at
once will spend the offset absorbing anything else that is systematic -- and on
this rover it did, claiming half a metre between two cameras that are a few
centimetres apart, and fitting the data *better* for it. **A ruler measures the
offset far better than this can; this measures the rotation far better than a
ruler can.** Give it the offset with `--offset` and it solves the rest.

**No target and no vocabulary.** Both cameras look at whatever room the rover is
in, the OAK's picture is warped into the fisheye's own geometry so that the two
differ by a few degrees of mount rather than by a lens, ORB matches them, the
depth camera ranges every match, and the rotation is the one that lines the two
sets of directions up -- re-selecting its own inliers as it goes, because a room
of repeated chair slats produces plenty of matches that are simply wrong.

**Matching by appearance was tried first and does not work here, measured.**
Matching the two pictures' *regions* by the encoder vectors the resolver uses
gave two usable pairs out of ten on 2026-09-04, because within a single frame six
different dining chairs scored up to 0.913 against each other -- higher than the
same chair scored across the two lenses. Identity by appearance is what this
component gave up on, for this reason.

**What it prints goes into `oak.MOUNT` by hand, with the date.** A calibration
that rewrites the constant the whole component depends on, from whatever the
rover happened to be looking at, is a calibration nobody can review.

**It wants a textured room with things in it.** A blank wall has nothing to
match and a plain floor matches itself everywhere; it says so rather than
fitting whatever it was given.
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

#: How many ORB features to look for in each picture. Generous: most land on the
#: floor and the walls, most of those are thrown out, and what is wanted is
#: enough left over on the furniture.
FEATURES = 2000
#: Lowe's ratio. A match whose second-best is nearly as good is a match on a
#: texture that repeats, which is most of a tiled floor.
RATIO = 0.75
#: How far a point may miss the fit and still be counted, in degrees.
#:
#: **This is doing outlier rejection, not error budgeting.** On the dining-table
#: scene of 2026-09-04 only 46 to 70 of 220 matched points landed inside it, and
#: the rest are matches that are simply wrong -- one chair slat taken for the
#: next. Two degrees is comfortably wider than what a right match misses by
#: (0.3 to 0.9) and far tighter than what a wrong one does.
INLIER_DEG = 2.0
#: The fewest inliers worth believing a fit from.
MIN_INLIERS = 12
#: How big a patch of the depth map a feature's range is taken over, in colour
#: pixels either side. Small enough to be the thing the feature sits on, wide
#: enough to clear the twelve valid pixels the service needs to answer at all --
#: the depth map is half the colour frame's size, so this is a 12x12 box there.
DEPTH_PATCH_PX = 12
#: How coarse a grid the fisheye warp is built on before it is interpolated up.
#: The lens is smooth, so sampling every eighth pixel is well under a pixel out --
#: and it means the map is built from `lens.ray_at` itself rather than from a
#: second copy of this camera's optics written out here in numpy.
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


def gimbal_frame(pan_deg: float, tilt_deg: float = 0.0) -> dict:
    """Park the gimbal and take one picture through it.

    The pan and tilt come back from the daemon rather than being assumed. What is
    read back is what it was *told*, so the servos' own errors are still in here
    -- which no longer reaches the answer, because both cameras carry the same
    error, and is why the report still says where each position was.
    """
    aimed = call("look_at", {"pan": pan_deg, "tilt": tilt_deg})
    if not aimed.get("ok"):
        return {"ok": False, "error": str(aimed.get("error", "the gimbal refused"))}
    time.sleep(SETTLE_S)
    got = call("camera_jpeg")
    if not got.get("ok") or not got.get("jpeg_base64"):
        return {"ok": False, "error": str(got.get("error", "no picture"))}
    return {"ok": True, "jpeg": base64.b64decode(got["jpeg_base64"]),
            "pan": float(aimed.get("pan", pan_deg)),
            "tilt": float(aimed.get("tilt", tilt_deg)),
            "size": (int(got.get("width") or 640), int(got.get("height") or 480))}


# --- putting the two pictures in one geometry --------------------------------


def warp_maps(numpy, size, lens_oak):
    """Where each pixel of the fisheye's picture falls in the OAK's, as cv2 maps.

    **Warping the OAK into the fisheye and not the other way round, because that
    direction needs no inverse.** `lens.ray_at` answers "which way does this pixel
    look", which is exactly what is wanted: for every pixel of the fisheye frame,
    take its direction and project it into the OAK's pinhole. Going the other way
    would mean inverting the fitted fisheye, which is a cubic and a second
    description of optics this repository keeps in one place.

    The mount is taken as zero while the map is built, which is the point: the two
    images then differ by *only* the few degrees of mount, at the same scale and
    in the same geometry, which is the easy case for a feature matcher.

    Built on every eighth pixel and interpolated up -- 4,800 calls into
    `lens.ray_at` rather than 307,200. **Interpolated by hand rather than with
    `cv2.resize`, which was a bug**: resize maps cell *centres*, so a coarse grid
    sampled every eighth pixel comes back shifted by about three pixels and
    scaled by a percent. Three pixels is 0.4 degrees at this focal length, which
    is the same size as what the fit leaves over.
    """
    import lens as fitted                                         # noqa: PLC0415

    width, height = size
    optics = fitted.lens_for(width, height)
    xs = numpy.arange(0, width, WARP_STEP, dtype=numpy.float64)
    if xs[-1] != width - 1:
        xs = numpy.append(xs, width - 1)
    ys = numpy.arange(0, height, WARP_STEP, dtype=numpy.float64)
    if ys[-1] != height - 1:
        ys = numpy.append(ys, height - 1)
    coarse = [numpy.zeros((len(ys), len(xs))), numpy.zeros((len(ys), len(xs)))]
    for row, y in enumerate(ys):
        for column, x in enumerate(xs):
            dx, dy, dz = fitted.ray_at(float(x), float(y), optics)
            if dz <= 1e-6:
                # Behind the OAK's lens. Sent off the edge of its picture so
                # `cv2.remap` leaves it blank rather than wrapping it round.
                coarse[0][row, column] = coarse[1][row, column] = -1.0
                continue
            coarse[0][row, column] = lens_oak.cx + lens_oak.fx * dx / dz
            coarse[1][row, column] = lens_oak.cy + lens_oak.fy * dy / dz
    full_x = numpy.arange(width, dtype=numpy.float64)
    full_y = numpy.arange(height, dtype=numpy.float64)
    maps = []
    for grid in coarse:
        along = numpy.vstack([numpy.interp(full_x, xs, row) for row in grid])
        both = numpy.vstack([numpy.interp(full_y, ys, along[:, one])
                             for one in range(width)]).T
        maps.append(both.astype(numpy.float32))
    return maps


def matched_points(cv2, gimbal_grey, warped_grey):
    """Feature pairs between the fisheye picture and the OAK's, warped to match.

    `[(gimbal_xy, warped_xy)]`. ORB rather than anything licensed, a ratio test
    against the second-best match because a tiled floor repeats, and a mutual
    check because a corner that resembles every other corner will otherwise be
    somebody's best match in one direction only. Plenty of what survives that is
    still wrong -- one chair slat taken for the next -- which is what the fit's
    own inlier selection is for.
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


# --- the fit ------------------------------------------------------------------


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


def rotation_for(numpy, objects, images, offset, rounds=8):
    """The rotation between the two cameras, with the offset taken as known.

    Wahba's problem each round -- the rotation lining up two sets of unit
    directions is the singular value decomposition of their outer product -- and
    then the points within `INLIER_DEG` of that answer are kept and it is done
    again. Starting from everything and tightening rather than starting from a
    random four: the rotation is well enough determined that it does not need a
    random start, and what has to be excluded is a third of the matches rather
    than a few.

    The offset enters by moving each of the gimbal camera's rays to where the OAK
    stands, at the range the OAK measured, which is what makes the two sets of
    directions comparable at all. With the offset at nothing this is the plain
    two-set alignment; with it at a few centimetres it is the same thing done
    honestly.
    """
    into_optical = chassis_from_optical(numpy, 0.0, 0.0).T
    shift = into_optical @ numpy.array(offset, dtype=numpy.float64)
    from_oak = numpy.array(objects, dtype=numpy.float64)
    unit_oak = from_oak / numpy.linalg.norm(from_oak, axis=1, keepdims=True)
    rays = numpy.array([[x, y, 1.0] for x, y in images], dtype=numpy.float64)
    rays /= numpy.linalg.norm(rays, axis=1, keepdims=True)

    rotation = numpy.eye(3)
    keep = numpy.ones(len(objects), dtype=bool)
    miss = numpy.zeros(len(objects))
    for _round in range(rounds):
        if int(keep.sum()) < MIN_INLIERS:
            return None
        reach = numpy.linalg.norm((rotation @ from_oak.T).T + shift,
                                  axis=1, keepdims=True)
        moved = rays * reach - shift
        moved /= numpy.linalg.norm(moved, axis=1, keepdims=True)
        u, _s, vt = numpy.linalg.svd(moved[keep].T @ unit_oak[keep])
        middle = numpy.eye(3)
        middle[2, 2] = numpy.sign(numpy.linalg.det(u @ vt))
        rotation = u @ middle @ vt
        predicted = (rotation @ from_oak.T).T + shift
        predicted /= numpy.linalg.norm(predicted, axis=1, keepdims=True)
        miss = numpy.degrees(numpy.arccos(numpy.clip(
            numpy.sum(predicted * rays, axis=1), -1.0, 1.0)))
        keep = miss <= INLIER_DEG
    if int(keep.sum()) < MIN_INLIERS:
        return None
    return rotation, miss, keep


def angles_of(numpy, rotation, pan_deg, tilt_deg):
    """A rotation as the mount's yaw, pitch and roll, in degrees.

    `rotation` takes a direction in the OAK's optical frame to the gimbal
    camera's. `oak._in_oak` goes the other way and works in the gimbal camera's
    own frame as it stands at pan 0 and tilt 0, so what it needs is the transpose
    of the two put together -- and since the OAK rides the gimbal, 0 and 0 is
    what every caller passes whatever the gimbal was doing. The arguments stay so
    that a recording made while the OAK was on the chassis can still be read.
    """
    into_oak = (chassis_from_optical(numpy, pan_deg, tilt_deg) @ rotation).T
    pitch = math.asin(max(-1.0, min(1.0, float(into_oak[2][2]))))
    yaw = math.atan2(float(into_oak[2][1]), float(into_oak[2][0]))
    roll = math.atan2(float(into_oak[0][2]), -float(into_oak[1][2]))
    return {"yaw_deg": -math.degrees(yaw), "pitch_deg": math.degrees(pitch),
            "roll_deg": math.degrees(roll)}


def free_offset(cv2, numpy, objects, images):
    """What a solver claims the offset is when nobody tells it. A warning, not a
    measurement -- see the module docstring.

    Kept because it is the evidence for the docstring's claim rather than a
    restatement of it: on this rover it comes back with half a metre between two
    cameras a few centimetres apart, and the report prints that so nobody has to
    take the warning on trust.
    """
    if len(objects) < MIN_INLIERS:
        return None
    got = cv2.solvePnPRansac(
        numpy.array(objects, numpy.float64).reshape(-1, 1, 3),
        numpy.array(images, numpy.float64).reshape(-1, 1, 2),
        numpy.eye(3), numpy.zeros(5),
        reprojectionError=math.tan(math.radians(INLIER_DEG)),
        iterationsCount=2000, confidence=0.999, flags=cv2.SOLVEPNP_EPNP)
    ok, _rvec, tvec, inliers = got
    if not ok or inliers is None or len(inliers) < MIN_INLIERS:
        return None
    return chassis_from_optical(numpy, 0.0, 0.0) @ tvec.reshape(3)


# --- every position at once -----------------------------------------------------


def joint_fit(numpy, kept, forward_m=0.0, resamples=200):
    """One mount for every collected position, with the fisheye's scale free.

    **Why the scale has to be free, measured on 2026-09-30.** With the OAK on
    the rail every position ought to agree, and with the rotation alone they did
    not; with the offset free as well the fit claimed 23 cm of forward reach
    between two lenses set in one plane. What closed it, to a median of 0.24
    degrees, was letting the fisheye's angles off its axis stretch -- by 7.2%,
    which was `face_tracking/lens.py` being short and not anything about the
    mount, and which the lens refitted the same day brought to 1.012 (the rest
    is the OAK drawn as a pinhole). So the fit here is the rotation, the left
    and up offset and that scale, with the forward offset held where the owner
    set it: forward and the scale are the one pair these points cannot tell
    apart. A scale well away from 1 means the fisheye wants measuring again.

    `(answer, per_position)`, or `(None, [])` if nothing survives. The answer
    carries the angles and offsets in `Mount`'s terms, the scale, the median
    miss, and 95% intervals from `resamples` resamplings of the points.
    """
    from scipy.optimize import least_squares                      # noqa: PLC0415
    from scipy.spatial.transform import Rotation                  # noqa: PLC0415

    objects = numpy.vstack([numpy.array(one["objects"]) for one in kept])
    rays = numpy.vstack([numpy.array([[x, y, 1.0] for x, y in one["images"]])
                         for one in kept])
    rays /= numpy.linalg.norm(rays, axis=1, keepdims=True)
    where = numpy.concatenate([[index] * len(one["objects"])
                               for index, one in enumerate(kept)])
    # The optical frame is x right, y down, z out; the forward offset is z.
    held = float(forward_m)

    def stretched(k):
        theta = numpy.arccos(numpy.clip(rays[:, 2], -1.0, 1.0)) * k
        around = numpy.arctan2(rays[:, 1], rays[:, 0])
        return numpy.stack([numpy.sin(theta) * numpy.cos(around),
                            numpy.sin(theta) * numpy.sin(around),
                            numpy.cos(theta)], axis=1)

    def misses(x, pick):
        moved = Rotation.from_rotvec(x[:3]).apply(objects[pick]) \
            + numpy.array([x[3], x[4], held])
        moved /= numpy.linalg.norm(moved, axis=1, keepdims=True)
        return numpy.degrees(numpy.cross(moved, stretched(x[5])[pick]))

    def solve(pick, rotation_only=None):
        x = numpy.array([0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        if rotation_only is not None:
            x = rotation_only.copy()
        keep = numpy.ones(len(pick), dtype=bool)
        free = slice(0, 3) if rotation_only is not None else slice(0, 6)
        for _round in range(5):
            if int(keep.sum()) < MIN_INLIERS:
                return None, None, keep
            chosen = pick[keep]

            def residual(z):
                full = x.copy()
                full[free] = z
                return misses(full, chosen).ravel()

            x[free] = least_squares(residual, x[free], loss="soft_l1",
                                    f_scale=0.5).x
            miss = numpy.linalg.norm(misses(x, pick), axis=1)
            keep = miss <= INLIER_DEG
        return x, miss, keep

    everything = numpy.arange(len(objects))
    x, miss, keep = solve(everything)
    if x is None:
        return None, []

    def described(z):
        angles = angles_of(numpy, Rotation.from_rotvec(z[:3]).as_matrix(),
                           0.0, 0.0)
        return [angles["yaw_deg"], angles["pitch_deg"], angles["roll_deg"],
                held, -z[3], -z[4], z[5]]

    names = ("yaw_deg", "pitch_deg", "roll_deg", "forward_m", "left_m", "up_m",
             "fisheye_scale")
    answer = dict(zip(names, described(x)))
    answer.update(miss_deg=float(numpy.median(miss[keep])),
                  points=int(keep.sum()), of=len(objects))
    chance = numpy.random.default_rng(1)
    again = []
    for _round in range(resamples):
        z, _m, _k = solve(chance.integers(0, len(objects), len(objects)))
        if z is not None:
            again.append(described(z))
    if again:
        low, high = numpy.percentile(numpy.array(again), [2.5, 97.5], axis=0)
        answer["interval"] = {name: (float(a), float(b))
                              for name, a, b in zip(names, low, high)}

    per_position = []
    for index, one in enumerate(kept):
        z, m, k = solve(numpy.flatnonzero(where == index), rotation_only=x)
        if z is None:
            continue
        found = dict(zip(names[:3], described(z)[:3]))
        found.update(pan_deg=one["pan_deg"], tilt_deg=one.get("tilt_deg", 0.0),
                     points=int(k.sum()), miss_deg=float(numpy.median(m[k])))
        per_position.append(found)
    return answer, per_position


def report_joint(answer, per_position) -> int:
    if answer is None:
        print("the joint fit found too few points that agree")
        return 1
    print(f"one mount for every position, the fisheye's scale free, forward held "
          f"at {answer['forward_m']:+.3f} m: {answer['points']} of {answer['of']} "
          f"points, median miss {answer['miss_deg']:.2f} deg")
    interval = answer.get("interval", {})
    for name in ("yaw_deg", "pitch_deg", "roll_deg", "left_m", "up_m",
                 "fisheye_scale"):
        low, high = interval.get(name, (float("nan"), float("nan")))
        print(f"  {name:<14s} {answer[name]:+8.4f}   95% {low:+.4f} to {high:+.4f}")
    print()
    print("each position on its own, offsets and scale held")
    print("    pan  tilt     yaw    pitch     roll   pts   miss")
    for one in per_position:
        print(f"  {one['pan_deg']:+5.0f} {one['tilt_deg']:+5.0f}  "
              f"{one['yaw_deg']:+6.2f}  {one['pitch_deg']:+6.2f}  "
              f"{one['roll_deg']:+6.2f}  {one['points']:4d}  "
              f"{one['miss_deg']:5.2f}")
    print()
    print("a scale away from 1 is the fisheye's lens model, not the mount -- see "
          "world_state/oak.py")
    return 0


# --- one position, end to end -------------------------------------------------


def collect_at(cv2, numpy, pan, tilt, ranger, lens_oak, maps, save):
    """One gimbal position, as points to solve from, or a sentence saying why not.

    Kept apart from the solve because the offset the solve needs is measured with
    a ruler rather than from these pictures, and by the time somebody has held one
    up the rover has usually been moved. What comes back is plain lists, so it
    survives being written to a file and read again.
    """
    import lens as fitted                                         # noqa: PLC0415

    where = f"pan {pan:+.0f} tilt {tilt:+.0f}"
    frame = gimbal_frame(pan, tilt)
    if not frame["ok"]:
        return None, f"{where}: {frame['error']}"
    theirs = ranger.frame()
    if not theirs.ok:
        return None, f"{where}: {theirs.error}"

    gimbal = cv2.imdecode(numpy.frombuffer(frame["jpeg"], numpy.uint8),
                          cv2.IMREAD_GRAYSCALE)
    picture = cv2.imdecode(numpy.frombuffer(theirs.jpeg, numpy.uint8),
                           cv2.IMREAD_GRAYSCALE)
    if gimbal is None or picture is None:
        return None, f"{where}: a picture would not decode"
    warped = cv2.remap(picture, maps[0], maps[1], cv2.INTER_LINEAR,
                       borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    if save:
        base = f"{save}-pan{int(round(pan)):+03d}-tilt{int(round(tilt)):+03d}"
        cv2.imwrite(base + "-gimbal.png", gimbal)
        cv2.imwrite(base + "-oak-warped.png", warped)

    pairs, counts = matched_points(cv2, gimbal, warped)
    if not pairs:
        return None, (f"{where}: {counts[0]} and {counts[1]} features, "
                      f"none matched")

    # Back out of the warp: the map says which OAK pixel each warped pixel came
    # from, so reading it at the match is the real pixel to ask for a range.
    optics = fitted.lens_for(*frame["size"])
    in_oak, rays, seen_at = [], [], []
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
        seen_at.append((gx, gy))
    if len(in_oak) < MIN_INLIERS:
        return None, (f"{where}: {len(pairs)} matched, only "
                      f"{len(in_oak)} of them usable")

    measured, error = ranges_at(ranger, in_oak, lens_oak)
    if measured is None:
        return None, f"{where}: no ranges ({error})"

    objects, images, ranged, pixels = [], [], [], []
    for (ox, oy), ray, range_m, pixel in zip(in_oak, rays, measured, seen_at):
        if not range_m:
            continue
        # The fisheye pixel as well as its ray, so that the points can be
        # re-rayed when the lens is refitted -- which on 2026-09-30 meant
        # inverting the old lens to get them back.
        pixels.append([round(pixel[0], 3), round(pixel[1], 3)])
        # **The lens alone and not the mount.** `oak.ray_at` takes the adopted
        # roll out, which is right for anything drawing a bearing and wrong
        # here: this would then measure how far the mount has moved since
        # somebody last wrote a number down, and print it as if it were the
        # mount. See `oak.pinhole_at`.
        direction = oak.pinhole_at(ox / lens_oak.width, oy / lens_oak.height,
                                   lens_oak)
        objects.append([one * range_m for one in direction])
        images.append(list(ray))
        ranged.append(range_m)
    if len(objects) < MIN_INLIERS:
        return None, (f"{where}: {len(pairs)} matched, only "
                      f"{len(objects)} had a range")

    return ({"objects": objects, "images": images, "ranges": ranged,
             "pixels": pixels,
             "pan_deg": frame["pan"], "tilt_deg": frame["tilt"]},
            f"{where}: {counts[0]}/{counts[1]} features, "
            f"{len(pairs)} matched, {len(objects)} ranged")


def solve_at(numpy, kept, offset):
    """The rotation for one collected position, at this offset, or None."""
    solved = rotation_for(numpy, kept["objects"], kept["images"], offset)
    if solved is None:
        return None
    rotation, miss, keep = solved
    # In the gimbal camera's own frame and not the rover's, whatever the gimbal
    # was doing: the OAK turns with it, so the pan and tilt are where the pair
    # was pointed and not part of the mount. See `angles_of`.
    found = angles_of(numpy, rotation, 0.0, 0.0)
    found.update(inliers=int(keep.sum()), points=len(kept["objects"]),
                 pan_deg=kept["pan_deg"], tilt_deg=kept.get("tilt_deg", 0.0),
                 miss_deg=float(numpy.median(miss[keep])),
                 worst_deg=float(numpy.max(miss[keep])),
                 free=None,
                 ranges=[r for r, k in zip(kept["ranges"], keep) if k])
    return found


# --- what it all came to ------------------------------------------------------


def report(numpy, found: list, notes: list, offset) -> int:
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
          f"{sum(one['inliers'] for one in found)} points fitted of "
          f"{sum(one['points'] for one in found)} ranged")
    print(f"offset taken as given: {offset[0]:+.3f} forward, {offset[1]:+.3f} "
          f"left, {offset[2]:+.3f} up")
    print()
    print("what each position said on its own")
    print("    pan  tilt     yaw    pitch     roll   pts   miss  worst")
    for one in found:
        print(f"  {one['pan_deg']:+5.0f} {one['tilt_deg']:+5.0f}  "
              f"{one['yaw_deg']:+6.2f}  "
              f"{one['pitch_deg']:+6.2f}  {one['roll_deg']:+6.2f}  "
              f"{one['inliers']:4d}  {one['miss_deg']:5.2f}  "
              f"{one['worst_deg']:5.2f}")
    print()
    for key, label in (("yaw_deg", "yaw"), ("pitch_deg", "pitch"),
                       ("roll_deg", "roll")):
        middle, apart = spread(key)
        print(f"  {label:<6s} {middle:+7.2f} deg   spread {apart:5.2f}")
    misses = [one["miss_deg"] for one in found]
    print(f"  what is left over: median {statistics.median(misses):.2f} deg, "
          f"worst {max(one['worst_deg'] for one in found):.2f}")
    print(f"  (a bearing on this rover is believed to {_bearing_sigma():.1f} deg, "
          f"which is what that has to beat)")
    ranged = [one for solved in found for one in solved["ranges"]]
    if ranged:
        print(f"  fitted points {min(ranged):.2f} to {max(ranged):.2f} m out")
    print()
    _about_the_offset(numpy, found)
    _leftover(spread, offset)
    return 0


def _about_the_offset(numpy, found) -> None:
    """What a solver says the offset is, and why it is not in the answer."""
    claims = [one["free"] for one in found if one["free"] is not None]
    if not claims:
        return
    middle = numpy.median(numpy.vstack(claims), axis=0)
    print(f"  a solver left to choose the offset for itself says "
          f"{middle[0]:+.3f} forward, {middle[1]:+.3f} left, {middle[2]:+.3f} up")
    print("  -- which is printed as a warning and not used. Two lenses a few")
    print("  centimetres apart looking at a room metres away see it in almost the")
    print("  same direction, so the offset is the part of this pose the data")
    print("  barely constrains, and a solver will spend it absorbing anything")
    print("  else that is systematic. On this rover it claimed half a metre")
    print("  between two cameras that are a few centimetres apart. Measure the")
    print("  offset with a ruler and pass it in with --offset.")
    print()


def _bearing_sigma() -> float:
    try:
        from world_state import locate

        return float(locate.BEARING_SIGMA_DEG)
    except Exception:                                             # noqa: BLE001
        return 1.5


def _leftover(spread, offset) -> None:
    """The block to paste into `oak.py`, so nobody retypes a number."""
    print("to adopt this, put it in oak.py and set MEASURED = True:")
    print()
    print("    MOUNT = Mount(")
    print(f"        yaw_deg={spread('yaw_deg')[0]:.2f},")
    print(f"        pitch_deg={spread('pitch_deg')[0]:.2f},")
    print(f"        roll_deg={spread('roll_deg')[0]:.2f},")
    print(f"        forward_m={offset[0]:.3f},")
    print(f"        left_m={offset[1]:.3f},")
    print(f"        up_m={offset[2]:.3f},")
    print("    )")
    print("    MEASURED = True")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--pan", type=float, nargs="+", default=[0.0],
                        help="gimbal pan positions to solve at (default: just "
                             "0). Several is a check on the mount's rigidity "
                             "rather than more of the answer")
    parser.add_argument("--tilt", type=float, nargs="+", default=[0.0],
                        help="gimbal tilts to solve at, each at every --pan "
                             "(default: just 0). The tilt is what loads the "
                             "mount against gravity, so a sagging clamp shows "
                             "here first")
    parser.add_argument("--offset", type=float, nargs=3, default=[0.0, 0.0, 0.0],
                        metavar=("FORWARD", "LEFT", "UP"),
                        help="where the OAK's lens sits relative to the gimbal "
                             "camera's, in metres. **Measure this with a ruler**: "
                             "this bench cannot, and says why. Default is nothing, "
                             "which for two cameras a few centimetres apart costs "
                             "about a degree of bearing at two metres")
    parser.add_argument("--points", metavar="FILE",
                        help="where to keep the matched points. Written after a "
                             "run and read instead of the cameras if the file "
                             "is already there -- so the offset can be measured "
                             "with a ruler afterwards and the rotation re-solved "
                             "without needing the rover back in the same room")
    parser.add_argument("--joint", action="store_true",
                        help="also fit one mount to every position at once, with "
                             "the left and up offset and the fisheye's angular "
                             "scale free and the forward offset held at "
                             "--offset's. This is how oak.MOUNT was measured on "
                             "2026-09-30; it needs scipy")
    parser.add_argument("--save", metavar="PREFIX",
                        help="write both pictures per position, the OAK's warped "
                             "into the fisheye's geometry, so a person can see "
                             "what was matched")
    args = parser.parse_args()

    print("bench_oak -- which way the OAK points, against the camera the rover "
          "trusts")
    print()
    try:
        import cv2
        import numpy
    except ImportError as error:
        print(f"this needs OpenCV and numpy on the host that runs it: {error}")
        return 1

    kept = _read_points(args.points)
    if kept is None:
        ranger = SidecarRanger()
        lens_oak = ranger.lens()
        if lens_oak is None:
            print("the depth camera would not say what lens it has; is oak_depth "
                  "running, and is it a build that serves the colour half?")
            return 1
        first = gimbal_frame(args.pan[0], args.tilt[0])
        if not first["ok"]:
            print(f"the gimbal camera would not answer: {first['error']}")
            return 1
        maps = warp_maps(numpy, first["size"], lens_oak)
        kept, notes = [], []
        for pan in args.pan:
            for tilt in args.tilt:
                one, note = collect_at(cv2, numpy, pan, tilt, ranger, lens_oak,
                                       maps, args.save)
                notes.append(note)
                if one is not None:
                    kept.append(one)
        _write_points(args.points, kept)
    else:
        notes = [f"{len(kept)} position(s) read back from {args.points}, "
                 f"the cameras untouched"]

    found = []
    for one in kept:
        solved = solve_at(numpy, one, args.offset)
        if solved is None:
            notes.append(f"pan {one['pan_deg']:+.0f} tilt "
                         f"{one.get('tilt_deg', 0.0):+.0f}: no rotation "
                         f"survived its {len(one['objects'])} points")
            continue
        # What a solver claims the offset is, from this position's own points.
        # Paired here rather than inside the solve because it is a warning about
        # the method rather than part of the answer -- see `free_offset`.
        solved["free"] = free_offset(cv2, numpy, one["objects"], one["images"])
        found.append(solved)
    status = report(numpy, found, notes, args.offset)
    if args.joint and kept:
        print()
        status = report_joint(*joint_fit(numpy, kept, forward_m=args.offset[0]))
    return status


def _read_points(path):
    """The points a previous run collected, or None to go and collect some.

    **This is what lets a ruler come afterwards.** The offset cannot be measured
    from these pictures and has to be given, and by the time somebody has held a
    ruler up to the rover it has usually been moved -- so what was matched is
    kept, and the rotation can be solved again at a different offset without the
    room having to be the same room.
    """
    if not path or not os.path.exists(path):
        return None
    try:
        with open(path, encoding="utf-8") as handle:
            return json.load(handle)["positions"]
    except (KeyError, OSError, ValueError) as error:
        print(f"  {path} would not read ({error}); collecting afresh")
        return None


def _write_points(path, kept) -> None:
    if not path or not kept:
        return
    try:
        with open(path, "w", encoding="utf-8") as handle:
            json.dump({"positions": kept}, handle)
        print(f"  matched points kept in {path}")
    except OSError as error:
        print(f"  {path} would not be written ({error})")


if __name__ == "__main__":
    raise SystemExit(main())
