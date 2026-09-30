#!/usr/bin/env python3
"""Fit the gimbal camera's lens without trusting a servo, and check it on gravity.

    python usb_cameras/fit_fisheye.py
    python usb_cameras/fit_fisheye.py --terms 3

What `face_tracking/lens.py` flies since 2026-09-30 came out of this. Its own form
-- an angular scale, distortion terms in the normalised radius, a centre -- is
fitted to three kinds of evidence at once, each carrying its own nuisance
parameters and none carrying a servo angle:

  board corners      a printed ChArUco board in many frames, each frame its own
                     pose: the centre and the shape of the middle of the picture
  OAK matches        features matched with the OAK on the same platform, through
                     its factory lens and stored distortion, its stereo ranges and
                     the mount between them: the absolute scale, which the board
                     cannot pin (a flat target nearly face on trades focal length
                     for distance)
  sweep pairs        still pictures either side of a gimbal step, distant features
                     only, each pair its own rotation: the shape out to the edges

Then it is checked against something it was not fitted to: the tilt steps of the
same sweep, whose true size the OAK's accelerometer read off gravity. It prints
the scale each lens would need to match them -- 1 is right -- for the fit and for
the lens `lens.py` flies now.

The inputs are what was kept on 2026-09-30 and are not in Git (`captures/` is
ignored): the board frames of 2026-09-07, the OAK matches from
`world_state/bench_oak.py` (a points file whose positions carry `pixels`, which
the bench records since that day), and a sweep from `capture_lens_sweep.py`.
Nothing is written into `lens.py`; the answer is printed in its form.
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import brentq, least_squares
from scipy.sparse import lil_matrix
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "usb_cameras"))
sys.path.insert(0, str(ROOT / "voice_chat"))
sys.path.insert(0, str(ROOT / "face_tracking"))
sys.path.insert(0, str(ROOT))
import calibrate_gimbal as gimbal                                  # noqa: E402
import lens as flown                                               # noqa: E402
from world_state.bench_oak import angles_of                       # noqa: E402

FRAME = (640, 480)
NORMAL = FRAME[0] / 2.0
CAPTURES = ROOT / "captures"


# --- the model -------------------------------------------------------------------


def theta(radius, scale, bends):
    poly = 1.0
    for power, bend in enumerate(bends, 1):
        poly = poly + bend * (radius / NORMAL) ** (2 * power)
    return radius * scale * poly


def rays(pixels, params, terms):
    """Unit directions, x right, y down, z out, through (scale, cx, cy, bends...)."""
    offset = pixels - np.array([params[1], params[2]])
    radius = np.hypot(offset[:, 0], offset[:, 1])
    angle = theta(radius, params[0], params[3:3 + terms])
    across = np.sin(angle) / np.where(radius > 1e-9, radius, 1.0)
    return np.column_stack([offset[:, 0] * across, offset[:, 1] * across,
                            np.cos(angle)])


# --- the evidence ----------------------------------------------------------------


def board_frames(pattern):
    board, detector = gimbal.board_and_detector()
    corners = board.getChessboardCorners()
    frames = []
    for path in sorted(glob.glob(pattern)):
        image = cv2.imread(path)
        if image is None:
            continue
        found = gimbal.detect(image, detector)
        if found["charuco_corners"] < 12:
            continue
        scale = FRAME[0] / image.shape[1]
        frames.append((corners[np.array(found["ids"])],
                       np.array(found["image_points_px"], float) * scale))
    return frames


def oak_matches(points_files, health_file):
    """Fisheye pixels, and the OAK's 3D points through its exact lens."""
    health = json.loads(Path(health_file).read_text(encoding="utf-8"))["colour"]
    lens = health["intrinsics"]
    matrix = np.array([[lens["fx"], 0, lens["cx"]], [0, lens["fy"], lens["cy"]],
                       [0, 0, 1.0]])
    distortion = np.array(health["distortion"], float)
    pixels, points = [], []
    for name in points_files:
        for position in json.loads(Path(name).read_text())["positions"]:
            if "pixels" not in position:
                raise SystemExit(f"{name} carries no fisheye pixels; re-collect it "
                                 f"with today's bench_oak.py")
            # The bench stores the OAK's points through the pinhole the service
            # publishes; back to its pixels and depth, then through its real lens.
            obj = np.array(position["objects"])
            u = obj[:, 0] / obj[:, 2] * lens["fx"] + lens["cx"]
            v = obj[:, 1] / obj[:, 2] * lens["fy"] + lens["cy"]
            exact = cv2.undistortPoints(np.column_stack([u, v]).reshape(-1, 1, 2),
                                        matrix, distortion).reshape(-1, 2)
            points.append(np.column_stack([exact, np.ones(len(exact))])
                          * obj[:, 2:3])
            pixels.append(np.array(position["pixels"], float))
    return np.vstack(pixels), np.vstack(points)


def tracks(before, after, band):
    grey = [cv2.cvtColor(one, cv2.COLOR_BGR2GRAY) for one in (before, after)]
    height, width = grey[0].shape
    mask = np.zeros_like(grey[0])
    mask[int(band[0] * height):int(band[1] * height),
         int(0.03 * width):int(0.97 * width)] = 255
    start = cv2.goodFeaturesToTrack(grey[0], 1500, 0.01, 6, mask=mask)
    if start is None:
        return np.zeros((0, 2)), np.zeros((0, 2))
    moved, ok, _ = cv2.calcOpticalFlowPyrLK(grey[0], grey[1], start, None,
                                            winSize=(21, 21), maxLevel=4)
    back, ok_back, _ = cv2.calcOpticalFlowPyrLK(grey[1], grey[0], moved, None,
                                                winSize=(21, 21), maxLevel=4)
    good = ((ok.ravel() == 1) & (ok_back.ravel() == 1)
            & (np.linalg.norm(back - start, axis=2).ravel() < 0.3))
    return start.reshape(-1, 2)[good], moved.reshape(-1, 2)[good]


def sweep(folder):
    """(stops, pan_from): the capture's metadata, in either of its two shapes."""
    meta = json.loads((Path(folder) / "meta.json").read_text())
    if isinstance(meta, list):                  # the first sweep, 2026-09-30
        return meta, 12
    return meta["stops"], meta["pan_from"]


def sweep_pairs(folder, stops, pan_from):
    pairs = []
    for index in range(1, len(stops)):
        if index == pan_from:
            continue
        before = cv2.imread(f"{folder}/{index - 1:02d}-gimbal.jpg")
        after = cv2.imread(f"{folder}/{index:02d}-gimbal.jpg")
        # Distant things only: the floor is half a metre off and moves by parallax.
        band = (0.0, 0.55) if stops[index]["tilt"] < 25 else (0.0, 0.8)
        pairs.append((index, *tracks(before, after, band)))
    return pairs


# --- the fit ---------------------------------------------------------------------


def initial_poses(frames, params, terms):
    poses = []
    for obj, pixels in frames:
        direction = rays(pixels, params, terms)
        normalised = direction[:, :2] / direction[:, 2:3]
        _ok, rvec, tvec = cv2.solvePnP(obj.astype(np.float64),
                                       normalised.astype(np.float64), np.eye(3),
                                       None, flags=cv2.SOLVEPNP_IPPE)
        poses.append(np.concatenate([rvec.ravel(), tvec.ravel()]))
    return poses


def fit(frames, oak_pixels, oak_points, pairs, terms):
    start = [flown.lens_for(*FRAME)[0] * 1.07, FRAME[0] / 2 + 3, FRAME[1] / 2 - 13,
             -0.05] + [0.0] * (terms - 1)
    nl = 3 + terms
    x0 = np.concatenate([start, np.zeros(3), [-0.005, 0.045]]
                        + initial_poses(frames, np.array(start), terms)
                        + [np.zeros(3)] * len(pairs))
    base = nl + 5
    base2 = base + 6 * len(frames)

    def residual(x):
        out = []
        for index, (obj, pixels) in enumerate(frames):
            pose = x[base + 6 * index: base + 6 * index + 6]
            want = obj @ Rotation.from_rotvec(pose[:3]).as_matrix().T + pose[3:]
            want /= np.linalg.norm(want, axis=1, keepdims=True)
            out.append(np.cross(rays(pixels, x[:nl], terms), want))
        seen = (Rotation.from_rotvec(x[nl:nl + 3]).apply(oak_points)
                + np.array([-x[nl + 3], -x[nl + 4], 0.0]))
        seen /= np.linalg.norm(seen, axis=1, keepdims=True)
        out.append(np.cross(rays(oak_pixels, x[:nl], terms), seen))
        for index, (_i, first, second) in enumerate(pairs):
            turn = Rotation.from_rotvec(x[base2 + 3 * index: base2 + 3 * index + 3])
            out.append(np.cross(rays(first, x[:nl], terms) @ turn.as_matrix().T,
                                rays(second, x[:nl], terms)))
        return np.vstack(out).ravel() / x[0]    # in pixels on the axis

    sizes = ([len(pixels) for _obj, pixels in frames] + [len(oak_points)]
             + [len(first) for _i, first, _s in pairs])
    edges = np.cumsum([0] + sizes)
    sparsity = lil_matrix((3 * edges[-1], len(x0)), dtype=int)
    sparsity[:, :nl] = 1
    for index in range(len(frames)):
        sparsity[3 * edges[index]:3 * edges[index + 1],
                 base + 6 * index: base + 6 * index + 6] = 1
    sparsity[3 * edges[len(frames)]:3 * edges[len(frames) + 1], nl:nl + 5] = 1
    for index in range(len(pairs)):
        k = len(frames) + 1 + index
        sparsity[3 * edges[k]:3 * edges[k + 1],
                 base2 + 3 * index: base2 + 3 * index + 3] = 1
    solved = least_squares(residual, x0, jac_sparsity=sparsity, x_scale="jac",
                           loss="soft_l1", f_scale=0.5, max_nfev=300)
    miss = np.linalg.norm(solved.fun.reshape(-1, 3), axis=1)
    split = (edges[len(frames)], edges[len(frames) + 1])
    return solved.x, miss, split


# --- the check -------------------------------------------------------------------


def gravity_scale(folder, stops, pan_from, params, terms):
    """The scale this lens needs to match the tilt steps gravity measured."""
    gravity = [np.array(one["acc"]) / np.linalg.norm(one["acc"]) for one in stops]
    needed = []
    for index in range(1, pan_from):
        truth = math.degrees(math.acos(float(np.clip(gravity[index - 1]
                                                     @ gravity[index], -1, 1))))
        first, second = tracks(cv2.imread(f"{folder}/{index - 1:02d}-gimbal.jpg"),
                               cv2.imread(f"{folder}/{index:02d}-gimbal.jpg"),
                               (0.02, 0.62))

        def angle_at(k):
            stretched = np.array(params, float)
            stretched[0] *= k
            a, b = rays(first, stretched, terms), rays(second, stretched, terms)
            keep = np.ones(len(a), bool)
            for _round in range(6):
                u, _s, vt = np.linalg.svd(b[keep].T @ a[keep])
                d = np.eye(3)
                d[2, 2] = np.sign(np.linalg.det(u @ vt))
                turn = u @ d @ vt
                miss = np.degrees(np.arccos(np.clip(np.sum((a @ turn.T) * b, axis=1),
                                                    -1, 1)))
                keep = miss < max(0.15, 3 * np.median(miss))
            return math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(turn) - 1) / 2))))

        try:
            needed.append(brentq(lambda k: angle_at(k) - truth, 0.8, 1.25, xtol=1e-5))
        except ValueError:
            continue
    return needed


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--terms", type=int, default=2)
    parser.add_argument("--board", default=str(CAPTURES / "p0-gimbal-2026-09-07"
                                               / "*" / "optics_*.jpg"))
    parser.add_argument("--oak-points", nargs="+", default=[str(
        CAPTURES / "2026-09-30-oak-rail" / "points-both-refitted-lens.json")])
    parser.add_argument("--oak-health", default=str(
        CAPTURES / "2026-09-30-oak-rail" / "oak-health.json"))
    parser.add_argument("--sweep", default=str(CAPTURES / "2026-09-30-fisheye-sweep"))
    args = parser.parse_args()

    frames = board_frames(args.board)
    oak_pixels, oak_points = oak_matches(args.oak_points, args.oak_health)
    stops, pan_from = sweep(args.sweep)
    pairs = sweep_pairs(args.sweep, stops, pan_from)
    print(f"{len(frames)} board frames, {sum(len(p) for _o, p in frames)} corners; "
          f"{len(oak_points)} OAK matches; {len(pairs)} sweep pairs, "
          f"{sum(len(f) for _i, f, _s in pairs)} tracks")

    x, miss, (board_end, oak_end) = fit(frames, oak_pixels, oak_points, pairs,
                                        args.terms)
    params = x[:3 + args.terms]
    print(f"\nboard corners median {np.median(miss[:board_end]):.3f} px; OAK "
          f"matches {np.median(miss[board_end:oak_end]):.3f} px; sweep tracks "
          f"{np.median(miss[oak_end:]):.3f} px")
    now = flown.lens_for(*FRAME)
    print("\n  off axis   fitted   lens.py now")
    for radius in (100, 150, 200, 250, 300, 320, 400):
        print(f"  {radius:4d} px  {math.degrees(theta(radius, params[0], params[3:])):7.2f}"
              f"  {math.degrees(flown.theta_of(radius, now)):7.2f}")
    mount = angles_of(np, Rotation.from_rotvec(x[3 + args.terms:6 + args.terms])
                      .as_matrix(), 0.0, 0.0)
    print(f"\nthe OAK through it: yaw {mount['yaw_deg']:+.2f} pitch "
          f"{mount['pitch_deg']:+.2f} roll {mount['roll_deg']:+.2f} deg, "
          f"{x[6 + args.terms]:+.4f} left {x[7 + args.terms]:+.4f} up (through its "
          f"exact lens; bench_oak.py --joint gives the figure for the pinhole the "
          f"rover draws it as)")

    fitted = gravity_scale(args.sweep, stops, pan_from, params, args.terms)
    flown_params = [now[0], now[2], now[3]] + list(
        now[1] if isinstance(now[1], (list, tuple)) else (now[1],))
    flying = gravity_scale(args.sweep, stops, pan_from, flown_params,
                           len(flown_params) - 3)
    print(f"\nagainst gravity, the scale each lens needs (1 is right): fitted "
          f"{np.median(fitted):.4f} over {len(fitted)} tilt steps, lens.py now "
          f"{np.median(flying):.4f}")
    bends = ", ".join(f"{b:+.4f}" for b in params[3:])
    print(f"\nin lens.py's form:  (640, 480): ({math.degrees(params[0]) * 60:.3f}, "
          f"({bends}), ({params[1]:.1f}, {params[2]:.1f})),")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
