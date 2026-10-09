#!/usr/bin/env python3
"""Where the gimbal's pan really goes, from a `capture_pan_sweep.py` recording.

    python usb_cameras/fit_pan_sweep.py captures/2026-10-09-pan-sweep

Applies the pass rule fixed in `capture_pan_sweep.py` before the capture and
prints the verdict, the table of commanded against actual pan, the backlash, and
how long a pan takes. Nothing is written into the rover's constants.

**Actual pan is the OAK's gyro, integrated about gravity across each move.** The
bias is read at every still hold and interpolated between them, so a move of a
second and a half integrates a few thousandths of a degree of it. The scale comes
from closing the circle: the camera's picture at -180 and at +180 look the same
way, the small rotation between them is measured from the pictures, and the
travel between the two is 360 degrees plus that, with no lens and no servo in it.
Positive is to the right, as the servo's own pan is.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "face_tracking"))
import lens as flown                                               # noqa: E402

#: The pass rule, as `capture_pan_sweep.py` fixed it.
HELD_OUT_DEG = 0.5
CLOSURE_AGREE = 0.003
CAMERA_STEP_P95_DEG = 0.3
REST_AGREE_DEG = 0.2


# --- the pictures ----------------------------------------------------------------


def rays(points, size):
    lens = flown.lens_for(*size)
    return np.array([flown.ray_at(x, y, lens) for x, y in points])


def rotation_between(first, second, sift):
    """The rotation taking directions in the first picture to the second, from
    matched features, with RANSAC over three-point rotations. None when the two
    do not share enough."""
    one, two = cv2.imread(str(first)), cv2.imread(str(second))
    if one is None or two is None:
        return None
    size = (one.shape[1], one.shape[0])
    # Evened out first: the camera stops down after facing a window and never
    # opens up again, so the two ends of the circle differ by a fifth in
    # brightness and plain SIFT found too few matches between them to close it.
    even = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    grey = [even.apply(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
            for image in (one, two)]
    (k1, d1), (k2, d2) = (sift.detectAndCompute(g, None) for g in grey)
    if d1 is None or d2 is None or len(k1) < 20 or len(k2) < 20:
        return None
    pairs = cv2.BFMatcher().knnMatch(d1, d2, k=2)
    good = [m for m, n in (p for p in pairs if len(p) == 2)
            if m.distance < 0.75 * n.distance]
    if len(good) < 20:
        return None
    a = rays([k1[m.queryIdx].pt for m in good], size)
    b = rays([k2[m.trainIdx].pt for m in good], size)
    rng = np.random.default_rng(1)
    best, best_in = None, None
    limit = math.radians(0.3)
    for _ in range(400):
        pick = rng.choice(len(a), 3, replace=False)
        rot = kabsch(a[pick], b[pick])
        err = np.arccos(np.clip(np.sum((a @ rot.T) * b, axis=1), -1, 1))
        inliers = err < limit
        if best_in is None or inliers.sum() > best_in.sum():
            best, best_in = rot, inliers
    for _ in range(3):
        best = kabsch(a[best_in], b[best_in])
        err = np.arccos(np.clip(np.sum((a @ best.T) * b, axis=1), -1, 1))
        best_in = err < limit
    if best_in.sum() < 15:
        return None
    return best, int(best_in.sum()), float(np.degrees(np.median(err[best_in])))


def kabsch(a, b):
    """The rotation R with R a ~ b."""
    h = a.T @ b
    u, _, vt = np.linalg.svd(h)
    d = np.sign(np.linalg.det(vt.T @ u.T))
    return vt.T @ np.diag([1.0, 1.0, d]) @ u.T


def pan_of(rotation, tilt_deg):
    """How far a camera rotation turned it to the right about the pan axis, in
    degrees. The axis is the rover's up, which in the camera's own frame (x right,
    y down, z out) at a tilt t above level is (0, -cos t, sin t); the camera's
    rotation is the inverse of the one that moves directions in its picture."""
    t = math.radians(tilt_deg)
    up = np.array([0.0, -math.cos(t), math.sin(t)])
    vector = cv2.Rodrigues(rotation.T)[0].ravel()
    return -math.degrees(float(vector @ up))


# --- the gyro ----------------------------------------------------------------------


#: A hold whose mean rate is this far from the recording's median bias was not
#: still: the gimbal had not finished the move before it. On 2026-10-09 that was
#: every hold after a swing of 180 degrees (4 to 5 degrees a second), against
#: 0.13 at most for every other hold. Such a hold supplies neither a bias nor a
#: stop -- reading its mean as the bias put tens of degrees into the moves on
#: either side of it.
MOVING_DPS = 0.5


def integrate(imu, stops):
    """The angle turned about gravity, accumulated over the whole recording, read
    at each stop's hold: degrees, to the left positive (as about up). A stop whose
    hold was not still comes back None."""
    host, device = imu[:, 6], imu[:, 7]
    holds = []
    for stop in stops:
        inside = (host >= stop["hold_from"]) & (host <= stop["hold_to"])
        if inside.sum() < 20:
            holds.append(None)
            continue
        holds.append({"bias": imu[inside, 3:6].mean(axis=0),
                      "gravity": imu[inside, 0:3].mean(axis=0),
                      "at": float(device[inside].mean()),
                      "from": float(device[inside].min()),
                      "to": float(device[inside].max())})
    middle = np.median([h["bias"] for h in holds if h is not None], axis=0)
    for index, hold in enumerate(holds):
        if hold is not None and np.degrees(
                np.linalg.norm(hold["bias"] - middle)) > MOVING_DPS:
            holds[index] = None
    known = [h for h in holds if h is not None]
    at = np.array([h["at"] for h in known])
    bias = np.array([h["bias"] for h in known])
    gravity = np.array([h["gravity"] for h in known])
    order = np.argsort(device)
    t = device[order]
    gyro = imu[order, 3:6]
    b = np.column_stack([np.interp(t, at, bias[:, k]) for k in range(3)])
    g = np.column_stack([np.interp(t, at, gravity[:, k]) for k in range(3)])
    g /= np.linalg.norm(g, axis=1, keepdims=True)
    rate = np.sum((gyro - b) * g, axis=1)
    dt = np.diff(t, prepend=t[0])
    angle = np.degrees(np.cumsum(rate * dt))
    out = []
    for hold in holds:
        if hold is None:
            out.append(None)
            continue
        inside = (t >= hold["from"]) & (t <= hold["to"])
        out.append({"angle": float(angle[inside].mean()),
                    "drift": float(np.ptp(angle[inside])),
                    "gravity": hold["gravity"].tolist()})
    return out, t, np.degrees(rate), angle


# --- the verdict --------------------------------------------------------------------


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("folder")
    args = parser.parse_args()
    folder = Path(args.folder)
    meta = json.loads((folder / "meta.json").read_text(encoding="utf-8"))
    imu = np.load(folder / "imu.npy")
    stops = meta["stops"]
    gyro, t, rate, angle = integrate(imu, stops)
    sift = cv2.SIFT_create(nfeatures=8000)

    def of(label, pan=None, tilt=None):
        return [s for s in stops if s["label"] == label
                and (pan is None or s["pan"] == pan)
                and (tilt is None or s["tilt"] == tilt)]

    def picture(stop):
        return folder / stop["picture"]

    # The circle, closed wherever a still stop at -180 and one at +180 were
    # pictured at the same tilt. "A" is the development pass's own +180 against
    # the first still -180 there is, which is the end of the descending pass.
    scales = {}
    for label, start, end in (("A", of("C", -180)[0], of("A", 180)[0]),
                              ("B", of("B", -180)[0], of("B", 180)[0]),
                              ("D", of("D", -180)[0], of("D", 180)[0])):
        if gyro[start["index"]] is None or gyro[end["index"]] is None:
            print(f"closure {label}: an end was not still, so the gyro cannot "
                  f"say how far apart they are")
            continue
        rot = rotation_between(picture(start), picture(end), sift)
        if rot is None:
            print(f"closure {label}: the pictures at the two ends do not match")
            continue
        small = pan_of(rot[0], start["tilt"])
        turned = -(gyro[end["index"]]["angle"] - gyro[start["index"]]["angle"])
        truth = 360.0 + small if turned > 0 else -360.0 + small
        scales[label] = turned / truth
        print(f"closure {label}: gyro {turned:+.3f}, pictures say {truth:+.3f} "
              f"({rot[1]} features, {rot[2]:.3f} deg median), scale "
              f"{scales[label]:.5f}")
    if "A" not in scales:
        print("INCONCLUSIVE: pass A did not close")
        return 1
    scale = float(np.mean([scales[k] for k in ("A", "B") if k in scales]))
    closure_ok = abs(scales["A"] - scales.get("B", float("nan"))) <= CLOSURE_AGREE
    print(f"gyro scale {scale:.5f}; A and B agree within "
          f"{abs(scales['A'] - scales.get('B', float('nan'))):.5f} "
          f"(rule {CLOSURE_AGREE}): {'pass' if closure_ok else 'FAIL'}")
    skipped = [f"{s['label']} {s['pan']:+d}" for s in stops
               if gyro[s["index"]] is None]
    print(f"stops not still, and left out: {', '.join(skipped) or 'none'}")

    zero = of("A", 0)[0]

    def actual(stop):
        if gyro[stop["index"]] is None:
            return None
        return -(gyro[stop["index"]]["angle"] - gyro[zero["index"]]["angle"]) / scale

    # The camera's own account of each ten-degree step of pass A.
    a_stops = [s for s in of("A") if actual(s) is not None]
    diffs = []
    for one, two in zip(a_stops, a_stops[1:]):
        if two["pan"] - one["pan"] != 10:
            continue
        rot = rotation_between(picture(one), picture(two), sift)
        if rot is None:
            continue
        diffs.append((two["pan"], pan_of(rot[0], 20) - (actual(two) - actual(one)),
                      rot[1]))
    d = np.array([x[1] for x in diffs])
    common = float(np.median(d)) if len(d) else float("nan")
    p95 = float(np.percentile(np.abs(d - common), 95)) if len(d) else float("nan")
    camera_ok = len(d) >= 0.8 * (len(a_stops) - 1) and p95 <= CAMERA_STEP_P95_DEG
    print(f"camera against gyro, {len(d)} of {len(a_stops) - 1} steps: common "
          f"{common:+.3f} deg a step, p95 about it {p95:.3f} (rule "
          f"{CAMERA_STEP_P95_DEG}): {'pass' if camera_ok else 'FAIL'}")
    worst = sorted(diffs, key=lambda x: -abs(x[1] - common))[:4]
    print("  worst steps:", ", ".join(f"to {p:+d}: {v - common:+.2f} ({n})"
                                     for p, v, n in worst))

    rests = [s for s in stops if s["label"].startswith("rest")]
    rot = rotation_between(picture(rests[0]), picture(rests[-1]), sift)
    rest_shift = abs(pan_of(rot[0], 20)) if rot else float("nan")
    rest_ok = rest_shift <= REST_AGREE_DEG
    print(f"rest pictures, first to last: {rest_shift:.3f} deg (rule "
          f"{REST_AGREE_DEG}): {'pass' if rest_ok else 'FAIL'}")
    print("rest by the gyro:", ", ".join(
        f"{s['label']} {actual(s):+.3f}" for s in rests if actual(s) is not None))
    backs = [actual(s) for s in of("T-back") if actual(s) is not None]
    if backs:
        print(f"rest after each timed swing, by the gyro: "
              f"{min(backs):+.3f} to {max(backs):+.3f}")
    drift = max(g["drift"] for g in gyro if g)
    print(f"largest angle change within a hold: {drift:.3f} deg")
    reference_ok = closure_ok and camera_ok and rest_ok

    # The candidate, and the held-out judgement.
    commanded = np.array([s["pan"] for s in a_stops], float)
    measured = np.array([actual(s) for s in a_stops])
    print("\npass A, commanded -> actual (error):")
    for c, m in zip(commanded, measured):
        print(f"  {c:+5.0f}  {m:+8.2f}  ({m - c:+.2f})")
    gain = np.polyfit(commanded[np.abs(commanded) < 180],
                      measured[np.abs(commanded) < 180], 1)
    print(f"one straight line through A: actual = {gain[0]:.4f} x commanded "
          f"{gain[1]:+.2f}; worst miss "
          f"{np.max(np.abs(np.polyval(gain, commanded) - measured)):.2f} deg")

    held = []
    for s in of("B"):
        if abs(s["pan"]) == 180 or actual(s) is None:
            continue
        predicted = float(np.interp(s["pan"], commanded, measured))
        held.append((s["pan"], actual(s) - predicted, actual(s) - s["pan"]))
    print("\npass B held out, commanded: residual against the candidate "
          "(raw error)")
    for p, r, raw in held:
        print(f"  {p:+5d}  {r:+.2f}  ({raw:+.2f})")
    reach = {}
    for side in (1, -1):
        reach[side] = 0
        for p, r, _ in sorted((h for h in held if h[0] * side > 0),
                              key=lambda h: abs(h[0])):
            if abs(r) > HELD_OUT_DEG:
                break
            reach[side] = abs(p)
    print(f"held-out stops within {HELD_OUT_DEG} deg out to -{reach[-1]} and "
          f"+{reach[1]}; residual p95 "
          f"{np.percentile(np.abs([h[1] for h in held]), 95):.2f}")

    print("\nbacklash, ascending minus descending (pass A against C):")
    back = []
    for s in of("C"):
        up = of("A", s["pan"])
        if up and abs(s["pan"]) < 180 and None not in (actual(up[0]), actual(s)):
            back.append((s["pan"], actual(up[0]) - actual(s)))
    print("  " + ", ".join(f"{p:+d}: {b:+.2f}" for p, b in back))

    if of("D"):
        dz = of("D", 0)[0]
        print("\ntilt 0 (pass D), commanded -> actual (error):")
        for s in of("D"):
            if gyro[s["index"]] is None:
                continue
            m = -(gyro[s["index"]]["angle"] - gyro[dz["index"]]["angle"]) / scale
            print(f"  {s['pan']:+5d}  {m:+8.2f}  ({m - s['pan']:+.2f})")

    print("\nhow long a pan takes, from the command until the gimbal is within "
          "0.2 deg of where it stops:")
    host, device = imu[:, 6], imu[:, 7]
    offset = float(np.median(host - device))
    for move in meta["moves"]:
        if move["label"] not in ("T-under", "T-back"):
            continue
        start = move["sent"] - offset
        later = [s for s in stops if s["hold_from"] > move["sent"]]
        if not later:
            continue
        end = later[0]["hold_from"] - offset
        inside = (t >= start) & (t <= end)
        if inside.sum() < 10:
            continue
        final = angle[inside][-1]
        far = np.abs(angle[inside] - final) > 0.2
        arrived = t[inside][np.nonzero(far)[0][-1] + 1] if far.any() else start
        swing = angle[inside][-1] - angle[inside][0]
        print(f"  {move['label']:8s} to {move['pan']:+5.0f}: {abs(swing):6.1f} deg "
              f"in {arrived - start:.2f} s")

    print(f"\nreference: {'PASS' if reference_ok else 'INCONCLUSIVE'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
