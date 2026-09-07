#!/usr/bin/env python3
"""Measure commanded gimbal pan against ChArUco-derived camera orientation.

The board pose and lens fit come from the printed geometry and image corners. Servo
commands choose views but are never used as observations. A live run preserves every
attempt and returns the camera to pan/tilt zero without changing deployed constants.

    python usb_cameras/calibrate_gimbal.py captures/p0-gimbal-YYYY-MM-DD/campaign
    python usb_cameras/calibrate_gimbal.py captures/.../campaign --fit-only
    python usb_cameras/calibrate_gimbal.py --selftest
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import math
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "voice_chat"))
from rover_tools import RoverClient, discover  # noqa: E402


SQUARES = (10, 7)
SQUARE_M = 0.024
MARKER_M = 0.017
UPSCALE = 3
MIN_CORNERS = 12
SETTLE_S = 2.5
PAN_SAMPLES = (-20, -10, 0, 10, 20)
OPTICS_VIEWS = (
    (-20, 0), (0, 0), (20, 0),
    (-20, 10), (0, 10), (20, 10),
    (-20, 20), (0, 20), (20, 20),
)


def now() -> str:
    return datetime.now().astimezone().isoformat()


def repository_head() -> str | None:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], text=True, stderr=subprocess.DEVNULL
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def board_and_detector():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    board = cv2.aruco.CharucoBoard(SQUARES, SQUARE_M, MARKER_M, dictionary)
    parameters = cv2.aruco.DetectorParameters()
    parameters.minMarkerPerimeterRate = 0.005
    parameters.adaptiveThreshWinSizeMin = 3
    parameters.adaptiveThreshWinSizeMax = 53
    parameters.adaptiveThreshWinSizeStep = 4
    parameters.minCornerDistanceRate = 0.01
    parameters.minDistanceToBorder = 1
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.CharucoDetector(
        board, cv2.aruco.CharucoParameters(), parameters
    )
    return board, detector


def detect(frame: np.ndarray, detector) -> dict:
    enlarged = cv2.resize(
        frame, None, fx=UPSCALE, fy=UPSCALE, interpolation=cv2.INTER_LANCZOS4
    )
    corners, ids, marker_corners, marker_ids = detector.detectBoard(enlarged)
    marker_count = 0 if marker_ids is None else len(marker_ids)
    corner_count = 0 if ids is None else len(ids)
    return {
        "markers": marker_count,
        "charuco_corners": corner_count,
        "ids": [] if ids is None else [int(value) for value in ids.reshape(-1)],
        "image_points_px": [] if corners is None else (
            np.asarray(corners, float).reshape(-1, 2) / UPSCALE
        ).round(5).tolist(),
    }


def capture_jpeg(rover: RoverClient,
                 size: tuple[int, int]) -> tuple[bytes, dict]:
    reply = rover.call("camera_jpeg", {"width": size[0], "height": size[1]})
    if not reply.get("ok"):
        raise RuntimeError(f"camera capture failed: {reply.get('error')}")
    return base64.b64decode(reply["jpeg_base64"]), reply


def save_frame(folder: Path, name: str, data: bytes, reply: dict,
               detector) -> dict:
    path = folder / f"{name}.jpg"
    path.write_bytes(data)
    frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        raise RuntimeError(f"OpenCV could not decode {path}")
    found = detect(frame, detector)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return {
        "file": path.name,
        "sha256": hashlib.sha256(data).hexdigest(),
        "width": int(frame.shape[1]),
        "height": int(frame.shape[0]),
        "mean_luma": round(float(gray.mean()), 2),
        "laplacian_variance": round(
            float(cv2.Laplacian(gray, cv2.CV_64F).var()), 2
        ),
        "camera_reply": {
            key: reply[key] for key in ("width", "height") if key in reply
        },
        "detection": found,
    }


def write_meta(folder: Path, meta: dict) -> None:
    (folder / "campaign.json").write_text(
        json.dumps(meta, indent=2) + "\n", encoding="utf-8"
    )


def take_view(rover: RoverClient, folder: Path, detector, name: str,
              pan: float, tilt: float, settle: float, size: tuple[int, int],
              details: dict) -> list[dict]:
    sent = rover.call("look_at", {"pan": pan, "tilt": tilt})
    if not sent.get("ok"):
        raise RuntimeError(f"gimbal refused pan {pan}, tilt {tilt}: {sent}")
    time.sleep(settle)
    # Existing camera bench tools discard the first reopened snapshot because it
    # can predate the move. The following two are independent stationary frames;
    # their difference measures image/pose noise without calling it servo error.
    capture_jpeg(rover, size)
    rows = []
    for duplicate in range(2):
        data, reply = capture_jpeg(rover, size)
        row = dict(details)
        row.update({
            "name": f"{name}_f{duplicate + 1}",
            "captured_at": now(),
            "commanded_pan_deg": pan,
            "commanded_tilt_deg": tilt,
            "returned_pan_deg": sent.get("pan"),
            "returned_tilt_deg": sent.get("tilt"),
            "stationary_duplicate": duplicate + 1,
        })
        row.update(save_frame(folder, row["name"], data, reply, detector))
        rows.append(row)
        time.sleep(0.25)
    return rows


def capture(folder: Path, rover: RoverClient, settle: float,
            size: tuple[int, int], pan_samples: tuple[int, ...] = PAN_SAMPLES,
            candidate: str | None = None, pan_tilt_deg: int = 0) -> dict:
    """Record one campaign. `pan_tilt_deg` is the tilt the pan sweeps are taken
    at, and it is a measurement input rather than a convenience.

    **The pan result belongs to the tilt it was measured at, and the first
    campaign measured only tilt zero.** The rover rests at tilt 20, because the
    camera is low and level fills the frame with floor, and 1828 of the 2165
    looks in its store were taken there -- so the servo behaviour that matters
    most in service is the behaviour at a tilt no campaign had visited. The
    optics views already span tilt 0 to 20 and the lens fit is unaffected; what
    is tilt-specific is the servo's own backlash and gain, which is what the
    sweeps measure. It is written into the campaign metadata so an analysis
    cannot silently compare two tilts.
    """
    if folder.exists():
        raise RuntimeError(f"refusing to overwrite existing attempt: {folder}")
    folder.mkdir(parents=True)
    _, detector = board_and_detector()
    status = rover.call("tracking_status", {})
    meta = {
        "schema": 1,
        "status": "running",
        "started_at": now(),
        "repository_head": repository_head(),
        "opencv_version": cv2.__version__,
        "rover": rover.describe(),
        "board": {
            "squares_x": SQUARES[0], "squares_y": SQUARES[1],
            "square_m": SQUARE_M, "marker_m": MARKER_M,
            "dictionary": "DICT_4X4_50",
        },
        "settle_s": settle,
        "requested_frame_size": list(size),
        "pan_samples_deg": list(pan_samples),
        "pan_sweep_tilt_deg": pan_tilt_deg,
        "candidate": candidate,
        "initial_status": status,
        "optics": [],
        "samples": [],
        "failure": None,
    }
    write_meta(folder, meta)
    if not status.get("ok"):
        raise RuntimeError(f"could not read gimbal state: {status}")
    if status.get("tracking"):
        raise RuntimeError("face tracking is active; no campaign was started")

    try:
        print("capturing independent optics views", flush=True)
        for index, (pan, tilt) in enumerate(OPTICS_VIEWS):
            rows = take_view(
                rover, folder, detector, f"optics_{index:02d}", pan, tilt,
                settle, size, {"kind": "optics", "view": index},
            )
            meta["optics"].extend(rows)
            write_meta(folder, meta)
            print(
                f"  optics {pan:+3d}/{tilt:+2d}: "
                + ", ".join(
                    f"{row['detection']['charuco_corners']} corners" for row in rows
                ), flush=True,
            )

        print("capturing three paired pan sweeps", flush=True)
        for pair in range(1, 4):
            approaches = ("ascending", "descending") if pair % 2 else (
                "descending", "ascending"
            )
            for approach in approaches:
                values = pan_samples if approach == "ascending" else tuple(
                    reversed(pan_samples)
                )
                overshoot = -30 if approach == "ascending" else 30
                sent = rover.call("look_at",
                                  {"pan": overshoot, "tilt": pan_tilt_deg})
                if not sent.get("ok"):
                    raise RuntimeError(f"gimbal refused overshoot {overshoot}: {sent}")
                time.sleep(settle)
                for pan in values:
                    stem = f"pair{pair}_{approach[:3]}_{pan:+03d}".replace("+", "p").replace("-", "m")
                    rows = take_view(
                        rover, folder, detector, stem, pan, pan_tilt_deg,
                        settle, size,
                        {"kind": "pan", "pair": pair, "approach": approach},
                    )
                    meta["samples"].extend(rows)
                    write_meta(folder, meta)
                    print(
                        f"  pair {pair} {approach:10s} {pan:+3d}: "
                        + ", ".join(
                            f"{row['detection']['charuco_corners']} corners"
                            for row in rows
                        ), flush=True,
                    )
        meta["status"] = "complete"
    except Exception as error:
        meta["status"] = "invalid"
        meta["failure"] = repr(error)
        raise
    finally:
        meta["returned_to"] = rover.call("look_at", {"pan": 0, "tilt": 0})
        meta["finished_at"] = now()
        write_meta(folder, meta)
    return meta


def observations(rows: list[dict], board) -> tuple[list[np.ndarray], list[np.ndarray], list[dict]]:
    object_all = np.asarray(board.getChessboardCorners(), np.float64)
    objects, images, accepted = [], [], []
    for row in rows:
        found = row["detection"]
        if found["charuco_corners"] < MIN_CORNERS:
            continue
        ids = np.asarray(found["ids"], int)
        image = np.asarray(found["image_points_px"], np.float64)
        objects.append(object_all[ids].reshape(1, -1, 3))
        images.append(image.reshape(1, -1, 2))
        accepted.append(row)
    return objects, images, accepted


def fit_intrinsics(objects: list[np.ndarray], images: list[np.ndarray],
                   size: tuple[int, int]):
    if len(objects) < 6:
        raise RuntimeError(f"only {len(objects)} usable optics frames; need at least 6")
    width, height = size
    focal = max(width, height) / math.pi
    matrix = np.array(
        [[focal, 0, width / 2], [0, focal, height / 2], [0, 0, 1]],
        np.float64,
    )
    distortion = np.zeros((4, 1), np.float64)
    flags = (
        cv2.CALIB_USE_INTRINSIC_GUESS
        | cv2.CALIB_RECOMPUTE_EXTRINSIC
        | cv2.CALIB_FIX_SKEW
    )
    return cv2.fisheye.calibrate(
        objects, images, size, matrix, distortion, None, None, flags,
        (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 250, 1e-10),
    )


def pose(object_points: np.ndarray, image_points: np.ndarray,
         matrix: np.ndarray, distortion: np.ndarray):
    """Solve the board pose, then finish the fit by least squares.

    IPPE is an analytic planar solution, not a minimiser.  On a board that is
    small in frame and close to face-on it stops short of the least-squares
    optimum by enough to matter: across the five still frames of one mount
    capture it left 0.22 px of reprojection where the refined fit leaves 0.18,
    and it turned that shortfall into 0.79 degrees of frame-to-frame
    disagreement about the board's out-of-plane tilt against 0.36 refined.
    Refining every branch before choosing also puts this fit where the
    iterative and SQPNP solvers already agree, to within 0.001 degrees.
    """
    undistorted = cv2.fisheye.undistortPoints(
        image_points.reshape(1, -1, 2), matrix, distortion
    ).reshape(-1, 1, 2)
    ok, rvecs, tvecs, _ = cv2.solvePnPGeneric(
        object_points.reshape(-1, 1, 3), undistorted,
        np.eye(3), None, flags=cv2.SOLVEPNP_IPPE,
    )
    if not ok:
        raise RuntimeError("IPPE could not solve board pose")
    candidates = []
    for rvec, tvec in zip(rvecs, tvecs):
        if float(tvec.reshape(-1)[2]) <= 0:
            continue
        rvec, tvec = cv2.solvePnPRefineLM(
            object_points.reshape(-1, 1, 3), undistorted,
            np.eye(3), None, rvec.copy(), tvec.copy(),
        )
        projected, _ = cv2.fisheye.projectPoints(
            object_points.reshape(1, -1, 3), rvec, tvec, matrix, distortion
        )
        residual = np.linalg.norm(
            projected.reshape(-1, 2) - image_points.reshape(-1, 2), axis=1
        )
        candidates.append((float(np.sqrt(np.mean(residual ** 2))), rvec, tvec))
    if not candidates:
        raise RuntimeError("board pose was behind the camera")
    reprojection, rvec, tvec = min(candidates, key=lambda item: item[0])
    rotation = cv2.Rodrigues(rvec)[0]
    return rotation.T, tvec.reshape(-1), reprojection


def rotation_vector(rotation: np.ndarray) -> np.ndarray:
    return cv2.Rodrigues(rotation)[0].reshape(-1)


def analyse(folder: Path) -> dict:
    meta = json.loads((folder / "campaign.json").read_text(encoding="utf-8"))
    if meta.get("status") != "complete":
        raise RuntimeError(f"attempt is {meta.get('status')}; it is not acceptance data")
    board, _ = board_and_detector()
    optics_objects, optics_images, optics_rows = observations(meta["optics"], board)
    size = (int(optics_rows[0]["width"]), int(optics_rows[0]["height"]))
    rms, matrix, distortion, _, _ = fit_intrinsics(
        optics_objects, optics_images, size
    )
    pan_objects, pan_images, pan_rows = observations(meta["samples"], board)
    if len(pan_rows) < 24:
        raise RuntimeError(f"only {len(pan_rows)} usable pan frames; need at least 24")

    cameras, pose_rows = [], []
    for object_points, image_points, row in zip(pan_objects, pan_images, pan_rows):
        camera, translation, reprojection = pose(
            object_points, image_points, matrix, distortion
        )
        cameras.append(camera)
        pose_rows.append({
            "name": row["name"], "pair": row["pair"],
            "approach": row["approach"],
            "commanded_pan_deg": row["commanded_pan_deg"],
            "stationary_duplicate": row["stationary_duplicate"],
            "pose_reprojection_rms_px": reprojection,
            "target_distance_m": float(np.linalg.norm(translation)),
        })

    base_index = min(
        range(len(pose_rows)),
        key=lambda index: (
            abs(pose_rows[index]["commanded_pan_deg"]),
            pose_rows[index]["pair"], pose_rows[index]["stationary_duplicate"],
        ),
    )
    base = cameras[base_index]
    vectors = np.asarray([rotation_vector(camera @ base.T) for camera in cameras])
    _, _, vh = np.linalg.svd(vectors, full_matrices=False)
    axis = vh[0]
    commanded = np.asarray([row["commanded_pan_deg"] for row in pose_rows], float)
    if np.corrcoef(vectors @ axis, commanded)[0, 1] < 0:
        axis = -axis
    measured = np.degrees(vectors @ axis)
    direction = np.asarray([
        1.0 if row["approach"] == "ascending" else -1.0 for row in pose_rows
    ])
    design = np.column_stack([np.ones(len(commanded)), commanded, direction])
    coefficients = np.linalg.lstsq(design, measured, rcond=None)[0]
    predicted = design @ coefficients
    residual = measured - predicted
    for row, value, error in zip(pose_rows, measured, residual):
        row["measured_pan_coordinate_deg"] = float(value)
        row["model_residual_deg"] = float(error)

    gaps = []
    sample_commands = tuple(sorted({int(value) for value in commanded}))
    for command in sample_commands:
        up = measured[(commanded == command) & (direction == 1)]
        down = measured[(commanded == command) & (direction == -1)]
        gaps.append({
            "commanded_pan_deg": command,
            "ascending_minus_descending_deg": float(np.mean(up) - np.mean(down)),
            "ascending_sd_deg": float(np.std(up, ddof=1)),
            "descending_sd_deg": float(np.std(down, ddof=1)),
            "samples_each": int(min(len(up), len(down))),
        })

    duplicate_differences = []
    grouped: dict[tuple, list[float]] = {}
    for row, value in zip(pose_rows, measured):
        key = (row["pair"], row["approach"], row["commanded_pan_deg"])
        grouped.setdefault(key, []).append(float(value))
    for values in grouped.values():
        if len(values) == 2:
            duplicate_differences.append(abs(values[0] - values[1]))

    duplicate_median = float(np.median(duplicate_differences))
    duplicate_p95 = float(np.percentile(duplicate_differences, 95))
    reference_resolves = duplicate_median <= 0.25 and duplicate_p95 <= 0.75
    approach_models = {}
    for name, sign in (("ascending", 1.0), ("descending", -1.0)):
        mask = direction == sign
        slope, intercept = np.polyfit(commanded[mask], measured[mask], 1)
        errors = measured[mask] - (intercept + slope * commanded[mask])
        approach_models[name] = {
            "gain_actual_per_commanded": float(slope),
            "gain_error_percent": float((slope - 1.0) * 100),
            "residual_rms_deg": float(np.sqrt(np.mean(errors ** 2))),
            "absolute_residual_p95_deg": float(np.percentile(np.abs(errors), 95)),
        }

    candidate = meta.get("candidate")
    candidate_pass = None
    if candidate == "consistent-ascending":
        chosen = approach_models["ascending"]
        candidate_pass = (
            reference_resolves
            and abs(chosen["gain_error_percent"]) <= 1.0
            and chosen["absolute_residual_p95_deg"] <= 0.5
        )

    result = {
        "analysed_at": now(),
        "source": str(folder),
        "optics": {
            "accepted_frames": len(optics_rows),
            "total_frames": len(meta["optics"]),
            "fisheye_reprojection_rms_px": float(rms),
            "camera_matrix": matrix.tolist(),
            "distortion": distortion.reshape(-1).tolist(),
        },
        "pan": {
            # Carried through from the campaign, because a pan result is only
            # true of the tilt it was measured at and two campaigns at
            # different tilts must not be compared as if they were repeats.
            # Older campaigns predate the option and were all taken at zero.
            "sweep_tilt_deg": int(meta.get("pan_sweep_tilt_deg") or 0),
            "accepted_frames": len(pose_rows),
            "total_frames": len(meta["samples"]),
            "dominant_axis_in_board_frame": axis.tolist(),
            "gain_actual_per_commanded": float(coefficients[1]),
            "gain_error_percent": float((coefficients[1] - 1.0) * 100),
            "ascending_minus_descending_model_deg": float(2 * coefficients[2]),
            "model_residual_rms_deg": float(np.sqrt(np.mean(residual ** 2))),
            "stationary_duplicate_difference_median_deg": duplicate_median,
            "stationary_duplicate_difference_p95_deg": duplicate_p95,
            "stationary_duplicate_difference_max_deg": float(
                np.max(duplicate_differences)
            ),
            "direction_gap_by_command": gaps,
            "approach_models": approach_models,
            "poses": pose_rows,
        },
        "verdict": {
            "reference_resolves_task_error": reference_resolves,
            "status": "development measurement" if reference_resolves else "inconclusive",
            "rule": "stationary duplicate median <= 0.25 deg and p95 <= 0.75 deg",
            "candidate": candidate,
            "candidate_pass": candidate_pass,
            "candidate_rule": (
                "ascending-only gain error <= 1.0% and absolute residual p95 <= 0.5 deg"
                if candidate == "consistent-ascending" else None
            ),
        },
    }
    (folder / "analysis.json").write_text(
        json.dumps(result, indent=2) + "\n", encoding="utf-8"
    )
    return result


def selftest() -> int:
    board, _ = board_and_detector()
    points = np.asarray(board.getChessboardCorners(), np.float64).reshape(1, -1, 3)
    size = (640, 480)
    truth_matrix = np.array(
        [[285.0, 0, 316.0], [0, 286.0, 227.0], [0, 0, 1]], np.float64
    )
    truth_distortion = np.array([[-0.02], [0.01], [-0.004], [0.001]], np.float64)
    objects, images = [], []
    for yaw in (-22, 0, 22):
        for pitch in (-12, 0, 12):
            rvec = np.radians([pitch, yaw, 3.0]).reshape(3, 1)
            tvec = np.array([[-0.11], [-0.08], [0.62]], np.float64)
            projected, _ = cv2.fisheye.projectPoints(
                points, rvec, tvec, truth_matrix, truth_distortion
            )
            objects.append(points.copy())
            images.append(projected)
    rms, matrix, _, _, _ = fit_intrinsics(objects, images, size)
    focal_error = max(
        abs(matrix[0, 0] - truth_matrix[0, 0]),
        abs(matrix[1, 1] - truth_matrix[1, 1]),
    )
    ok = rms < 1e-3 and focal_error < 0.5
    print(f"synthetic fisheye RMS {rms:.6f} px; focal error {focal_error:.3f} px")
    print("selftest ok" if ok else "SELFTEST FAILED")
    return 0 if ok else 1


def print_result(result: dict) -> None:
    optics, pan = result["optics"], result["pan"]
    print(
        f"optics: {optics['accepted_frames']}/{optics['total_frames']} frames, "
        f"{optics['fisheye_reprojection_rms_px']:.3f} px RMS"
    )
    print(
        f"pan at tilt {pan.get('sweep_tilt_deg', 0):+d}: "
        f"{pan['accepted_frames']}/{pan['total_frames']} frames; "
        f"gain error {pan['gain_error_percent']:+.2f}%; "
        f"ascending-descending {pan['ascending_minus_descending_model_deg']:+.2f} deg"
    )
    print(
        "stationary-frame difference: "
        f"median {pan['stationary_duplicate_difference_median_deg']:.3f} deg, "
        f"p95 {pan['stationary_duplicate_difference_p95_deg']:.3f} deg, "
        f"max {pan['stationary_duplicate_difference_max_deg']:.3f} deg"
    )
    print(f"verdict: {result['verdict']['status']}")
    if result["verdict"]["candidate"]:
        chosen = pan["approach_models"]["ascending"]
        print(
            f"candidate {result['verdict']['candidate']}: "
            f"gain error {chosen['gain_error_percent']:+.2f}%, "
            f"residual p95 {chosen['absolute_residual_p95_deg']:.3f} deg; "
            + ("PASS" if result["verdict"]["candidate_pass"] else "FAIL")
        )


def main() -> int | str:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("folder", nargs="?", type=Path)
    parser.add_argument("--rover", default=None, metavar="HOST[:PORT]")
    parser.add_argument("--settle", type=float, default=SETTLE_S, metavar="SECONDS")
    parser.add_argument("--size", default="1280x960", metavar="WIDTHxHEIGHT")
    parser.add_argument(
        "--angles", default=",".join(str(value) for value in PAN_SAMPLES),
        metavar="DEG,...", help="ordered ascending pan samples",
    )
    parser.add_argument("--candidate", choices=("consistent-ascending",))
    parser.add_argument(
        "--tilt", type=int, default=0, metavar="DEG",
        help="tilt the pan sweeps are taken at; the pan result belongs to it",
    )
    parser.add_argument("--fit-only", action="store_true")
    parser.add_argument("--selftest", action="store_true")
    args = parser.parse_args()
    if args.selftest:
        return selftest()
    if args.folder is None:
        parser.error("name a new capture folder, an existing folder with --fit-only, or --selftest")
    try:
        width, separator, height = args.size.lower().partition("x")
        if not separator:
            raise ValueError
        size = (int(width), int(height))
    except ValueError:
        parser.error("--size must look like 1280x960")
    try:
        pan_samples = tuple(int(value) for value in args.angles.split(","))
        if len(pan_samples) < 3 or tuple(sorted(set(pan_samples))) != pan_samples:
            raise ValueError
        if pan_samples[0] <= -30 or pan_samples[-1] >= 30:
            raise ValueError
    except ValueError:
        parser.error("--angles must be at least three unique ascending integers inside -30..30")
    # The gimbal's own travel, and the sweeps must stay well inside it: the
    # overshoot needs room and a servo against its stop is not a measurement.
    if not -20 <= args.tilt <= 80:
        parser.error("--tilt must be between -20 and 80 degrees")
    if not args.fit_only:
        rover = RoverClient(args.rover) if args.rover else discover()
        if rover is None or not rover.probe():
            return "no rover daemon found; name one with --rover"
        rover.timeout = 30.0
        try:
            capture(args.folder, rover, args.settle, size, pan_samples,
                    args.candidate, args.tilt)
        finally:
            rover.close()
    result = analyse(args.folder)
    print_result(result)
    return 0


if __name__ == "__main__":
    sys.exit(main())
