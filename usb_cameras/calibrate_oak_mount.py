#!/usr/bin/env python3
"""Measure the fixed OAK mount against the independently fitted gimbal camera.

The same printed ChArUco board is solved in both cameras.  A development capture
produces a candidate; a second target distance can compare against that candidate.
No result is written into world_state/oak.py automatically.

    python usb_cameras/calibrate_oak_mount.py captures/.../oak-mount-dev \
      --gimbal-analysis captures/.../held-out-01/analysis.json \
      --rover 192.168.1.80:8769
    python usb_cameras/calibrate_oak_mount.py captures/.../oak-mount-held-out \
      --gimbal-analysis captures/.../held-out-01/analysis.json \
      --rover 192.168.1.80:8769 --compare captures/.../oak-mount-dev/mount-analysis.json
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
import time
from itertools import product
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE.parent / "voice_chat"))

import calibrate_gimbal as gimbal  # noqa: E402
from rover_tools import RoverClient, discover  # noqa: E402
from world_state.bench_oak import angles_of, chassis_from_optical  # noqa: E402


FRAMES = 5
MIN_GIMBAL_CORNERS = 40
MIN_OAK_CORNERS = 45
MAX_REPROJECTION_RMS_PX = 0.5
MAX_ANGLE_RANGE_DEG = 0.75
MAX_OFFSET_RANGE_M = 0.015
MAX_PINHOLE_RAY_ERROR_DEG = 0.75
MAX_HELD_OUT_ANGLE_CHANGE_DEG = 0.75
MAX_HELD_OUT_OFFSET_CHANGE_M = 0.015
MIN_HELD_OUT_DISTANCE_CHANGE_M = 0.10


def now() -> str:
    return gimbal.now()


def save_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")


def ssh_bytes(host: str, command: str) -> bytes:
    return subprocess.check_output(["ssh", host, command], timeout=40)


def oak_power_on(host: str) -> dict:
    raw = ssh_bytes(
        host,
        "curl -sS -X POST -H 'Content-Type: application/json' "
        "--data-binary '{\"on\":true}' http://127.0.0.1:8770/power",
    )
    return json.loads(raw)


def oak_health(host: str) -> dict:
    return json.loads(ssh_bytes(host, "curl -sS http://127.0.0.1:8770/health"))


def oak_frame(host: str) -> bytes:
    return ssh_bytes(host, "curl -sS http://127.0.0.1:8770/frame")


def wait_for_oak(host: str) -> dict:
    last = None
    for _ in range(20):
        try:
            last = oak_health(host)
            if last.get("ok") and (last.get("colour") or {}).get("distortion"):
                return last
        except (OSError, subprocess.SubprocessError, ValueError):
            pass
        time.sleep(2)
    raise RuntimeError(f"OAK did not become ready: {last}")


def save_oak_jpeg(folder: Path, number: int, data: bytes, detector) -> dict:
    path = folder / f"oak-{number}.jpg"
    path.write_bytes(data)
    frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        raise RuntimeError(f"OpenCV could not decode {path}")
    found = detect_best(frame, detector)
    if found["charuco_corners"] < 4:
        raise RuntimeError(f"only {found['charuco_corners']} corners in {path}")
    return {
        "file": path.name,
        "sha256": hashlib.sha256(data).hexdigest(),
        "bytes": len(data),
        "width": int(frame.shape[1]),
        "height": int(frame.shape[0]),
        "detection": found,
    }


def detect_best(frame: np.ndarray, detector) -> dict:
    """Use whichever of native or enlarged detection finds more board corners.

    Enlargement recovers this board in a 640 x 480 frame, but at 1280 x 960 its
    resampling can erase marker cells: one preserved frame fell from 54 corners
    at native resolution to 39 at 3x.  Pose fitting should use the observation
    with more printed reference, not a fixed preprocessing scale.
    """
    best = gimbal.detect(frame, detector)
    corners, ids, marker_corners, marker_ids = detector.detectBoard(frame)
    native_count = 0 if ids is None else len(ids)
    if native_count <= best["charuco_corners"]:
        return best
    return {
        "markers": 0 if marker_ids is None else len(marker_ids),
        "charuco_corners": native_count,
        "ids": [int(value) for value in ids.reshape(-1)],
        "image_points_px": np.asarray(corners, float).reshape(-1, 2).round(5).tolist(),
    }


def capture(folder: Path, rover: RoverClient, oak_host: str, settle: float) -> None:
    if folder.exists():
        raise RuntimeError(f"refusing to overwrite existing attempt: {folder}")
    folder.mkdir(parents=True)
    _, detector = gimbal.board_and_detector()
    status = rover.call("tracking_status", {})
    meta = {
        "schema": 1,
        "status": "running",
        "started_at": now(),
        "repository_head": gimbal.repository_head(),
        "opencv_version": cv2.__version__,
        "rover": rover.describe(),
        "oak_host": oak_host,
        "settle_s": settle,
        "approach": "ascending",
        "gimbal": [],
        "oak": [],
        "health": None,
        "failure": None,
    }
    save_json(folder / "mount-campaign.json", meta)
    if not status.get("ok"):
        raise RuntimeError(f"could not read gimbal state: {status}")
    if status.get("tracking"):
        raise RuntimeError("face tracking is active; no campaign was started")

    try:
        sent = rover.call("look_at", {"pan": -10, "tilt": 0})
        if not sent.get("ok"):
            raise RuntimeError(f"gimbal refused approach move: {sent}")
        time.sleep(settle)
        sent = rover.call("look_at", {"pan": 0, "tilt": 0})
        if not sent.get("ok"):
            raise RuntimeError(f"gimbal refused zero: {sent}")
        time.sleep(settle)
        gimbal.capture_jpeg(rover, (1280, 960))
        for number in range(1, FRAMES + 1):
            data, reply = gimbal.capture_jpeg(rover, (1280, 960))
            row = gimbal.save_frame(
                folder, f"gimbal-{number}", data, reply, detector
            )
            frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            row["detection"] = detect_best(frame, detector)
            meta["gimbal"].append(row)
            save_json(folder / "mount-campaign.json", meta)
            time.sleep(0.25)

        power = oak_power_on(oak_host)
        if not power.get("ok"):
            raise RuntimeError(f"OAK refused power-on: {power}")
        meta["health"] = wait_for_oak(oak_host)
        for number in range(1, FRAMES + 1):
            row = save_oak_jpeg(folder, number, oak_frame(oak_host), detector)
            meta["oak"].append(row)
            save_json(folder / "mount-campaign.json", meta)
            time.sleep(0.5)
        meta["status"] = "complete"
    except Exception as error:
        meta["status"] = "invalid"
        meta["failure"] = repr(error)
        raise
    finally:
        meta["returned_to"] = rover.call("look_at", {"pan": 0, "tilt": 0})
        meta["finished_at"] = now()
        save_json(folder / "mount-campaign.json", meta)


def opencv_pose(objects: np.ndarray, images: np.ndarray, matrix: np.ndarray,
                distortion: np.ndarray):
    ok, rvecs, tvecs, _ = cv2.solvePnPGeneric(
        objects.reshape(-1, 1, 3), images.reshape(-1, 1, 2), matrix,
        distortion, flags=cv2.SOLVEPNP_IPPE,
    )
    if not ok:
        raise RuntimeError("OAK IPPE could not solve board pose")
    candidates = []
    for rvec, tvec in zip(rvecs, tvecs):
        projected, _ = cv2.projectPoints(
            objects, rvec, tvec, matrix, distortion
        )
        residual = np.linalg.norm(projected.reshape(-1, 2) - images, axis=1)
        if float(tvec.reshape(-1)[2]) > 0:
            candidates.append((float(np.sqrt(np.mean(residual ** 2))), rvec, tvec))
    if not candidates:
        raise RuntimeError("board pose was behind the OAK")
    reprojection, rvec, tvec = min(candidates, key=lambda item: item[0])
    return cv2.Rodrigues(rvec)[0].T, tvec.reshape(-1), reprojection


def detected_pose(path: Path, board, detector, matrix: np.ndarray,
                  distortion: np.ndarray, fisheye: bool) -> dict:
    frame = cv2.imread(str(path))
    if frame is None:
        raise RuntimeError(f"could not read {path}")
    found = detect_best(frame, detector)
    ids = np.asarray(found["ids"], dtype=np.int32)
    images = np.asarray(found["image_points_px"], dtype=np.float64)
    objects = np.asarray(board.getChessboardCorners(), dtype=np.float64)[ids]
    if fisheye:
        camera, translation, reprojection = gimbal.pose(
            objects, images, matrix, distortion
        )
    else:
        camera, translation, reprojection = opencv_pose(
            objects, images, matrix, distortion
        )
    return {
        "file": path.name,
        "markers": found["markers"],
        "corners": found["charuco_corners"],
        "ids": found["ids"],
        "reprojection_rms_px": reprojection,
        "target_distance_m": float(np.linalg.norm(translation)),
        "camera": camera,
        "translation": translation,
    }


def stats(values) -> dict:
    values = np.asarray(values, dtype=float)
    return {
        "median": float(np.median(values)),
        "sd": float(np.std(values)),
        "min": float(np.min(values)),
        "max": float(np.max(values)),
        "range": float(np.ptp(values)),
    }


def pinhole_ray_error(matrix: np.ndarray, distortion: np.ndarray,
                      width: int, height: int) -> dict:
    points = np.asarray([
        (x, y) for y in np.linspace(0, height, 13)
        for x in np.linspace(0, width, 17)
    ], dtype=np.float64).reshape(-1, 1, 2)
    calibrated = cv2.undistortPoints(points, matrix, distortion).reshape(-1, 2)
    pinhole = cv2.undistortPoints(points, matrix, np.zeros(5)).reshape(-1, 2)
    calibrated = np.column_stack([calibrated, np.ones(len(calibrated))])
    pinhole = np.column_stack([pinhole, np.ones(len(pinhole))])
    calibrated /= np.linalg.norm(calibrated, axis=1, keepdims=True)
    pinhole /= np.linalg.norm(pinhole, axis=1, keepdims=True)
    errors = np.degrees(np.arccos(np.clip(
        np.sum(calibrated * pinhole, axis=1), -1.0, 1.0
    )))
    return {"median_deg": float(np.median(errors)),
            "p95_deg": float(np.percentile(errors, 95)),
            "max_deg": float(np.max(errors))}


def analyse(folder: Path, gimbal_analysis: Path,
            compare: Path | None = None) -> dict:
    board, detector = gimbal.board_and_detector()
    calibration = json.loads(gimbal_analysis.read_text(encoding="utf-8"))
    kg = np.asarray(calibration["optics"]["camera_matrix"], dtype=np.float64)
    dg = np.asarray(calibration["optics"]["distortion"], dtype=np.float64)
    health = json.loads((folder / "health.json").read_text(encoding="utf-8")) \
        if (folder / "health.json").exists() else json.loads(
            (folder / "mount-campaign.json").read_text(encoding="utf-8"))["health"]
    lens = health["colour"]["intrinsics"]
    ko = np.asarray([[lens["fx"], 0, lens["cx"]],
                     [0, lens["fy"], lens["cy"]],
                     [0, 0, 1]], dtype=np.float64)
    do = np.asarray(health["colour"]["distortion"], dtype=np.float64)

    gimbals = [detected_pose(path, board, detector, kg, dg, True)
               for path in sorted(folder.glob("gimbal-[1-5].jpg"))]
    oaks = [detected_pose(path, board, detector, ko, do, False)
            for path in sorted(folder.glob("oak-[1-5].jpg"))]
    if len(gimbals) != FRAMES or len(oaks) != FRAMES:
        raise RuntimeError(f"need {FRAMES} frames from each camera")

    fits = []
    for one_gimbal, one_oak in product(gimbals, oaks):
        g_to_board = one_gimbal["camera"]
        o_to_board = one_oak["camera"]
        rotation = g_to_board.T @ o_to_board
        angle = angles_of(np, rotation, 0.0, 0.0)
        g_position = -g_to_board @ one_gimbal["translation"]
        o_position = -o_to_board @ one_oak["translation"]
        optical_offset = g_to_board.T @ (o_position - g_position)
        offset = chassis_from_optical(np, 0.0, 0.0) @ optical_offset
        fits.append({
            **angle,
            "forward_m": float(offset[0]),
            "left_m": float(offset[1]),
            "up_m": float(offset[2]),
        })

    keys = ("yaw_deg", "pitch_deg", "roll_deg",
            "forward_m", "left_m", "up_m")
    transform = {key: stats([row[key] for row in fits]) for key in keys}
    ray_error = pinhole_ray_error(ko, do, lens["width"], lens["height"])
    gates = {
        "gimbal_board_coverage": min(row["corners"] for row in gimbals)
        >= MIN_GIMBAL_CORNERS,
        "oak_board_coverage": min(row["corners"] for row in oaks)
        >= MIN_OAK_CORNERS,
        "reprojection": max(row["reprojection_rms_px"]
                            for row in gimbals + oaks) <= MAX_REPROJECTION_RMS_PX,
        "angular_repeatability": max(transform[key]["range"] for key in
                                     ("yaw_deg", "pitch_deg", "roll_deg"))
        <= MAX_ANGLE_RANGE_DEG,
        "offset_repeatability": max(transform[key]["range"] for key in
                                    ("forward_m", "left_m", "up_m"))
        <= MAX_OFFSET_RANGE_M,
        "runtime_pinhole_model": ray_error["max_deg"]
        <= MAX_PINHOLE_RAY_ERROR_DEG,
    }
    comparison = None
    if compare is not None:
        prior = json.loads(compare.read_text(encoding="utf-8"))
        delta = {key: transform[key]["median"]
                 - prior["transform"][key]["median"] for key in keys}
        current_distance = float(np.median([
            row["target_distance_m"] for row in gimbals
        ]))
        prior_distance = float(np.median([
            row["target_distance_m"] for row in prior["frames"]["gimbal"]
        ]))
        comparison = {
            "source": str(compare),
            "delta": delta,
            "development_pass": bool(prior["verdict"]["all_gates_pass"]),
            "gimbal_target_distance_m": current_distance,
            "development_gimbal_target_distance_m": prior_distance,
            "target_distance_change_m": abs(current_distance - prior_distance),
            "distance_change_pass": abs(current_distance - prior_distance)
            >= MIN_HELD_OUT_DISTANCE_CHANGE_M,
            "angle_change_pass": max(abs(delta[key]) for key in
                                     ("yaw_deg", "pitch_deg", "roll_deg"))
            <= MAX_HELD_OUT_ANGLE_CHANGE_DEG,
            "offset_change_pass": max(abs(delta[key]) for key in
                                      ("forward_m", "left_m", "up_m"))
            <= MAX_HELD_OUT_OFFSET_CHANGE_M,
        }
        gates["held_out_agreement"] = (
            comparison["development_pass"] and comparison["angle_change_pass"]
            and comparison["offset_change_pass"]
            and comparison["distance_change_pass"]
        )

    result = {
        "analysed_at": now(),
        "source": str(folder),
        "gimbal_analysis": str(gimbal_analysis),
        "frames": {
            "gimbal": [{k: v for k, v in row.items()
                        if k not in ("camera", "translation")} for row in gimbals],
            "oak": [{k: v for k, v in row.items()
                     if k not in ("camera", "translation")} for row in oaks],
        },
        "transform": transform,
        "runtime_pinhole_ray_error": ray_error,
        "comparison": comparison,
        "gates": gates,
        "verdict": {
            "status": "pass" if all(gates.values()) else "inconclusive",
            "all_gates_pass": all(gates.values()),
            "acceptance": "held-out" if compare is not None else "development",
        },
    }
    save_json(folder / "mount-analysis.json", result)
    return result


def print_result(result: dict) -> None:
    transform = result["transform"]
    print("mount candidate: " + ", ".join(
        f"{key}={transform[key]['median']:+.3f}" for key in
        ("yaw_deg", "pitch_deg", "roll_deg", "forward_m", "left_m", "up_m")
    ))
    print("repeatability range: " + ", ".join(
        f"{key}={transform[key]['range']:.3f}" for key in
        ("yaw_deg", "pitch_deg", "roll_deg", "forward_m", "left_m", "up_m")
    ))
    ray = result["runtime_pinhole_ray_error"]
    print(f"pinhole ray approximation: p95 {ray['p95_deg']:.3f} deg, "
          f"max {ray['max_deg']:.3f} deg")
    failed = [name for name, passed in result["gates"].items() if not passed]
    print(f"verdict: {result['verdict']['status']}" +
          (f" ({', '.join(failed)} failed)" if failed else ""))


def main() -> int | str:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("folder", type=Path)
    parser.add_argument("--gimbal-analysis", required=True, type=Path)
    parser.add_argument("--compare", type=Path)
    parser.add_argument("--rover", metavar="HOST[:PORT]")
    parser.add_argument("--oak-host", default="orin")
    parser.add_argument("--settle", type=float, default=gimbal.SETTLE_S)
    parser.add_argument("--fit-only", action="store_true")
    args = parser.parse_args()
    if not args.fit_only:
        rover = RoverClient(args.rover) if args.rover else discover()
        if rover is None or not rover.probe():
            return "no rover daemon found; name one with --rover"
        rover.timeout = 30.0
        try:
            capture(args.folder, rover, args.oak_host, args.settle)
        finally:
            rover.close()
    result = analyse(args.folder, args.gimbal_analysis, args.compare)
    print_result(result)
    return 0


if __name__ == "__main__":
    sys.exit(main())
