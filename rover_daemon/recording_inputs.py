"""The declared calibration a recording session hands Perception: `inputs.json`.

Perception's system may use the rover's calibration as constants, never refit it
(docs/plans/recording-for-perception.md). So each value here is read from the
code that uses it on the rover -- the lens table, the gimbal's limits and
approach, the OAK's mount, the odometry file -- and only the provenance is
written down: which file, the commit that set it, and the days it was measured,
from docs/progress. A value refitted later changes the code, and the next
session's file with it; its provenance here is then out of date and says so to
whoever compares the commit.

Built once per session, at its start. Anything it cannot read stops the session
from starting: a recording whose calibration is a guess is worse than none.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
from typing import Any

#: base_node's odometry file, which is runtime state outside git and so is cited
#: by its hash.
ODOMETRY_FILE = os.path.expanduser("~/ugv/odometry.json")

#: (source file, commit, days measured) for each entry. The commits are the ones
#: that set the values the code holds today; the days are the progress entries'.
PROVENANCE = {
    "fisheye": ("face_tracking/lens.py", "05e046b", ["2026-09-30", "2026-10-02"]),
    "gimbal": ("rover_daemon/rover.py", "583756b", ["2026-09-07", "2026-10-05", "2026-10-09"]),
    "oak_mount": ("world_state/oak.py", "05e046b", ["2026-09-30"]),
    "oak_imu_axes": ("usb_cameras/pan_candidate_2026-10-09.json", "038ff66",
                     ["2026-09-30", "2026-10-09"]),
    "chassis_transforms": ("ros_nav/base_node.py", "9e3fe73", ["2026-08-17", "2026-10-01"]),
    "odometry": ("odometry.json", None, ["2026-08-31", "2026-10-10"]),
    "board_scales": ("rover_daemon/board_link.py", "9e3fe73", ["2026-08-31", "2026-10-10"]),
}


def _entry(name: str, values: dict[str, Any], sha256: str | None = None) -> dict[str, Any]:
    source, commit, measured = PROVENANCE[name]
    return {"source": source, "commit": commit, "sha256": sha256,
            "measured": measured, "values": values}


def _sha256(path: str) -> str:
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def build(colour_lens: dict[str, Any] | None = None) -> dict[str, Any]:
    """The whole file. `colour_lens` is the OAK colour camera's pinhole as the
    depth service publishes it (width, height, fx, fy, cx, cy); without it the
    values the service has published since 2026-09-30 stand in."""
    import aiming
    import lens
    import rover as rover_mod
    from world_state import locate, oak

    scale, (b1, b2), centre = lens.LENS[(640, 480)]
    fisheye = {
        "model": "equidistant_even_polynomial", "width_px": 640, "height_px": 480,
        "scale_rad_per_px": math.radians(scale / 60.0), "b1": b1, "b2": b2,
        "centre_px": list(centre),
        "elevation_bias_at_rest_rad": math.radians(locate.ELEVATION_BIAS_DEG[aiming.REST_TILT_DEG]),
        "elevation_bias_sigma_rad": math.radians(locate.ELEVATION_SIGMA_DEG),
    }
    gimbal = {
        # The firmware maps 0-360 commanded degrees onto 0-4095 servo counts.
        "degrees_per_commanded_degree": 1.0,
        "counts_per_turn": 4095,
        "t134_counts_per_degree": aiming.COUNTS_PER_DEGREE,
        "pan_limits_deg": [-float(aiming.PAN_LIMIT), float(aiming.PAN_LIMIT)],
        "tilt_limits_deg": [float(aiming.TILT_LIMITS[0]), float(aiming.TILT_LIMITS[1])],
        "rest_deg": [0.0, float(aiming.REST_TILT_DEG)],
        "rest_approach_below_deg": float(rover_mod.APPROACH_UNDERSHOOT_DEG),
        "settle_min_s": rover_mod.APPROACH_SETTLE_S,
        "settle_base_s": rover_mod.APPROACH_SWING_FLOOR_S,
        "settle_per_degree_s": 1.0 / rover_mod.APPROACH_SWING_DPS,
        # Measured, 2026-09-07 to 2026-10-09; nothing in the code applies them.
        "backlash_deg": [1.6, 1.8],
        "gain_error_within_20deg": {"tilt_0": [-0.0034, 0.0048], "tilt_20": [-0.0092, -0.0104]},
        # Filled at commissioning, from frames, with the session it came from.
        "speed_meaning": None,
    }
    m = oak.MOUNT
    pinhole = colour_lens or {"width_px": 640, "height_px": 360, "fx": 500.3,
                              "fy": 500.2, "cx": 321.2, "cy": 190.7}
    oak_mount = {
        "yaw_rad": math.radians(m.yaw_deg), "pitch_rad": math.radians(m.pitch_deg),
        "roll_rad": math.radians(m.roll_deg),
        "translation_m": [m.forward_m, m.left_m, m.up_m],
        # Two runs fitted apart differed by 13 mm and 0.2 degrees (2026-09-30).
        "rotation_sigma_rad": math.radians(0.1), "translation_sigma_m": 0.01,
        "colour_pinhole": pinhole,
        "depth_scale": 0.5,
    }
    # Both as the gyro's reading over the truth: it over-reads pan by 1.8%
    # (closing the circle, 2026-10-09) and reads tilt 3.8% low (2026-09-30).
    oak_imu_axes = {"axes_to_camera": None, "gyro_scale_pan": 1.01824,
                    "gyro_scale_tilt": 0.962}
    chassis = {
        "base_to_laser": [0.0, 0.0, 0.0], "laser_zero": "left", "laser_direction": "cw",
        "lens_ahead_of_lidar_m": 0.02, "lens_height_tilt0_m": locate.CAMERA_HEIGHT_M,
        "lidar_plane_height_m": 0.20, "board_imu_to_base": None, "gz_positive": "left_turn",
    }
    with open(ODOMETRY_FILE, encoding="utf-8") as f:
        stored = json.load(f)
    odometry = {
        "gyro_units_per_dps": float(stored["gyro_lsb_per_dps"]),
        "encoder_counts_per_m": float(stored["ticks_per_metre"]),
        "stamp_offset_s": -0.048,
        "track_width_m": None, "slip_factor": None,
    }
    board = {
        "encoder_zeroed_on_board_reboot": True,
        "gz_units_per_dps": float(stored["gyro_lsb_per_dps"]),
        "gx_gy_units_per_dps": None,
        # Read off az at rest (about 8,400), not measured.
        "accel_units_per_g": 8192.0, "accel_scale_measured": False,
        "mag_units_per_tesla": None,
        "volt_per_unit": 0.01,
    }
    entries = {
        "fisheye": _entry("fisheye", fisheye),
        "gimbal": dict(_entry("gimbal", gimbal), speed_meaning_session=None),
        "oak_mount": _entry("oak_mount", oak_mount),
        "oak_imu_axes": _entry("oak_imu_axes", oak_imu_axes),
        "chassis_transforms": _entry("chassis_transforms", chassis),
        "odometry": _entry("odometry", odometry, sha256=_sha256(ODOMETRY_FILE)),
        "board_scales": _entry("board_scales", board),
    }
    # A calibration changed between sessions is a new version: name it by what
    # it holds, so two sessions with the same values share one.
    digest = hashlib.sha256(json.dumps(entries, sort_keys=True).encode()).hexdigest()[:12]
    return {"schema": "rover-recording", "version": 1, "file": "inputs",
            "calibration_version": "cal-" + digest, **entries}
