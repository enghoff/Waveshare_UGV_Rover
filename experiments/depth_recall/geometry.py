"""Where an OAK depth pixel is on the map, and where a map point lands in an OAK
picture, for the blind depth-recall experiment.

The chain is the one the rover already uses, written out with numpy so a whole
depth map goes through at once:

    OAK sensor (x right, y down, z out)       the device's own lens, a pinhole
      -> gimbal camera frame (forward, left, up)   world_state/oak.py MOUNT
      -> chassis frame                              the gimbal's pan and tilt
      -> map frame                                  the look's stored pose

Two numbers are this experiment's, not the rover's: the gimbal camera sits
`CAMERA_FORWARD_M` ahead of the pose's origin (the lidar's spin centre) and
`CAMERA_HEIGHT_M` above the floor, both taped (R-WS-11). The gimbal's own pivots
are not modelled, as they are not anywhere on the rover; the camera's swing about
them is about a centimetre at the tilts used here.
"""
from __future__ import annotations

import gzip
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(REPO, "world_state"))
sys.path.insert(0, os.path.join(REPO, "face_tracking"))

import oak  # noqa: E402  (world_state/oak.py)
import lens as fisheye  # noqa: E402  (face_tracking/lens.py)

CAMERA_FORWARD_M = 0.02
CAMERA_HEIGHT_M = 0.235


def read_header(path: str) -> dict:
    """The depth file's description, without reading the depth itself."""
    with gzip.open(path, "rb") as handle:
        head = handle.read(4)
        size = int.from_bytes(head, "little")
        return json.loads(handle.read(size).decode())


def read_depth(path: str) -> tuple[np.ndarray, dict]:
    """Depth in metres along the optical axis (0 where invalid), and the header."""
    with gzip.open(path, "rb") as handle:
        body = handle.read()
    size = int.from_bytes(body[:4], "little")
    header = json.loads(body[4:4 + size].decode())
    millimetres = np.frombuffer(body[4 + size:], dtype=np.uint16)
    depth = millimetres.reshape(header["height"], header["width"]).astype(np.float64)
    return depth / 1000.0, header


def _turn_matrix(yaw_right_deg: float, pitch_up_deg: float) -> np.ndarray:
    """oak._turn as a matrix on (forward, left, up): pitch up, then yaw right."""
    p = math.radians(pitch_up_deg)
    pitch = np.array([[math.cos(p), 0.0, -math.sin(p)],
                      [0.0, 1.0, 0.0],
                      [math.sin(p), 0.0, math.cos(p)]])
    y = math.radians(-yaw_right_deg)
    yaw = np.array([[math.cos(y), -math.sin(y), 0.0],
                    [math.sin(y), math.cos(y), 0.0],
                    [0.0, 0.0, 1.0]])
    return yaw @ pitch


def _sensor_to_flu(mount) -> np.ndarray:
    """OAK sensor (right, down, out) to gimbal-camera (forward, left, up), as
    oak.ray_at does it: unroll, swap axes, then the mount's yaw and pitch."""
    r = math.radians(mount.roll_deg)
    unroll = np.array([[math.cos(r), math.sin(r), 0.0],
                       [-math.sin(r), math.cos(r), 0.0],
                       [0.0, 0.0, 1.0]])
    swap = np.array([[0.0, 0.0, 1.0],     # forward = out
                     [-1.0, 0.0, 0.0],    # left = -right
                     [0.0, -1.0, 0.0]])   # up = -down
    return _turn_matrix(mount.yaw_deg, mount.pitch_deg) @ swap @ unroll


def camera_pose(pose: dict, pan_deg: float, tilt_deg: float, when: float):
    """(R, t): OAK sensor coordinates to map coordinates (x, y, z up)."""
    mount = oak.mount_at(when)
    to_flu = _sensor_to_flu(mount)
    offset = np.array([mount.forward_m, mount.left_m, mount.up_m])
    gimbal = _turn_matrix(pan_deg, tilt_deg) if mount.on_gimbal else np.eye(3)
    h = math.radians(pose["heading_deg"])
    heading = np.array([[math.cos(h), -math.sin(h), 0.0],
                        [math.sin(h), math.cos(h), 0.0],
                        [0.0, 0.0, 1.0]])
    rotation = heading @ gimbal @ to_flu
    camera = np.array([CAMERA_FORWARD_M, 0.0, CAMERA_HEIGHT_M])
    translation = (np.array([pose["x_m"], pose["y_m"], 0.0])
                   + heading @ (camera + gimbal @ offset))
    return rotation, translation


def pixel_grid(header: dict) -> tuple[np.ndarray, np.ndarray]:
    """Normalised pinhole coordinates of every depth pixel's centre."""
    lens = header["lens"]
    width, height = header["width"], header["height"]
    u = (np.arange(width) + 0.5) / width * lens["width"]
    v = (np.arange(height) + 0.5) / height * lens["height"]
    x = (u - lens["cx"]) / lens["fx"]
    y = (v - lens["cy"]) / lens["fy"]
    return np.meshgrid(x, y)


def to_map(depth: np.ndarray, header: dict, rotation, translation) -> np.ndarray:
    """Map points (N, 3) for every valid pixel, row-major order, plus NaN rows
    where the depth is invalid."""
    x, y = pixel_grid(header)
    sensor = np.stack([x * depth, y * depth, depth], axis=-1).reshape(-1, 3)
    points = sensor @ rotation.T + translation
    points[depth.reshape(-1) <= 0] = np.nan
    return points


def project(points: np.ndarray, header: dict, rotation, translation):
    """Map points into a camera: (column, row, depth) arrays in the depth map's
    own pixel grid; columns and rows are floats, depth along the axis."""
    sensor = (points - translation) @ rotation   # inverse of R @ s + t
    lens = header["lens"]
    z = sensor[:, 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        u = sensor[:, 0] / z * lens["fx"] + lens["cx"]
        v = sensor[:, 1] / z * lens["fy"] + lens["cy"]
    column = u / lens["width"] * header["width"] - 0.5
    row = v / lens["height"] * header["height"] - 0.5
    return column, row, z


def fisheye_pixel(direction_flu: np.ndarray, width=640, height=480):
    """Gimbal-camera directions (forward, left, up) to fisheye pixels."""
    lens = fisheye.lens_for(width, height)
    _, _, cx, cy, _ = lens
    right, down, out = -direction_flu[:, 1], -direction_flu[:, 2], direction_flu[:, 0]
    norm = np.sqrt(right ** 2 + down ** 2 + out ** 2)
    theta = np.arccos(np.clip(out / norm, -1, 1))
    radius = np.array([fisheye.radius_of(t, lens) for t in theta])
    across = np.hypot(right, down)
    across[across < 1e-12] = 1e-12
    return cx + radius * right / across, cy + radius * down / across


def oak_pixels_in_fisheye(header: dict, nominal_m: float = 2.0, when=None):
    """Where each OAK depth pixel's centre lands in the 640x480 fisheye, for a
    surface `nominal_m` away: used only to carry labels drawn on the photograph
    into the depth grid, so no depth is read."""
    mount = oak.mount_at(when)
    x, y = pixel_grid(header)
    sensor = np.stack([x * nominal_m, y * nominal_m,
                       np.full_like(x, nominal_m)], axis=-1).reshape(-1, 3)
    flu = sensor @ _sensor_to_flu(mount).T + np.array(
        [mount.forward_m, mount.left_m, mount.up_m])
    fx, fy = fisheye_pixel(flu)
    return fx.reshape(x.shape), fy.reshape(x.shape)
