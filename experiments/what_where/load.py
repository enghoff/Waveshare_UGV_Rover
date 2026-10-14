"""The world store's regions as arrays: what each looked like and where it was.

Read-only. One row per region the region finder kept, with its DINOv2 vectors
(the crop, and the crop with everything but the region blanked), where the
camera stood and pointed, the region's map bearing and, where the depth camera
measured it, its range and the map point that puts it at.
"""
import json
import math
import sqlite3

import numpy as np

#: Range limits the store's own placement uses (world_state/locate.py).
MIN_RANGE_M = 0.2
MAX_RANGE_M = 8.0


def room_of(x, y):
    """Which room a point is in, by the map of 2026-10-11 (score_session.py)."""
    if x < -15.4:
        return "living"
    if x < -9.8 and y < -12.0:
        return "corridor"
    return "bedroom"


def _unit(v):
    v = np.frombuffer(v, dtype=np.float32).astype(np.float64)
    n = np.linalg.norm(v)
    return v / n if n > 0 else v


class Regions:
    """Every region in a time window, as parallel arrays."""

    def __init__(self, rows):
        self.n = len(rows)
        self.id = np.array([r["id"] for r in rows], dtype=np.int64)
        self.t = np.array([r["t"] for r in rows])
        self.frame = np.array([r["frame"] for r in rows], dtype=object)
        self.posed = np.array([r["pose"] is not None for r in rows])
        nan = float("nan")
        self.cx = np.array([r["pose"]["x_m"] if r["pose"] else nan for r in rows])
        self.cy = np.array([r["pose"]["y_m"] if r["pose"] else nan for r in rows])
        self.yaw = np.array([(r["pose"]["heading_deg"] - r["pan"]) if r["pose"] else nan
                             for r in rows])
        self.bearing = np.array([nan if r["bearing"] is None else r["bearing"] for r in rows])
        self.range = np.array([nan if r["range"] is None else r["range"] for r in rows])
        self.bbox = np.array([r["bbox"] for r in rows]).reshape(-1, 4)
        self.vec = np.stack([r["alone"] for r in rows]) if rows else np.zeros((0, 384))
        ranged = (self.posed & np.isfinite(self.bearing) & np.isfinite(self.range)
                  & (self.range >= MIN_RANGE_M) & (self.range <= MAX_RANGE_M))
        self.ranged = ranged
        b = np.radians(np.nan_to_num(self.bearing))
        self.px = np.where(ranged, self.cx + np.nan_to_num(self.range) * np.cos(b), nan)
        self.py = np.where(ranged, self.cy + np.nan_to_num(self.range) * np.sin(b), nan)
        self.room = np.array([room_of(x, y) if math.isfinite(x) else None
                              for x, y in zip(self.cx, self.cy)], dtype=object)

    def subset(self, mask):
        out = object.__new__(Regions)
        for k, v in self.__dict__.items():
            out.__dict__[k] = v[mask] if isinstance(v, np.ndarray) else v
        out.n = int(np.count_nonzero(mask))
        return out

    def floor_sized(self):
        """Codex's floor for a crop worth labelling on the shakedown
        (shakedown_labels.json): the box reaches the lower part of the picture,
        covers 1.2% of it and is a tenth of it tall."""
        x0, y0, x1, y1 = self.bbox.T
        return (y1 >= 0.57) & ((x1 - x0) * (y1 - y0) >= 0.012) & ((y1 - y0) >= 0.1)


def load(db_path, start, end, map_session=67):
    """Regions from the gimbal camera observed in [start, end), oldest first."""
    con = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True)
    cur = con.execute(
        "select id, observed_at, frame_id, observer_pose_json, observer_pan_deg,"
        " bearing_deg, range_m, bbox_json, dino_alone_blob"
        " from observations where source='perception' and camera='gimbal'"
        " and map_session=? and observed_at>=? and observed_at<?"
        " and dino_alone_blob is not null order by observed_at, id",
        (map_session, start, end))
    rows = []
    for (oid, t, frame, pose, pan, bearing, rng, bbox, alone) in cur:
        pose = json.loads(pose) if pose else None
        if pose is not None and bearing is None:
            pose = None
        rows.append({"id": oid, "t": t, "frame": frame, "pose": pose,
                     "pan": float(pan or 0.0), "bearing": bearing, "range": rng,
                     "bbox": json.loads(bbox), "alone": _unit(alone)})
    con.close()
    return Regions(rows)
