"""The world store's regions as arrays: what each looked like and where it was.

Read-only. One row per region the region finder kept, with its DINOv2 vectors
(the crop, and the crop with everything but the region blanked), where the
camera stood and pointed, the region's map bearing and, where the depth camera
measured it, its range and the map point that puts it at.
"""
import json
import math
import os
import sqlite3
import sys

import numpy as np

#: Ranges counted as measured. Not the store's own limits (world_state/locate.py
#: uses 0.75-12 m); the result is the same with those (results.md).
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
        self.source = np.array([r.get("source", "stored" if r["pose"] else None)
                                for r in rows], dtype=object)
        self.shutter = np.array([r.get("shutter", r["t"]) for r in rows])
        self.span = np.array([nan if r.get("span") is None else r["span"] for r in rows])

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


def load(db_path, start, end, map_session=67, recordings=None):
    """Regions from the gimbal camera observed in [start, end), oldest first.

    With recordings (recorded.Recording), a look the store kept no pose for
    takes navigation's pose at its shutter from the recording that covers it,
    unless the rover was turning faster than recorded.MAX_TURN_DPS, and its
    regions' bearings are worked out through the store's own lens arithmetic
    (world_state/view.py). Each row says where its pose came from."""
    con = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True)
    cur = con.execute(
        "select o.id, o.observed_at, o.frame_id, o.observer_pose_json, o.observer_pan_deg,"
        " o.bearing_deg, o.range_m, o.bbox_json, o.dino_alone_blob, o.observer_tilt_deg,"
        " f.taken_at, o.span_deg"
        " from observations o left join frames f on f.id = o.frame_id"
        " where o.source='perception' and o.camera='gimbal'"
        " and o.map_session=? and o.observed_at>=? and o.observed_at<?"
        " and o.dino_alone_blob is not null order by o.observed_at, o.id",
        (map_session, start, end))
    rows = []
    for (oid, t, frame, pose, pan, bearing, rng, bbox, alone, tilt, taken, span) in cur:
        pose = json.loads(pose) if pose else None
        if pose is not None and bearing is None:
            pose = None
        source = None if pose is None else "stored"
        if pose is None and recordings:
            pose, bearing, span = _recorded(recordings, taken or t, pan, tilt, json.loads(bbox))
            source = None if pose is None else "recorded"
        rows.append({"id": oid, "t": t, "frame": frame, "pose": pose,
                     "pan": float(pan or 0.0), "bearing": bearing, "range": rng,
                     "bbox": json.loads(bbox), "alone": _unit(alone),
                     "source": source, "shutter": taken or t, "span": span})
    con.close()
    return Regions(rows)


def _recorded(recordings, t, pan, tilt, bbox):
    """Navigation's pose at the shutter, and the bearing and angular width it
    gives, or (None, None, None)."""
    import recorded
    rec = recorded.for_time(recordings, t)
    if rec is None:
        return None, None, None
    rate = rec.turn_rate(t)
    if rate is None or rate > recorded.MAX_TURN_DPS:
        return None, None, None
    p = rec.pose(t)
    if p is None:
        return None, None, None
    pose = {"x_m": p[0], "y_m": p[1], "heading_deg": p[2]}
    ray = _view().ray({"pose": pose, "observer_pan_deg": pan, "observer_tilt_deg": tilt,
                       "bbox": bbox}, 130.0, size=(640, 480))
    if ray is None:
        return None, None, None
    return pose, ray["bearing_deg"], ray["span_deg"]


def _view():
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    if root not in sys.path:
        sys.path.insert(0, root)
    from world_state import view
    return view
