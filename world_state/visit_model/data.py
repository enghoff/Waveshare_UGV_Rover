"""Every look since 2026-10-03 as arrays, with where each region stands.

    python data.py STORE.db OUT.npz [RECORDING.npz ...]

Read-only on a SQLite copy of the world store. One row per region the region
finder kept from the gimbal camera in map session 67, oldest first: when, which
picture, where the camera stood and pointed, the region's bearing, width and
range, its masked DINOv2 vector, and what the deployed store filed it under
(`entity`, the baseline).

**Where a region stands** is the store's range where it has one (the depth
camera under the outline, its box, or since 2026-10-14 the lidar), else the
lidar's return across its width from a drive recording that covers its shutter
(`lidar.place`) when its lower edge reaches the lidar's plane (the deployed
rule), else nothing: a bearing only. A look the store kept no pose
for takes navigation's pose from a recording, as the what/where streams did
(`recorded.py`, `lidar.py` and `poses.py` are theirs, from branch what-where).
"""
import json
import math
import os
import sqlite3
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, ROOT)

import lidar  # noqa: E402
import recorded  # noqa: E402

MAP_SESSION = 67
#: Ranges counted as measured (the what/where streams' limits).
MIN_RANGE_M = 0.2
MAX_RANGE_M = 8.0
WIDTH = 384


def _unit(blob):
    v = np.frombuffer(blob, dtype=np.float32).astype(np.float32)
    n = float(np.linalg.norm(v))
    return v / n if n > 0 else v


def _view():
    from world_state import view
    return view


def _reaches_down(bbox, tilt):
    """The deployed rule for a lidar range (world_state/lidar_ranges.py): the
    region's lower edge comes down to the lidar's plane. Without it a painting
    takes the range of whatever stands under it -- on the shakedown, the
    cabinet under the snowy painting."""
    from world_state import lidar_ranges, locate
    x0, _y0, x1, y1 = bbox
    lower = _view().elevation_deg((x0 + x1) / 2.0, y1, float(tilt or 0.0), (640, 480), None)
    if lower is None:
        return False
    return lidar_ranges.reaches_down(lower - locate.ELEVATION_BIAS_DEG.get(round(float(tilt or 0.0)), 0.0))


def build(db_path, recordings=(), after_id=None):
    """The arrays for every region, or, with `after_id`, for the regions the
    store has added since (the job on the rover reads them a batch at a time;
    each region is built on its own, so a batch matches the whole). A
    recording may be a path or a recorded.Recording."""
    recs = [p if isinstance(p, recorded.Recording) else recorded.Recording(p) for p in recordings]
    con = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True)
    rows = con.execute(
        "select o.id, o.observed_at, f.taken_at, o.frame_id, o.observer_pose_json,"
        " o.observer_pan_deg, o.observer_tilt_deg, o.bearing_deg, o.span_deg,"
        " o.elevation_deg, o.range_m, o.range_from, o.bbox_json, o.dino_alone_blob,"
        " o.entity_id, o.note, o.mask_share"
        " from observations o left join frames f on f.id = o.frame_id"
        " where o.source = 'perception' and o.camera = 'gimbal' and o.map_session = ?"
        " and o.dino_alone_blob is not null and o.id > ? order by o.observed_at, o.id",
        (MAP_SESSION, int(after_id or 0))).fetchall()
    con.close()
    n = len(rows)
    nan = float("nan")
    out = {k: np.full(n, nan) for k in ("t", "shutter", "cx", "cy", "heading", "pan", "tilt",
                                         "bearing", "span", "elevation", "range", "px", "py",
                                         "mask_share")}
    out["id"] = np.zeros(n, np.int64)
    out["frame"] = np.empty(n, object)
    out["entity"] = np.empty(n, object)
    out["matched"] = np.zeros(n, bool)
    out["pose_from"] = np.empty(n, object)
    out["place_from"] = np.empty(n, object)
    out["bbox"] = np.zeros((n, 4))
    out["vec"] = np.zeros((n, WIDTH), np.float32)
    view = None
    for i, (oid, t, taken, frame, pose, pan, tilt, bearing, span, elev, rng, rfrom, bbox,
            alone, entity, note, share) in enumerate(rows):
        out["id"][i] = oid
        out["t"][i] = t
        out["shutter"][i] = taken or t
        out["frame"][i] = frame
        out["entity"][i] = entity
        # The resolver's note on an ordinary match (identity_count.MATCH_NOTE).
        out["matched"][i] = bool(note and note.startswith("the bearing points at"))
        out["pan"][i] = float(pan or 0.0)
        out["tilt"][i] = float(tilt or 0.0)
        out["bbox"][i] = json.loads(bbox)
        out["vec"][i] = _unit(alone)
        out["mask_share"][i] = nan if share is None else share
        out["elevation"][i] = nan if elev is None else elev
        pose = json.loads(pose) if pose else None
        source = "stored" if (pose is not None and bearing is not None) else None
        if source is None and recs:
            rec = recorded.for_time(recs, taken or t)
            if rec is not None:
                rate = rec.turn_rate(taken or t)
                p = rec.pose(taken or t) if rate is not None and rate <= recorded.MAX_TURN_DPS else None
                if p is not None:
                    view = view or _view()
                    pose = {"x_m": p[0], "y_m": p[1], "heading_deg": p[2]}
                    ray = view.ray({"pose": pose, "observer_pan_deg": pan, "observer_tilt_deg": tilt,
                                    "bbox": json.loads(bbox)}, 130.0, size=(640, 480))
                    if ray is not None:
                        bearing, span, source = ray["bearing_deg"], ray["span_deg"], "recorded"
                        rng = None  # the store's range went with a pose it did not keep
        if source is None:
            continue
        out["pose_from"][i] = source
        out["cx"][i], out["cy"][i] = pose["x_m"], pose["y_m"]
        out["heading"][i] = pose["heading_deg"]
        out["bearing"][i] = bearing
        out["span"][i] = nan if span is None else span
        b = math.radians(bearing)
        if rng is not None and MIN_RANGE_M <= rng <= MAX_RANGE_M:
            out["range"][i] = rng
            out["px"][i], out["py"][i] = pose["x_m"] + rng * math.cos(b), pose["y_m"] + rng * math.sin(b)
            out["place_from"][i] = rfrom or "depth"
        elif recs and _reaches_down(json.loads(bbox), tilt):
            p = lidar.place(recs, taken or t, bearing, out["span"][i])
            if p is not None:
                out["px"][i], out["py"][i] = p
                out["range"][i] = math.hypot(p[0] - pose["x_m"], p[1] - pose["y_m"])
                out["place_from"][i] = "recording lidar"
    return out


def save(arrays, path):
    np.savez_compressed(path, **{k: (v.astype(str) if v.dtype == object else v)
                                 for k, v in arrays.items()})


def load(path):
    d = np.load(path, allow_pickle=False)
    out = {k: d[k] for k in d.files}
    for k in ("frame", "entity", "pose_from", "place_from"):
        out[k] = np.where(out[k] == "None", None, out[k]).astype(object)
    return out


if __name__ == "__main__":
    a = build(sys.argv[1], sys.argv[3:])
    save(a, sys.argv[2])
    posed = np.array([p is not None for p in a["pose_from"]])
    placed = np.isfinite(a["px"])
    print("%d regions, %d posed (%d from recordings), %d placed (%s)" % (
        len(a["id"]), posed.sum(), sum(p == "recorded" for p in a["pose_from"]), placed.sum(),
        {k: int(sum(1 for p in a["place_from"] if p == k)) for k in set(a["place_from"]) if k}))
