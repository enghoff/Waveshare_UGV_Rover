"""Geometry arithmetic from the prior repair bench, isolated from its labels and paths.

These sigmas are model diagnostics, not calibrated probabilities.
"""
import math
import numpy as np
from world_state import locate

def unit(blob):
    if not blob:
        return None
    v = np.frombuffer(blob, dtype="<f4").astype("float64")
    n = float(np.linalg.norm(v))
    return None if n < 1e-9 else v / n


class Look:
    __slots__ = ("id", "inference", "entity", "ray", "v", "a", "g", "row")

    def __init__(self, row, ray):
        self.id = row["id"]
        self.inference = row["inference_id"]
        self.entity = row["entity_id"]
        self.ray = ray
        self.v = unit(row.get("dino_blob"))
        self.a = unit(row.get("dino_alone_blob"))
        self.g = unit(row.get("siglip_blob"))
        self.row = row


def fit(members, start=None):
    """A placement from these looks' rays: refit from `start` when given, else
    founded the resolver's way. None when the rays cannot place anything."""
    rays = [m.ray for m in members if m.ray is not None]
    if not rays:
        return None
    point = None
    if start is not None:
        got = locate.fit_over(rays, [1.0] * len(rays),
                              (float(start["x_m"]), float(start["y_m"])),
                              float(start.get("extent_m") or 0.0))
        if got is not None:
            point = {**start, "x_m": got["x_m"], "y_m": got["y_m"],
                     "error_major_m": max(got["error_major_m"], 0.02),
                     "error_minor_m": max(got["error_minor_m"], 0.02),
                     "error_major_deg": got["error_major_deg"],
                     "uncertainty_m": max(got["error_major_m"], 0.02)}
    if point is None:
        best = locate.best_fix(rays)
        if best is not None:
            point = locate.refine(best, rays)
        else:
            ranged = [r for r in rays if r.get("range_m")]
            point = locate.at_range(ranged[0]) if ranged else None
    if point is None:
        return None
    point = dict(point)
    if "extent_m" not in point or point.get("extent_m") is None:
        point["extent_m"] = locate.extent_of((point["x_m"], point["y_m"]), *rays)
    point.update(locate.height_fields(point, rays))
    return point


def geometry(point, ray):
    """zb, zr, zh (None where unmeasured) of one ray against one placement."""
    out = {"zb": None, "zr": None, "zh": None, "range_m": None}
    if point is None or ray is None:
        return out
    px, py = float(point["x_m"]), float(point["y_m"])
    rng = math.hypot(px - ray["x_m"], py - ray["y_m"])
    out["range_m"] = rng
    r = max(rng, locate.MIN_RANGE_M)
    off = abs(locate._wrap(math.degrees(math.atan2(py - ray["y_m"], px - ray["x_m"]))
                           - ray["bearing_deg"]))
    half = locate.silhouette_deg(px, py, ray, float(point.get("extent_m") or 0.0))
    stated = (locate.STATED_MOVING_BEARING_DEG if locate.moving_look(ray)
              else locate.STATED_STILL_BEARING_DEG)
    stated = max(stated, locate.sigma_of(ray))
    across = math.degrees(math.atan2(locate.cross_track(point, ray)
                                     + float(ray.get("origin_sigma_m") or 0.0), r))
    out["zb"] = max(0.0, off - half) / math.hypot(stated, across)
    if off >= 90.0:
        out["zb"] = 99.0
    if ray.get("range_m") is not None:
        measured = float(ray["range_m"])
        sigma = math.hypot(max(float(ray.get("range_sigma_m") or 0.0), 0.1 * measured),
                           locate.along_track(point, ray))
        ext = locate.range_extent_m(point, ray)
        out["zr"] = max(0.0, abs(rng - measured) - ext) / sigma
    if point.get("height_m") is not None:
        rise = locate.rise_m(point, ray)
        if rise is not None:
            noise = max(locate.rise_noise_m(point, ray), 0.03)
            ext = locate.rise_extent_m(point, ray)
            out["zh"] = max(0.0, abs(rise - float(point["height_m"])) - ext) / noise
    return out
