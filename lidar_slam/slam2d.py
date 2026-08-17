#!/usr/bin/env python3
"""ctypes binding for libslam2d.so.

The heavy lifting is in C because the Pi cannot afford it in Python -- see
slam2d.h -- but everything around it stays here: opening ports, unit conversion,
and deciding what to do with a pose. This module is only the boundary.

Build the library first:  ./build.sh
"""
import ctypes
import math
import os
import threading
from collections import namedtuple
from ctypes import POINTER, byref, c_char_p, c_float, c_int, c_void_p

_HERE = os.path.dirname(os.path.abspath(__file__))
_SONAME = "libslam2d.so"

WALL, OBJECT, GAP = 0, 1, 2
KIND_NAMES = {WALL: "wall", OBJECT: "object", GAP: "gap"}


class Config(ctypes.Structure):
    """Mirrors slam2d_config. Field order must match slam2d.h exactly; the
    constructor below checks the total size against the library to catch it when it
    does not, because a mismatch is otherwise completely silent."""

    _fields_ = [
        ("grid_cells", c_int),
        ("resolution_m", c_float),
        ("mount_deg", c_float),
        ("min_range_m", c_float),
        ("max_range_m", c_float),
        ("body_back_m", c_float),
        ("body_half_width_m", c_float),
        ("max_points", c_int),
        ("coarse_lin_m", c_float),
        ("coarse_ang_deg", c_float),
        ("coarse_lin_steps", c_int),
        ("coarse_ang_steps", c_int),
        ("fine_lin_m", c_float),
        ("fine_ang_deg", c_float),
        ("fine_lin_steps", c_int),
        ("fine_ang_steps", c_int),
        ("min_match_score", c_float),
        ("hit_inc", c_int),
        ("miss_dec", c_int),
        ("occupied_at", c_int),
        ("lik_stamp", c_int),
        ("lik_decay", c_int),
        ("rover_width_m", c_float),
    ]

    def __repr__(self):
        body = ", ".join(f"{n}={getattr(self, n)!r}" for n, _ in self._fields_)
        return f"Config({body})"


class _Feature(ctypes.Structure):
    """Mirrors slam2d_feature. Checked against the library like Config is."""

    _fields_ = [
        ("kind", c_int),
        ("bearing_deg", c_float),
        ("range_m", c_float),
        ("width_m", c_float),
        ("span_deg", c_float),
        ("straightness_m", c_float),
        ("x0", c_float), ("y0", c_float),
        ("x1", c_float), ("y1", c_float),
    ]


#: One thing the rover can see. `kind` is "wall", "object" or "gap"; bearings are
#: degrees counter-clockwise from straight ahead.
Feature = namedtuple("Feature", "kind bearing_deg range_m width_m span_deg "
                                "straightness_m x0 y0 x1 y1")

MAX_FEATURES = 64


def _load(path=None):
    path = path or os.path.join(_HERE, _SONAME)
    if not os.path.exists(path):
        raise RuntimeError(
            f"{path} is missing. It is built per-machine, not committed -- "
            f"run {os.path.join(_HERE, 'build.sh')} on the host that will run it."
        )
    lib = ctypes.CDLL(path)

    lib.slam2d_config_size.restype = c_int
    lib.slam2d_feature_size.restype = c_int
    lib.slam2d_default_config.argtypes = [POINTER(Config)]
    lib.slam2d_create.argtypes = [POINTER(Config)]
    lib.slam2d_create.restype = c_void_p
    lib.slam2d_destroy.argtypes = [c_void_p]
    lib.slam2d_feed_lidar.argtypes = [c_void_p, c_char_p, c_int]
    lib.slam2d_feed_lidar.restype = c_int
    lib.slam2d_set_prior.argtypes = [c_void_p, c_float, c_float]
    lib.slam2d_update.argtypes = [c_void_p]
    lib.slam2d_update.restype = c_int
    lib.slam2d_pose.argtypes = [c_void_p, POINTER(c_float), POINTER(c_float),
                                POINTER(c_float)]
    lib.slam2d_set_pose.argtypes = [c_void_p, c_float, c_float, c_float]
    lib.slam2d_score.argtypes = [c_void_p]
    lib.slam2d_score.restype = c_float
    for name in ("slam2d_rejected", "slam2d_scans", "slam2d_points"):
        getattr(lib, name).argtypes = [c_void_p]
        getattr(lib, name).restype = c_int
    lib.slam2d_sectors.argtypes = [c_void_p, POINTER(c_float), c_int]
    lib.slam2d_arc_clearance.argtypes = [c_void_p, c_float, c_float, c_float]
    lib.slam2d_arc_clearance.restype = c_float
    lib.slam2d_features.argtypes = [c_void_p, POINTER(_Feature), c_int]
    lib.slam2d_features.restype = c_int
    lib.slam2d_scan_xy.argtypes = [c_void_p, POINTER(c_float), c_int]
    lib.slam2d_scan_xy.restype = c_int
    lib.slam2d_grid.argtypes = [c_void_p]
    lib.slam2d_grid.restype = POINTER(ctypes.c_byte)
    lib.slam2d_grid_origin.argtypes = [c_void_p, POINTER(c_float), POINTER(c_float)]

    for name, struct, size_fn in (("Config", Config, lib.slam2d_config_size),
                                  ("Feature", _Feature, lib.slam2d_feature_size)):
        want, got = size_fn(), ctypes.sizeof(struct)
        if want != got:
            raise RuntimeError(
                f"{name} layout disagrees with {path}: the library says {want} "
                f"bytes, this module builds {got}. The _fields_ list above has "
                f"drifted from slam2d.h -- fix it rather than padding it."
            )
    return lib


def default_config(lib=None):
    lib = lib or _load()
    cfg = Config()
    lib.slam2d_default_config(byref(cfg))
    return cfg


class Slam2D:
    """One map and one pose estimate, fed raw lidar bytes.

    The lag is worth knowing about: a revolution is only complete once the *next*
    one's first packet arrives, because that is how the wrap is detected. So feed()
    reports a revolution roughly 1/419th of a turn after it truly ended, and the
    pose you read out belongs to the revolution before the bytes you just pushed in.
    """

    def __init__(self, config=None, library=None):
        self._lib = _load(library)
        self.config = config if config is not None else default_config(self._lib)
        self._h = self._lib.slam2d_create(byref(self.config))
        if not self._h:
            raise RuntimeError(f"slam2d_create rejected {self.config}")
        self._xy = (c_float * (2 * max(1, self.config.max_points)))()
        self._feat = (_Feature * MAX_FEATURES)()
        #: Nothing in the C is internally locked, and the daemon reads this from
        #: connection threads while a control loop writes it. Every method here takes
        #: this; hold it yourself across any read-modify-write of your own.
        self.lock = threading.RLock()

    def close(self):
        if getattr(self, "_h", None):
            self._lib.slam2d_destroy(self._h)
            self._h = None

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    __del__ = close

    # --- input ----------------------------------------------------------------
    def feed(self, data):
        """Push raw lidar bytes. Returns how many revolutions completed."""
        with self.lock:
            return self._lib.slam2d_feed_lidar(self._h, bytes(data), len(data))

    def set_prior(self, forward_m, yaw_rad):
        with self.lock:
            self._lib.slam2d_set_prior(self._h, forward_m, yaw_rad)

    def update(self):
        """Match and map the pending revolution. True if there was one."""
        with self.lock:
            return bool(self._lib.slam2d_update(self._h))

    # --- output ---------------------------------------------------------------
    @property
    def pose(self):
        with self.lock:
            x, y, th = c_float(), c_float(), c_float()
            self._lib.slam2d_pose(self._h, byref(x), byref(y), byref(th))
            return x.value, y.value, th.value

    @pose.setter
    def pose(self, xyth):
        with self.lock:
            self._lib.slam2d_set_pose(self._h, *(float(v) for v in xyth))

    @property
    def score(self):
        with self.lock:
            return self._lib.slam2d_score(self._h)

    @property
    def rejected(self):
        with self.lock:
            return bool(self._lib.slam2d_rejected(self._h))

    @property
    def scans(self):
        with self.lock:
            return self._lib.slam2d_scans(self._h)

    @property
    def points(self):
        with self.lock:
            return self._lib.slam2d_points(self._h)

    def sectors(self, n=36):
        """Nearest return per sector, metres, or **None** where nothing came back.

        Index 0 straddles straight ahead and they run counter-clockwise, so n=36
        gives 10-degree sectors. None is not the same as "clear": roughly a sixth of
        this sensor's beams return nothing, and a beam returns nothing off matt black
        fabric and off glass. Treat None as unknown and slow down, never as free.
        """
        with self.lock:
            out = (c_float * n)()
            self._lib.slam2d_sectors(self._h, out, n)
            return [None if v < 0.0 else v for v in out]

    def arc_clearance(self, curvature, half_width=None, max_dist=6.0):
        """Metres along the arc of the given curvature (1/m, + is left) before its
        corridor is blocked, capped at max_dist. Reads the live scan, not the map."""
        if half_width is None:
            half_width = self.config.rover_width_m * 0.5
        with self.lock:
            return self._lib.slam2d_arc_clearance(self._h, curvature, half_width,
                                                  max_dist)

    def features(self):
        """The last revolution as walls, objects and gaps -- see slam2d.h."""
        with self.lock:
            n = self._lib.slam2d_features(self._h, self._feat, MAX_FEATURES)
            return [Feature(KIND_NAMES.get(f.kind, "?"), f.bearing_deg, f.range_m,
                            f.width_m, f.span_deg, f.straightness_m,
                            f.x0, f.y0, f.x1, f.y1)
                    for f in self._feat[:n]]

    def scan_xy(self):
        """The last revolution in the rover frame, as a list of (x, y) metres."""
        with self.lock:
            n = self._lib.slam2d_scan_xy(self._h, self._xy, self.config.max_points)
            return [(self._xy[2 * i], self._xy[2 * i + 1]) for i in range(n)]

    # --- putting it into words ------------------------------------------------

    @staticmethod
    def direction(bearing_deg):
        """A bearing as something you could say out loud."""
        b = (bearing_deg + 180.0) % 360.0 - 180.0
        side = "left" if b > 0 else "right"
        a = abs(b)
        if a < 15:
            return "straight ahead"
        if a > 165:
            return "directly behind you"
        if a < 50:
            return f"ahead and to your {side}"
        if a < 115:
            return f"to your {side}"
        return f"behind you on the {side}"

    def describe(self, n_sectors=36):
        """What is around the rover, as structured data and as a sentence or two.

        Built for a language model rather than a plotter: a list of 36 ranges is
        something a model will happily hallucinate over, whereas walls, objects and
        gaps are things it can name. The one inference made here is grouping nearby
        small objects, because four of them in a square metre is the signature of
        furniture and no single reading says so.
        """
        with self.lock:
            feats = self.features()
            sectors = self.sectors(n_sectors)
            x, y, th = self.pose
            score, rejected, scans = self.score, self.rejected, self.scans

        walls = [f for f in feats if f.kind == "wall"]
        objects = [f for f in feats if f.kind == "object"]
        gaps = sorted((f for f in feats if f.kind == "gap"),
                      key=lambda f: -f.width_m)

        ahead = sectors[0]
        parts = []
        if ahead is None:
            parts.append("Nothing is coming back from straight ahead, which means "
                         "either open space beyond range or a surface this sensor "
                         "cannot see, so treat it as unknown rather than clear")
        else:
            parts.append(f"Straight ahead is clear for {ahead:.1f} m")

        for w in sorted(walls, key=lambda f: f.range_m)[:4]:
            parts.append(f"a flat surface {w.range_m:.1f} m away "
                         f"{self.direction(w.bearing_deg)}, running about "
                         f"{w.width_m:.1f} m")

        # Group the small stuff: individually a leg is meaningless, together they are
        # a piece of furniture, and the grouping is the part a model cannot do from a
        # list of bearings.
        clusters = []
        for o in objects:
            ox, oy = o.range_m * math.cos(math.radians(o.bearing_deg)), \
                     o.range_m * math.sin(math.radians(o.bearing_deg))
            for c in clusters:
                if math.hypot(ox - c["x"], oy - c["y"]) < 1.5:
                    c["n"] += 1
                    c["x"] = (c["x"] * (c["n"] - 1) + ox) / c["n"]
                    c["y"] = (c["y"] * (c["n"] - 1) + oy) / c["n"]
                    c["near"] = min(c["near"], o.range_m)
                    break
            else:
                clusters.append({"n": 1, "x": ox, "y": oy, "near": o.range_m})

        for c in clusters:
            where = self.direction(math.degrees(math.atan2(c["y"], c["x"])))
            if c["n"] >= 3:
                parts.append(f"{c['n']} narrow objects standing together about "
                             f"{c['near']:.1f} m away {where} -- this sensor sees one "
                             f"slice at its own height, so a table or a chair appears "
                             f"as its legs and nothing else")
            else:
                parts.append(f"{c['n']} small object{'s' if c['n'] > 1 else ''} "
                             f"{c['near']:.1f} m away {where}")

        if gaps:
            g = gaps[0]
            parts.append(f"the widest way through is {g.width_m:.1f} m across, "
                         f"{self.direction(g.bearing_deg)} at "
                         f"{abs(g.bearing_deg):.0f} degrees")

        unknown = sum(1 for v in sectors if v is None)
        if unknown > n_sectors // 3:
            parts.append(f"{unknown} of {n_sectors} directions returned nothing at "
                         f"all, so much of this is unknown rather than empty")

        text = ". ".join(parts) + "."
        if rejected:
            text += (" The position estimate is not being trusted at the moment, so "
                     "distances are reliable but the rover's own coordinates are not.")

        return {
            "text": text,
            "clear_ahead_m": ahead,
            "pose": {"x_m": round(x, 3), "y_m": round(y, 3),
                     "heading_deg": round(math.degrees(th), 1)},
            "match_score": round(score, 3),
            "position_trusted": not rejected,
            "scans": scans,
            "unknown_directions": unknown,
            "walls": [{"bearing_deg": round(w.bearing_deg, 1),
                       "range_m": round(w.range_m, 2),
                       "length_m": round(w.width_m, 2)} for w in walls],
            "objects": [{"bearing_deg": round(o.bearing_deg, 1),
                         "range_m": round(o.range_m, 2),
                         "width_m": round(o.width_m, 2)} for o in objects],
            "gaps": [{"bearing_deg": round(g.bearing_deg, 1),
                      "range_m": round(g.range_m, 2),
                      "width_m": round(g.width_m, 2)} for g in gaps],
        }

    def grid(self):
        """Occupancy log-odds as a numpy int8 view, shape (cells, cells), indexed
        [ix, iy] with ix along start-forward and iy along start-left. This is a view
        into the library's own memory, not a copy -- it changes under you on the next
        update, and it dies with close()."""
        import numpy as np

        n = self.config.grid_cells
        ptr = self._lib.slam2d_grid(self._h)
        return np.ctypeslib.as_array(ptr, shape=(n * n,)).reshape(n, n)

    @property
    def grid_origin(self):
        ox, oy = c_float(), c_float()
        self._lib.slam2d_grid_origin(self._h, byref(ox), byref(oy))
        return ox.value, oy.value

    def write_pgm(self, path):
        """Dump the map as a binary PGM, which needs no image library and which
        anything at all can open. Occupied is dark, free is light, and never-seen is
        the mid grey in between -- so an unexplored map is uniformly grey rather
        than confidently empty."""
        g = self.grid()
        occ, n = self.config.occupied_at, self.config.grid_cells
        import numpy as np

        img = np.full((n, n), 128, dtype=np.uint8)
        img[g >= occ] = 0
        img[(g < 0)] = 245
        img[(g >= 0) & (g < occ) & (g != 0)] = 190
        # Rows top-to-bottom with +x up and +y left, so the picture is the plan view
        # lidar/lidar_view.py draws rather than its transpose.
        out = np.flipud(np.fliplr(img.T))
        with open(path, "wb") as f:
            f.write(b"P5\n# slam2d occupancy, %d m across at %.3f m/cell\n%d %d\n255\n"
                    % (int(n * self.config.resolution_m), self.config.resolution_m,
                       n, n))
            f.write(out.tobytes())
        return path


if __name__ == "__main__":
    lib = _load()
    print(f"{_SONAME} loaded, slam2d_config is {lib.slam2d_config_size()} bytes")
    print(default_config(lib))
