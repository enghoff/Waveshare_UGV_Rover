"""The fixtures the world-state checks share, and the paths they need.

A store in a temporary directory, a camera that returns one real one-pixel JPEG,
a pose, a sighting shaped the way the sidecar returns them, a bearing pointed at
a known answer, and an inspector that sees whatever a check tells it to. Nothing here touches a rover, a GPU or an
encoder.

Importing this module also puts the package's parent on `sys.path` -- `~/ugv` on
the rover, the checkout root here -- which is what lets `world_state` be imported
as a package from both.
"""
from __future__ import annotations

import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
# The package is imported as a package, so what goes on the path is its parent --
# ~/ugv on the rover, the checkout root here.
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

from world_state.inspector import Inspector        # noqa: E402
from world_state.store import WorldStore           # noqa: E402


#: A one-pixel JPEG. Nothing decodes it here -- the store keeps bytes and the fake
#: sidecar counts them -- but it should be a real picture, because the thing the
#: rover stores is a real picture.
JPEG = bytes.fromhex(
    "ffd8ffe000104a46494600010100000100010000ffdb004300ffffffffffffffffffffffff"
    "ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff"
    "ffffffffffffffffffffffffffffffffffffffc00011080001000101011100ffc400140001"
    "00000000000000000000000000000009ffc4001401010000000000000000000000000000"
    "0000ffda000c03010002110311003f00b7ffd9")


def a_store(directory):
    return WorldStore(directory)


def a_capture(pan=0.0, tilt=0.0, ok=True, error="", live=False,
              taken_at=None, delay_s=0.0):
    """A camera. `taken_at` is what the real one now says about when the shutter
    opened, and `delay_s` is how long the grab takes, which is what makes the
    two pose readings either side of it different."""
    def capture():
        if delay_s:
            time.sleep(delay_s)
        if not ok:
            return {"ok": False, "error": error}
        frame = {"ok": True, "jpeg": JPEG, "pan": pan, "tilt": tilt,
                 "live": live, "width": 640, "height": 480}
        if taken_at is not None:
            frame["taken_at"] = taken_at() if callable(taken_at) else taken_at
        return frame
    return capture


def a_pose(x=1.0, y=2.0, heading=90.0):
    return lambda: {"x_m": x, "y_m": y, "heading_deg": heading}


def a_turning_pose(headings, x=1.0, y=2.0):
    """A rover whose heading is different each time it is asked.

    Which is the case that mattered and that no fixture could build: the pose is
    read on both sides of the grab, and a rover turning between the two readings
    is what cost the drive of 2026-09-03 two thirds of its bearings.
    """
    seen = []

    def pose():
        heading = headings[min(len(seen), len(headings) - 1)]
        seen.append(heading)
        return {"x_m": x, "y_m": y, "heading_deg": heading}
    return pose


# --- an inspection through the encoders --------------------------------------
#
# The path the rover actually uses now. What is worth proving offline is that
# what was *measured* survives into the database unchanged and that what was not
# measured stays empty: a bearing invented from a missing pose would be the one
# failure this whole design exists to avoid.


def a_sighting(bbox=None, dino=None, siglip=None):
    """One measured region, and there is nothing on it saying what it is.

    That is the whole shape of what perception returns now: a box, a region
    score and two vectors. Anything downstream that wants to tell two of
    these apart has to do it from the vectors or from where they point.
    """
    from world_state.perception_client import Sighting

    return Sighting(bbox=bbox or [0.1, 0.3, 0.5, 0.9],
                    region_score=0.83, area=0.24,
                    dino=dino if dino is not None else a_vector(1.0, 0.0),
                    siglip=siglip if siglip is not None
                    else a_vector(0.5, 0.5))


def a_seeing_inspector(directory, looks=None, fail="", capture=None, pose=None,
                       fov_deg=100.0):
    from world_state.perception_client import FakeEyes

    store = a_store(directory)
    eyes = FakeEyes(looks or [], fail=fail)
    return store, eyes, Inspector(
        store, eyes, capture or a_capture(pan=20.0, tilt=-5.0),
        pose or a_pose(), fov_deg=fov_deg)


# --- the resolver ------------------------------------------------------------
#
# The part the proof-of-concept failed at. Every test here is a case the rover
# actually has to survive rather than a check that the code runs: two identical
# chairs, a rover that only turned on the spot, and an appearance score high
# enough to be tempting and pointing at the wrong side of the room.


def a_vector(*values, width=8):
    """A float32 vector, padded, so appearance can be steered in a test."""
    import struct

    numbers = list(values) + [0.0] * (width - len(values))
    return struct.pack(f"<{width}f", *numbers)


#: A box sitting on the lens axis, so that the angle it contributes is zero.
#:
#: **Not the middle of the picture, which is a different point.** The sweep put
#: this camera's principal point thirteen pixels above the centre of the frame,
#: and a bearing is now worked out through that lens rather than by multiplying
#: a fraction of the frame by a field of view -- so a box centred in the picture
#: contributes 0.8 degrees and every expectation in every resolver test would be
#: carrying it. Written as the axis rather than as the numbers it comes to, so
#: that a re-swept lens moves the fixture instead of breaking forty tests.
ON_AXIS = (315.9 / 640.0, 227.4 / 480.0)


def a_ray(x_m, y_m, at, *, sigma=None, span=0.0, look=None, off=0.0,
          observation=None):
    """A bearing from `(x_m, y_m)` that points at `at`, give or take `off`."""
    bearing = math.degrees(math.atan2(at[1] - y_m, at[0] - x_m)) + off
    built = {"x_m": x_m, "y_m": y_m, "bearing_deg": bearing,
             "span_deg": span, "origin_sigma_m": 0.0,
             "inference_id": look, "observation_id": observation}
    if sigma is not None:
        built["bearing_sigma_deg"] = sigma
    return built


def a_box(width=0.10, height=0.60):
    """A box of this size in the frame, centred on the lens axis."""
    return [ON_AXIS[0] - width / 2.0, ON_AXIS[1] - height / 2.0,
            ON_AXIS[0] + width / 2.0, ON_AXIS[1] + height / 2.0]


def a_look(store, x, y, bearings, vectors=None, inference=1):
    """One look from one place, with a region on each of these bearings.

    **The thing `observe` cannot build, and the fault it is needed for.** Two
    regions of one picture share a pose and a gimbal angle, so their bearings
    differ only by where in the frame they sat -- which means `observe`, which
    takes a heading and puts the box on the lens axis, can only ever produce one
    bearing per look. Two adjacent objects seen in one picture is exactly the
    case that goes wrong, so the bearings are written here directly, the way
    `replay` writes a recording back.

    Nothing else is different: the rows are ordinary observations with a shared
    `inference_id`, no entity, and a pose the resolver will read.
    """
    import json

    vectors = vectors or [a_vector(1.0, 0.0)] * len(bearings)
    pose = json.dumps({"x_m": x, "y_m": y, "heading_deg": 0.0})
    with store._lock, store.db:
        for bearing, vector in zip(bearings, vectors):
            store.db.execute(
                "INSERT INTO observations(entity_id, inference_id, observed_at,"
                " source, frame_id, bbox_json, observer_pan_deg,"
                " observer_tilt_deg, observer_pose_json, map_session,"
                " bearing_deg, span_deg, region_source, dino_blob, siglip_blob,"
                " vectors_from)"
                " VALUES(NULL,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)",
                (inference, time.time(), "perception", f"f{inference}",
                 json.dumps(a_box()), 0.0, 0.0, pose, store.map_session(),
                 float(bearing), 12.6, "yoloe", vector,
                 a_vector(0.5, 0.5), "fake"))


def observe(store, x, y, bearing, vector=None, inference=None, fov_deg=100.0):
    """One look at something, from a place, along a bearing.

    The box sits on the lens axis, so the bearing really is the pose's heading
    minus the gimbal's pan and nothing else -- which keeps these tests about the
    resolver rather than about `view.ray`, which has its own.
    """
    from world_state.perception_client import Sighting

    seen = [Sighting(bbox=a_box(),
                     dino=vector if vector is not None else a_vector(1.0, 0.0),
                     siglip=a_vector(0.5, 0.5))]
    store.record(seen, capture={"frame_id": "f", "pan": 0.0,
                                "pose": {"x_m": x, "y_m": y,
                                         "heading_deg": bearing}},
                 fov_deg=fov_deg, region_source="yoloe", vectors_from="fake",
                 inference_id=inference)
