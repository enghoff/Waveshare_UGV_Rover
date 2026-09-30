#!/usr/bin/env python3
"""Work every look out again through the fisheye lens the rover flies now.

    ssh orin 'cd ~/ugv/world_state && python3 relens.py'           # what it would change
    ssh orin 'cd ~/ugv/world_state && python3 relens.py --apply'   # change it

**What this is for.** A look's direction is worked out once, when it is taken, and
stored -- deliberately, so that refitting the lens cannot silently rewrite what
the rover measured. When the lens turns out to have been wrong, that same rule
leaves every stored direction wrong with it, and a store in which old looks and
new ones disagree about where the same thing is. `face_tracking/lens.py` was
refitted on 2026-09-30 (it had been about 7% short off-axis), and this is the
deliberate, reviewable rewrite the rule was protecting against doing silently.

**Only what can be reproduced is rewritten.** Every look is first worked out
again through the lens it was recorded through, `PREVIOUS_LENS`; a look whose
stored direction, width and height angle that does not reproduce to the tenth
of a degree they are stored to was not recorded through that lens, or from
those inputs, and is left exactly as it is and counted. The same for a range:
it is replayed from the depth map kept beside the look, through the geometry
the rover had then, and only rewritten if that replay gives back the stored
number to within the uncertainty the reading carried -- the map kept is the
frame after the one ranged, and stereo moves by a few millimetres between
frames. On the drive of 2026-09-08 half the replays were exact to the
millimetre and nine in ten within 8 mm.

**What is rewritten.** For a look that reproduces: its bearing, span, elevation
and elevation span, through the lens as it is now. For a look whose depth map
was kept: its range and range uncertainty, from that depth map, through today's
fisheye, the OAK's corrected lens and the chassis mount it was on
(`oak.CHASSIS_MOUNT`, or whatever `oak.mount_at` says for when it was taken). The
part of a range's uncertainty that came from the rover moving is carried over
unchanged. Then every placed thing is placed again from its own looks by the
resolver's own rules -- the same pair-and-refine `resolve._replace_placement`
uses, or a single ranged look as `resolve._place_from_range` does -- which keeps
every identity and moves only positions. What the resolver decided about which
looks belong together is not revisited.

**Nothing is written without `--apply`**, and `--apply` copies the database
beside itself first, as `world.db.before-relens-<time>`.
"""
from __future__ import annotations

import argparse
import base64
import gzip
import json
import math
import os
import socket
import sqlite3
import statistics
import sys
import time
import types
import zlib
from typing import Any

if __package__ in (None, ""):                       # run as a script on the rover
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from world_state import locate, oak, resolve, view            # noqa: E402
from world_state.depth_client import (                        # noqa: E402
    Lens, NOTHING_TO_MEASURE, OUTSIDE_VIEW,
)
from world_state.store import WorldStore                      # noqa: E402

#: The fisheye every look before 2026-09-30 was drawn through, at 640x480:
#: arcmin per pixel, distortion term, centre. Swept 2026-08-19.
PREVIOUS_LENS = (11.82, 0.030, (315.9, 227.4))
#: And the OAK as the depth service described it until 2026-09-30 -- 9.6% short
#: in focal length, see `oak_depth/colour_lens.py`.
PREVIOUS_OAK_LENS = Lens(fx=456.54, fy=456.43, cx=321.12, cy=189.75,
                         width=640, height=360, hfov_deg=70.1, vfov_deg=43.0)
#: And the chassis bracket as `oak.MOUNT` held it then, before its forward offset
#: and its angles were corrected.
PREVIOUS_CHASSIS_MOUNT = oak.Mount(yaw_deg=1.492, pitch_deg=6.256, roll_deg=-1.200,
                                   forward_m=0.0872, left_m=-0.0031,
                                   up_m=-0.0937, on_gimbal=False)
#: The OAK's lens as the depth service publishes it now, if the service cannot
#: be asked. Its `/health` is the authority; this is what it said on 2026-09-30.
CURRENT_OAK_LENS = Lens(fx=500.30, fy=500.17, cx=321.23, cy=190.69,
                        width=640, height=360, hfov_deg=65.2, vfov_deg=39.6)
#: The stereo pair's own focal length and baseline, which set what a range is
#: worth in the depth service -- read off the device, unchanged by any of this.
STEREO_FOCAL_PX = 455.79
BASELINE_CM = 7.5
#: Stored angles are rounded to a tenth, so a replay within this reproduces one.
SAME_DEG = 0.051
#: And ranges to a millimetre.
SAME_M = 0.0015
FRAME = (640, 480)
NAVIGATION = ("127.0.0.1", 8773)
REACH_LIMIT_M = 12.0
OCCUPIED_AT = 50


# --- drawing a look through a given fisheye --------------------------------------


def through(spec: tuple | None) -> None:
    """Make `view` draw 640x480 looks through this fisheye; None for today's."""
    view._LENSES.clear()
    if spec is not None:
        arcmin, bend, centre = spec
        view._LENSES[FRAME] = (math.radians(arcmin / 60.0), bend, centre[0],
                               centre[1], FRAME[0] / 2.0)


def drawn(row: dict[str, Any]) -> tuple | None:
    """(bearing, span, elevation, elevation span), exactly as `store.record` does."""
    got = view.ray({"pose": row["pose"], "bbox": row["bbox"],
                    "observer_pan_deg": row["observer_pan_deg"],
                    "observer_tilt_deg": row["observer_tilt_deg"]},
                   130.0, size=FRAME)
    if got is None:
        return None
    return (got["bearing_deg"], got["span_deg"], got["elevation_deg"],
            got["elevation_span_deg"])


def _same(one: float | None, other: float | None, within: float,
          angle: bool = False) -> bool:
    if one is None or other is None:
        return one is None and other is None
    gap = float(one) - float(other)
    if angle:
        gap = (gap + 180.0) % 360.0 - 180.0
    return abs(gap) <= within


def reproduces(row: dict[str, Any], replayed: tuple | None) -> bool:
    if replayed is None:
        return False
    stored = (row["bearing_deg"], row["span_deg"], row["elevation_deg"],
              row["elevation_span_deg"])
    return (_same(stored[0], replayed[0], SAME_DEG, angle=True)
            and all(_same(a, b, SAME_DEG) for a, b in zip(stored[1:], replayed[1:])))


# --- replaying a range from the depth map kept beside a look ----------------------


def _depth_service():
    """The depth service's own range arithmetic, so a replay is not a copy of it."""
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    for folder in (os.path.join(here, "oak_depth"),):
        if os.path.isdir(folder) and folder not in sys.path:
            sys.path.insert(0, folder)
    import depth_server                                           # noqa: PLC0415

    return depth_server


def extractor(service, lens: Lens):
    """`Depth._range_in` as the service would run it, with this colour lens."""
    fake = types.SimpleNamespace(
        colour_intrinsics={"fx": lens.fx, "fy": lens.fy, "cx": lens.cx,
                           "cy": lens.cy, "width": lens.width,
                           "height": lens.height},
        focal_px=STEREO_FOCAL_PX, baseline_cm=BASELINE_CM)
    fake._secant = types.MethodType(service.Depth._secant, fake)
    import numpy                                                  # noqa: PLC0415

    def extract(depth, box):
        height, width = depth.shape
        return service.Depth._range_in(fake, numpy, depth, width, height, box)
    return extract


def load_depth(folder: str, frame_id: str):
    """The depth map kept beside a look, in millimetres, or None."""
    import numpy                                                  # noqa: PLC0415

    path = os.path.join(folder, f"{frame_id}.depth.gz")
    if not frame_id or not os.path.exists(path):
        return None
    with gzip.open(path, "rb") as handle:
        raw = handle.read()
    length = int.from_bytes(raw[:4], "little")
    meta = json.loads(raw[4:4 + length])
    return numpy.frombuffer(raw[4 + length:], dtype="<u2").reshape(
        meta["height"], meta["width"])


def replay_range(row, depth, oak_lens: Lens, mount: oak.Mount, extract):
    """(range, camera sigma, absent) for one look, as `InspectionRanges` did it.

    The corners are in the chassis frame for a bracket that did not turn with
    the gimbal, and in the gimbal camera's own frame for one that does.
    """
    pan, tilt = row["observer_pan_deg"], row["observer_tilt_deg"]
    if mount.on_gimbal:
        pan = tilt = 0.0
    corners = []
    left, top, right, bottom = row["bbox"]
    for x_frac, y_frac in ((left, top), (right, top), (left, bottom),
                           (right, bottom)):
        direction = view.chassis_direction(x_frac, y_frac, pan or 0.0,
                                           tilt or 0.0, FRAME)
        if direction is None:
            return None, None, NOTHING_TO_MEASURE
        corners.append(direction)
    box = oak.box_for(corners, oak_lens, None, mount)
    if box is None:
        return None, None, OUTSIDE_VIEW
    got = extract(depth, box)
    if got["range_m"] is not None and (abs(got["range_m"] - oak.GUESS_RANGE_M)
                                       > 0.40 * oak.GUESS_RANGE_M):
        again = oak.box_for(corners, oak_lens, got["range_m"], mount)
        if again is not None:
            second = extract(depth, again)
            if second["range_m"] is not None:
                got = second
    if got["range_m"] is None:
        return None, None, NOTHING_TO_MEASURE
    corrected = oak.range_from_gimbal(corners, got["range_m"], mount)
    if corrected is None or corrected <= 0.0:
        return None, None, NOTHING_TO_MEASURE
    return round(corrected, 3), got["sigma_m"], None


# --- where the rover could see, for placing things again --------------------------


def reach_from_navigation():
    """`RoverWorld._world_reach` over the navigation bridge's own grid, or None."""
    import numpy                                                  # noqa: PLC0415

    try:
        with socket.create_connection(NAVIGATION, timeout=8.0) as link:
            link.sendall(json.dumps({"op": "map"}).encode() + b"\n")
            with link.makefile("r") as stream:
                answer = json.loads(stream.readline())
    except (OSError, ValueError):
        return None
    if not answer.get("ok") or not answer.get("data"):
        return None
    width, height = int(answer["width"]), int(answer["height"])
    cells = numpy.frombuffer(zlib.decompress(base64.b64decode(answer["data"])),
                             dtype=numpy.int8).reshape(height, width)
    resolution = float(answer["resolution_m"])
    origin = (float(answer["origin_x_m"]), float(answer["origin_y_m"]))

    def reach(x_m, y_m, bearing_deg):
        step = resolution / 2.0
        dx = math.cos(math.radians(bearing_deg)) * step
        dy = math.sin(math.radians(bearing_deg)) * step
        reached = 0.0
        for count in range(1, int(REACH_LIMIT_M / step) + 1):
            ix = math.floor((float(x_m) + dx * count - origin[0]) / resolution)
            iy = math.floor((float(y_m) + dy * count - origin[1]) / resolution)
            if not (0 <= ix < width and 0 <= iy < height):
                break
            if cells[iy, ix] >= OCCUPIED_AT:
                break
            reached = step * count
        return reached
    return reach


def placement_for(store, entity: dict[str, Any], reach) -> dict[str, Any] | None:
    """Where the resolver's own rules put this thing now, from its own looks."""
    session = entity.get("placement_map_session")
    observations = [one for one in store.observations(entity["id"], limit=24)
                    if one.get("map_session") == session]
    rays = [ray for ray in (resolve.ray_of(one, reach) for one in observations)
            if ray]
    best = locate.best_fix(rays)
    if best is not None:
        return locate.refine(best, rays)
    singles = [one for one in (locate.at_range(ray) for ray in rays) if one]
    if singles:
        chosen = min(singles, key=lambda one: one["uncertainty_m"])
        return dict(chosen, rays_agreeing=1, viewpoints=1)
    return None


# --- the whole run ------------------------------------------------------------------


def plan(store, frames_folder: str, oak_lens_now: Lens) -> dict[str, Any]:
    columns = ("id, frame_id, observed_at, observer_pose_json, bbox_json,"
               " observer_pan_deg, observer_tilt_deg, bearing_deg, span_deg,"
               " elevation_deg, elevation_span_deg, range_m, range_sigma_m,"
               " range_absent, camera")
    rows = []
    for raw in store.db.execute(
            f"SELECT {columns} FROM observations WHERE bearing_deg IS NOT NULL"
            " AND (camera IS NULL OR camera = 'gimbal')"):
        row = dict(raw)
        row["pose"] = json.loads(row["observer_pose_json"] or "null")
        row["bbox"] = json.loads(row["bbox_json"] or "null")
        rows.append(row)

    through(PREVIOUS_LENS)
    replayed = {row["id"]: drawn(row) for row in rows}
    through(None)
    now = {row["id"]: drawn(row) for row in rows}
    kept = [row for row in rows if reproduces(row, replayed[row["id"]])
            and now[row["id"]] is not None]

    service = _depth_service()
    extract_then = extractor(service, PREVIOUS_OAK_LENS)
    extract_now = extractor(service, oak_lens_now)
    changes, range_kept, range_refused = {}, 0, 0
    depth_cache: dict[str, Any] = {}
    for row in kept:
        change = dict(zip(("bearing_deg", "span_deg", "elevation_deg",
                           "elevation_span_deg"), now[row["id"]]))
        frame_id = row["frame_id"] or ""
        if frame_id not in depth_cache:
            depth_cache[frame_id] = load_depth(frames_folder, frame_id)
        depth = depth_cache[frame_id]
        if depth is not None:
            mount = oak.mount_at(row["observed_at"])
            old_mount = (PREVIOUS_CHASSIS_MOUNT if not mount.on_gimbal
                         else mount)
            through(PREVIOUS_LENS)
            then = replay_range(row, depth, PREVIOUS_OAK_LENS, old_mount,
                                extract_then)
            through(None)
            # Within what the reading said it was worth: the depth map kept is
            # the frame after the one that was ranged, a tenth of a second on,
            # and stereo moves by that much between frames.
            within = max(SAME_M, float(row["range_sigma_m"] or 0.0))
            same = (_same(then[0], row["range_m"], within)
                    and (row["range_m"] is not None
                         or then[2] == row["range_absent"]))
            if same:
                again = replay_range(row, depth, oak_lens_now, mount, extract_now)
                sigma = None
                if again[0] is not None:
                    stale_sq = 0.0
                    if then[0] is not None and row["range_sigma_m"] is not None:
                        stale_sq = max(0.0, float(row["range_sigma_m"]) ** 2
                                       - float(then[1] or 0.0) ** 2)
                    sigma = round(math.hypot(float(again[1] or 0.0),
                                             math.sqrt(stale_sq)), 3)
                change.update(range_m=again[0], range_sigma_m=sigma,
                              range_absent=again[2])
                range_kept += 1
            else:
                range_refused += 1
        changes[row["id"]] = (row, change)
    return {"rows": rows, "changes": changes, "range_kept": range_kept,
            "range_refused": range_refused}


def summary(result: dict[str, Any], say=print) -> None:
    rows, changes = result["rows"], result["changes"]
    say(f"{len(rows)} looks with a direction; {len(changes)} reproduce through the "
        f"previous lens and would be redrawn, {len(rows) - len(changes)} would be "
        f"left alone")
    moved = [abs((c["bearing_deg"] - r["bearing_deg"] + 180.0) % 360.0 - 180.0)
             for r, c in changes.values()]
    if moved:
        say(f"  bearings move by a median {statistics.median(moved):.2f} deg, "
            f"90th percentile {sorted(moved)[int(0.9 * (len(moved) - 1))]:.2f}, "
            f"most {max(moved):.2f}")
    lifted = [abs(c["elevation_deg"] - r["elevation_deg"]) for r, c in
              changes.values() if c["elevation_deg"] is not None
              and r["elevation_deg"] is not None]
    if lifted:
        say(f"  elevations move by a median {statistics.median(lifted):.2f} deg, "
            f"most {max(lifted):.2f}")
    say(f"  ranges replayed from their depth maps: {result['range_kept']} reproduce "
        f"and would be redone, {result['range_refused']} do not and would be left")
    ranged = [(r, c) for r, c in changes.values() if "range_m" in c]
    both = [abs(c["range_m"] - r["range_m"]) for r, c in ranged
            if c["range_m"] is not None and r["range_m"] is not None]
    gained = sum(1 for r, c in ranged if c["range_m"] is not None
                 and r["range_m"] is None)
    lost = sum(1 for r, c in ranged if c["range_m"] is None
               and r["range_m"] is not None)
    if both:
        say(f"  of those, {len(both)} keep a range, moving by a median "
            f"{statistics.median(both):.3f} m (90th percentile "
            f"{sorted(both)[int(0.9 * (len(both) - 1))]:.3f}); {gained} gain one "
            f"and {lost} lose one")


def control(store, reach, say=print) -> None:
    """How far placing things again moves them *before* any look is redrawn.

    The resolver builds a thing incrementally, from whichever looks it had at
    the time, so its stored place is not always what a fresh fit of its looks
    gives today. That difference is the method's and not the lens's, and it is
    said here so the two are never read as one.
    """
    moved, unplaceable = [], 0
    for entity in store.placed():
        again = placement_for(store, entity, reach)
        if again is None:
            unplaceable += 1
            continue
        moved.append(math.hypot(again["x_m"] - entity["placement"]["x_m"],
                                again["y_m"] - entity["placement"]["y_m"]))
    if moved:
        same = sum(1 for one in moved if one < 0.01)
        say(f"  control, looks as stored: placing the {len(moved)} things again "
            f"leaves {same} where they are; the rest move by up to "
            f"{max(moved):.2f} m, which is the method and not the lens"
            + (f"; {unplaceable} cannot be placed at all" if unplaceable else ""))


def apply(store, db_path: str, result: dict[str, Any], reach,
          say=print) -> dict[str, Any]:
    backup = f"{db_path}.before-relens-{int(time.time())}"
    target = sqlite3.connect(backup)
    with target:
        store.db.backup(target)
    target.close()
    say(f"copied the database to {backup}")
    with store._lock, store.db:
        for observation_id, (_row, change) in result["changes"].items():
            names = sorted(change)
            store.db.execute(
                "UPDATE observations SET "
                + ", ".join(f"{name} = ?" for name in names) + " WHERE id = ?",
                [change[name] for name in names] + [observation_id])
    moved, unplaced = [], []
    for entity in store.placed():
        before = entity["placement"]
        after = placement_for(store, entity, reach)
        store.place(entity["id"], after, entity["placement_map_session"])
        if after is None:
            unplaced.append(entity["id"])
        else:
            moved.append(math.hypot(after["x_m"] - before["x_m"],
                                    after["y_m"] - before["y_m"]))
    if moved:
        say(f"placed {len(moved)} things again: they moved by a median "
            f"{statistics.median(moved):.2f} m, 90th percentile "
            f"{sorted(moved)[int(0.9 * (len(moved) - 1))]:.2f}, most {max(moved):.2f}")
    if unplaced:
        say(f"{len(unplaced)} things no longer have a place their own looks agree "
            f"on, and are unplaced: {', '.join(unplaced[:12])}"
            + (" ..." if len(unplaced) > 12 else ""))
    return {"backup": backup, "moved": moved, "unplaced": unplaced}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--apply", action="store_true",
                        help="rewrite what reproduces, after copying the database")
    parser.add_argument("--dir", help="the world-state directory, holding "
                                      "world.db and frames/ (default: the "
                                      "rover's own)")
    args = parser.parse_args()
    store = WorldStore(args.dir) if args.dir else WorldStore()
    db_path, frames = store.path, store.frames_dir
    try:
        from world_state.depth_client import SidecarRanger        # noqa: PLC0415

        oak_lens_now = SidecarRanger().lens() or CURRENT_OAK_LENS
    except Exception:                                             # noqa: BLE001
        oak_lens_now = CURRENT_OAK_LENS
    print(f"OAK lens now: fx {oak_lens_now.fx} fy {oak_lens_now.fy}; "
          f"fisheye now: {view._lens_module().LENS[FRAME]}")
    result = plan(store, frames, oak_lens_now)
    summary(result)
    reach = reach_from_navigation()
    if reach is None:
        print("  the navigation bridge gave no map, so things are placed again "
              "with their sight lines unbounded")
    control(store, reach)
    if not args.apply:
        print("\nnothing written; --apply to rewrite what reproduces")
        return 0
    apply(store, db_path, result, reach)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
