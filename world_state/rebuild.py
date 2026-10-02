"""Read the stored looks' ranges again under their outlines, and build the things again.

    world_state_rebuild                  what re-ranging would change, and nothing else
    world_state_rebuild {"apply": true}  back the store up, re-range, and rebuild

Both are a control call on the daemon (`rover_world._tool_world_state_rebuild`), which
holds the rover's own looks off for as long as this runs. Nothing here does that itself.

**What it is for.** When the way a look is measured, or the way the resolver decides,
changes, the things the rover holds were built the old way. `relens.py` answered that
once by moving each thing by what its redrawn looks moved it, and kept every identity.
This throws the things away instead and lets the resolver build them again from the
looks, in the order they were taken. What comes out is what the rover would have built
had the change been in place all along, which is what a replay shows at a desk and what
the store should then hold. It was written for ranging from the region's outline on
2026-10-02 (docs/progress/2026-10-02-ranging-from-the-outline.md).

**Two steps.**

- `rerange`: every look in the current map session whose depth map was kept is read again
  as `outline.read` reads a live look. A region's outline comes from its row, or, for a row
  written before outlines were kept, from the region finder run again on the stored picture
  and matched to the row by box overlap. A range the rover dropped because it was turning
  stays dropped: that rule is about the depth frame's moment, not where in it the reading is
  taken. The part of a range's uncertainty that came from the rover moving is carried over,
  as `relens.py` carried it.
- `rebuild`: every thing of the session is deleted and every look detached, and the looks
  are let back in one at a time, oldest first, with the resolver run after each. Those are
  the passes the rover ran live. New things take fresh numbers from the store's counter,
  so whatever remembered an old one finds nothing rather than something else.
"""
from __future__ import annotations

import json
import math
import sqlite3
import time
from typing import Any

from . import oak, outline
from . import resolve as resolver
from .depth_client import TURNING, Lens

#: The depth camera's colour lens as the service published it, for depth maps kept
#: before the lens went into their header.
DEPTH_LENS = Lens(fx=500.3, fy=500.17, cx=321.23, cy=190.69, width=640, height=360)

#: How much two boxes must overlap for a regenerated outline to be the stored row's.
MATCH_OVERLAP = 0.5

#: A range that moves by less than this is not counted as changed.
CHANGED_M = 0.02


def _overlap(first, second) -> float:
    across = max(0.0, min(first[2], second[2]) - max(first[0], second[0]))
    down = max(0.0, min(first[3], second[3]) - max(first[1], second[1]))
    shared = across * down
    union = ((first[2] - first[0]) * (first[3] - first[1])
             + (second[2] - second[0]) * (second[3] - second[1]) - shared)
    return shared / union if union > 0 else 0.0


def _lens(described: dict[str, Any]):
    got = described.get("lens")
    if not got:
        return DEPTH_LENS
    return Lens(fx=float(got["fx"]), fy=float(got["fy"]), cx=float(got["cx"]),
                cy=float(got["cy"]), width=float(got["width"]),
                height=float(got["height"]))


def rerange(np, store, eyes=None, write: bool = False, say=lambda line: None) -> dict:
    """Read every kept look's ranges again under their outlines. Counts what changed,
    and writes it only when `write`. `eyes` is what draws an outline a row does not
    have; without it such rows are read as their box."""
    session = store.map_session()
    with store._lock:
        rows = [dict(row) for row in store.db.execute(
            "SELECT id, frame_id, observed_at, bbox_json, camera, range_m, range_sigma_m,"
            " range_absent, outline_blob FROM observations WHERE map_session = ?"
            " ORDER BY id", (session,))]
        sizes = {row[0]: (row[1], row[2]) for row in store.db.execute(
            "SELECT id, width, height FROM frames")}
    frames: dict[str, list] = {}
    for row in rows:
        if (row["camera"] or oak.GIMBAL) == oak.GIMBAL and row["bbox_json"]:
            frames.setdefault(row["frame_id"], []).append(row)
    counts = {"looks": 0, "regions": 0, "outlines_drawn": 0, "unmatched": 0,
              "turning_kept_dropped": 0, "changed": 0, "newly_ranged": 0, "lost": 0,
              "by": {}}
    updates = []
    began = time.monotonic()
    for frame_id, looked in frames.items():
        millimetres, described = store.depth(frame_id)
        if millimetres is None:
            continue
        mount = oak.mount_at(looked[0]["observed_at"])
        if not getattr(mount, "on_gimbal", True):
            continue                               # the old chassis bracket: not this
        lens = _lens(described)
        if abs(int(described["width"]) / float(described["height"])
               - float(lens.width) / float(lens.height)) > 0.05:
            continue                               # not this lens's picture
        width, height = sizes.get(frame_id) or (640, 480)
        size = (int(width or 640), int(height or 480))
        counts["looks"] += 1
        drawn = {}
        if eyes is not None and any(not row["outline_blob"] for row in looked):
            jpeg = store.frame(frame_id)
            seen = eyes.look(jpeg) if jpeg else None
            regions = seen.regions if seen is not None and seen.ok else []
            for row in looked:
                if row["outline_blob"]:
                    continue
                bbox = json.loads(row["bbox_json"])
                best = max(regions, default=None,
                           key=lambda region: _overlap(bbox, region.bbox))
                if (best is not None and getattr(best, "outline", b"")
                        and _overlap(bbox, best.bbox) >= MATCH_OVERLAP):
                    drawn[row["id"]] = best.outline
                    counts["outlines_drawn"] += 1
                else:
                    counts["unmatched"] += 1
        boxes = [json.loads(row["bbox_json"]) for row in looked]
        answers = outline.read(np, millimetres, int(described["width"]),
                               int(described["height"]), lens,
                               [(bbox, row["outline_blob"] or drawn.get(row["id"]))
                                for bbox, row in zip(boxes, looked)], size, mount)
        image = outline.DepthImage(np, millimetres, int(described["width"]),
                                   int(described["height"]), lens)
        for bbox, row, answer in zip(boxes, looked, answers):
            counts["regions"] += 1
            if row["range_absent"] == TURNING:
                counts["turning_kept_dropped"] += 1
                if row["id"] in drawn:
                    updates.append((row["range_m"], row["range_sigma_m"],
                                    row["range_absent"], None, drawn[row["id"]], row["id"]))
                continue
            new = answer.get("range_m")
            sigma = answer.get("sigma_m")
            if new is not None and row["range_m"] is not None:
                # The share of the stored figure that was the rover moving, read off
                # against what the box from this same map says the camera was worth.
                was = outline.box_range(image, bbox, size, mount).get("sigma_m") or 0.0
                moving = math.sqrt(max(0.0, (row["range_sigma_m"] or 0.0) ** 2 - was ** 2))
                sigma = round(math.hypot(sigma, moving), 3)
            if new is None and row["range_m"] is not None:
                counts["lost"] += 1
            elif new is not None and row["range_m"] is None:
                counts["newly_ranged"] += 1
            elif new is not None and abs(new - row["range_m"]) > CHANGED_M:
                counts["changed"] += 1
            if new is not None:
                counts["by"][answer["method"]] = counts["by"].get(answer["method"], 0) + 1
            updates.append((new, sigma if new is not None else None,
                            None if new is not None else answer.get("absent"),
                            answer.get("method") if new is not None else None,
                            row["outline_blob"] or drawn.get(row["id"]), row["id"]))
    counts["seconds"] = round(time.monotonic() - began, 1)
    if write and updates:
        with store._lock, store.db:
            store.db.executemany(
                "UPDATE observations SET range_m = ?, range_sigma_m = ?, range_absent = ?,"
                " range_from = ?, outline_blob = ? WHERE id = ?", updates)
        counts["written"] = len(updates)
    say(f"re-ranged {counts['regions']} regions of {counts['looks']} looks: "
        f"{counts['changed']} changed, {counts['newly_ranged']} newly ranged, "
        f"{counts['lost']} lost")
    return counts


def rebuild(store, reach=None, say=lambda line: None) -> dict:
    """Delete the session's things and build them again from its looks, oldest first."""
    session = store.map_session()
    # Rows held back from the resolver while they wait their turn. Negative, which no
    # map session ever is, and one per session so that an interrupted rebuild of one
    # map cannot be mistaken for another's. A rebuild that finds rows already held
    # back picks them up: they are this session's looks, and nothing else holds them.
    held = -1 - session
    began = time.monotonic()
    with store._lock, store.db:
        before = store.db.execute(
            "SELECT COUNT(DISTINCT entity_id) FROM observations"
            " WHERE map_session IN (?, ?) AND entity_id IS NOT NULL",
            (session, held)).fetchone()[0]
        doomed = {row[0] for row in store.db.execute(
            "SELECT DISTINCT entity_id FROM observations WHERE map_session IN (?, ?)"
            " AND entity_id IS NOT NULL", (session, held))}
        doomed |= {row[0] for row in store.db.execute(
            "SELECT id FROM entities WHERE placement_map_session = ?", (session,))}
        store.db.executemany("DELETE FROM entities WHERE id = ?",
                             [(one,) for one in doomed])
        store.db.execute("UPDATE observations SET entity_id = NULL, map_session = ?"
                         " WHERE map_session IN (?, ?)", (held, session, held))
        looks = [row[0] for row in store.db.execute(
            "SELECT inference_id FROM observations WHERE map_session = ?"
            " GROUP BY inference_id ORDER BY MIN(id)", (held,))]
    say(f"deleted {len(doomed)} things; letting {len(looks)} looks back in, oldest first")
    for index, look in enumerate(looks):
        with store._lock, store.db:
            if look is None:
                store.db.execute("UPDATE observations SET map_session = ?"
                                 " WHERE map_session = ? AND inference_id IS NULL",
                                 (session, held))
            else:
                store.db.execute("UPDATE observations SET map_session = ?"
                                 " WHERE map_session = ? AND inference_id = ?",
                                 (session, held, look))
        resolver.resolve(store, map_session=session, reach=reach)
        if index % 50 == 49:
            say(f"  {index + 1} of {len(looks)} looks")
    with store._lock:
        after = store.db.execute(
            "SELECT COUNT(*) FROM entities WHERE placement_map_session = ?",
            (session,)).fetchone()[0]
        attached = store.db.execute(
            "SELECT COUNT(*) FROM observations WHERE map_session = ?"
            " AND entity_id IS NOT NULL", (session,)).fetchone()[0]
    return {"map_session": session, "things_before": before, "things_after": after,
            "looks": len(looks), "regions_attached": attached,
            "seconds": round(time.monotonic() - began, 1)}


def backup(store, label: str = "rebuild") -> str:
    """Copy the store beside itself, as `relens.py` does, before anything is changed."""
    path = f"{store.path}.before-{label}-{int(time.time())}"
    target = sqlite3.connect(path)
    try:
        with store._lock:
            store.db.backup(target)
    finally:
        target.close()
    return path
