"""A record that follows its thing when the thing is moved.

**A moved thing used to lose its record.** A record stays where its thing
stood; the moved thing's next looks meet nothing there and become a new record
or wait unfiled, which is how M5's criterion 4 failed on 2026-10-14
([R-WS-17](../docs/requirements/world-state.md#r-ws-17)). The what/where plan's
step 3 says when something moved: the lidar's change watch saw a place empty
and another filled, and the camera's pictures at the two look alike. This acts
on one such event (`relocate`):

1. **The record** is the one holding most of the pictures remembered at the old
   place. It must have been placed within `FROM_M` of the departure and look
   like the arrival (`PAIR_COS`, masked crops against its masked exemplars).
2. **It abstains for a nearer look-alike.** Another record, filed before the
   move, that looks as much like the arrival (within `RIVAL_COS`) and stands
   nearer the arrival by `NEARER_M` is the likelier to have come. On the
   shakedown of 2026-10-12 that refused the charger armchair's departure paired
   with the other, identical armchair's arrival.
3. **Otherwise** the arrival's pictures are filed under the record, a record
   made only of looks since the departure began is folded into it, the record is
   placed at the arrival, and it keeps three of the arrival's crops as
   exemplars. From then on its place is refitted from its looks since the move
   only (`WorldStore.moved_since`, read by `resolve._replace_placement`).

Everything a move changes is journalled (`moves`, `move_looks`, `move_folds`)
and `undo` puts it back. Replayed over the shakedown and M5 sessions 1 and 2
from the 10-11 snapshot, it acted on three of nine moved events, each the right
object, and kept three of six moved things' records where the rover had kept
one (docs/progress/2026-10-14-identity-across-a-move.md). That was development:
the next staged session is its test.
"""
from __future__ import annotations

import json
import math
import time
from typing import Any

import numpy as np

from . import locate
from .store import EXEMPLARS

#: How near the departure the record must have been placed, in metres.
FROM_M = 1.0
#: How alike the arrival and a record must look (step 3's pairing threshold).
PAIR_COS = 0.7
#: How much less alike than the record a rival may look and still be a rival.
RIVAL_COS = 0.05
#: How much nearer the arrival a rival must stand to be preferred, in metres.
NEARER_M = 0.3
#: A rival must have been filed this long before the arrival's first look: one
#: made from the arrival itself, or in the moment of the move, is not one.
RIVAL_BEFORE_S = 600.0
#: How far a moved record is said to be out until its own looks refit it.
PLACED_SIGMA_M = 0.3
#: How many of the arrival's crops the record keeps as exemplars.
ADDED_EXEMPLARS = 3
#: A masked DINOv2 vector, float32 x 384, in bytes.
WIDTH = 384 * 4
#: What a moved placement keeps of the old one: what the thing is, not where.
KEPT = ("extent_m", "height_m", "height_sigma_m", "height_above_floor_m")


def _unit(blob: bytes):
    v = np.frombuffer(blob, dtype=np.float32).astype(np.float64)
    n = np.linalg.norm(v)
    return v / n if n > 0 else v


def _placement(row) -> dict[str, Any] | None:
    raw = row["placement_json"] if row is not None else None
    try:
        p = json.loads(raw) if isinstance(raw, str) else raw
    except ValueError:
        return None
    return p if isinstance(p, dict) and p.get("x_m") is not None else None


def _point(value) -> tuple[float, float] | None:
    try:
        x, y = (float(v) for v in value)
    except (TypeError, ValueError):
        return None
    return (x, y) if math.isfinite(x) and math.isfinite(y) else None


def _ids(value) -> list[int]:
    try:
        return [int(v) for v in value or []]
    except (TypeError, ValueError):
        return []


def relocate(store, event: dict[str, Any], session: int,
             now: float | None = None) -> dict[str, Any]:
    """Act on one moved event: `{"from": [x, y], "to": [x, y], "regions": [ids
    of the arrival's pictures], "remembered": [ids of pictures at the old
    place]}`. Returns `done` ("moved" or "abstained") and a sentence why."""
    origin, target = _point(event.get("from")), _point(event.get("to"))
    remembered, arrival = _ids(event.get("remembered")), _ids(event.get("regions"))
    if origin is None or target is None or not remembered or not arrival:
        return {"ok": False, "error": "a moved event needs from, to, regions and remembered"}
    now = time.time() if now is None else float(now)
    with store._lock, store.db:
        db = store.db
        marks = ",".join("?" * len(remembered + arrival))
        rows = {r["id"]: r for r in db.execute(
            f"SELECT id, entity_id, observed_at, inference_id, note, dino_blob,"
            f" dino_alone_blob FROM observations WHERE id IN ({marks})",
            remembered + arrival)}
        held = [rows[i]["entity_id"] for i in remembered if i in rows and rows[i]["entity_id"]]
        if not held:
            return _said("abstained", "the old place's pictures are on no record")
        counts = sorted(((held.count(e), e) for e in set(held)), reverse=True)
        if len(counts) > 1 and counts[0][0] == counts[1][0]:
            return _said("abstained", "the old place's pictures are split evenly over %s and %s"
                         % (counts[0][1], counts[1][1]))
        record = counts[0][1]
        entity = db.execute("SELECT * FROM entities WHERE id = ?", (record,)).fetchone()
        placed = _placement(entity)
        if placed is None or entity["placement_map_session"] != session:
            return _said("abstained", f"{record} has no place on this map")
        gap = math.hypot(placed["x_m"] - origin[0], placed["y_m"] - origin[1])
        if gap > FROM_M:
            return _said("abstained", f"{record} was placed {gap:.2f} m from the departure")
        present = [i for i in arrival if i in rows]
        vectors = [_unit(rows[i]["dino_alone_blob"]) for i in present
                   if rows[i]["dino_alone_blob"]]
        if not vectors:
            return _said("abstained", "the arrival has no masked crops")
        seen = np.stack(vectors)

        def likeness(row) -> float | None:
            blob = row["exemplars_alone"] or b""
            kept = [_unit(blob[k:k + WIDTH]) for k in range(0, len(blob) - WIDTH + 1, WIDTH)]
            return float((seen @ np.stack(kept).T).max()) if kept else None

        own = likeness(entity)
        if own is None or own < PAIR_COS:
            return _said("abstained", f"{record} does not look like the arrival"
                         + ("" if own is None else f" ({own:.2f})"))
        began = min(float(rows[i]["observed_at"]) for i in present)
        mine = math.hypot(placed["x_m"] - target[0], placed["y_m"] - target[1])
        for other in db.execute("SELECT * FROM entities WHERE id != ? AND placement_json"
                                " IS NOT NULL AND placement_map_session = ?",
                                (record, session)):
            where = _placement(other)
            if where is None:
                continue
            distance = math.hypot(where["x_m"] - target[0], where["y_m"] - target[1])
            if distance >= mine - NEARER_M:
                continue
            first = db.execute("SELECT MIN(observed_at) FROM observations WHERE entity_id = ?",
                               (other["id"],)).fetchone()[0]
            if first is None or first >= began - RIVAL_BEFORE_S:
                continue
            cos = likeness(other)
            if cos is not None and cos >= PAIR_COS and cos >= own - RIVAL_COS:
                return _said("abstained",
                             f"{other['id']} looks as much like the arrival ({cos:.2f} against"
                             f" {own:.2f}) and stands {distance:.2f} m from it, against"
                             f" {mine:.2f} m for {record}")
        # Acting: journal first, in the same transaction as the change.
        move_id = db.execute(
            "INSERT INTO moves(applied_at, map_session, entity_id, event_json,"
            " placement_before, moved_since, exemplars_before, exemplars_alone_before)"
            " VALUES(?,?,?,?,?,?,?,?)",
            (now, session, record, json.dumps(event), entity["placement_json"], began,
             entity["exemplars"], entity["exemplars_alone"])).lastrowid
        folded, filed = [], []
        why = "moved with the thing it shows: from %.2f, %.2f to %.2f, %.2f" % (*origin, *target)
        for i in present:
            current = db.execute("SELECT entity_id FROM observations WHERE id = ?",
                                 (i,)).fetchone()[0]
            if current == record or current in folded:
                continue
            if current:
                first = db.execute("SELECT MIN(observed_at) FROM observations WHERE"
                                   " entity_id = ?", (current,)).fetchone()[0]
                if first is not None and first >= began - 1.0 and _fold(db, store, move_id,
                                                                        record, current):
                    folded.append(current)
                    continue
            look = rows[i]["inference_id"]
            if look is not None and db.execute(
                    "SELECT 1 FROM observations WHERE entity_id = ? AND inference_id = ?",
                    (record, look)).fetchone():
                continue  # a thing holds one region of any one picture
            db.execute("INSERT OR IGNORE INTO move_looks(move_id, observation_id, entity_before,"
                       " note_before) VALUES(?,?,?,?)", (move_id, i, current, rows[i]["note"]))
            db.execute("UPDATE observations SET entity_id = ?, note = ? WHERE id = ?",
                       (record, why, i))
            filed.append(i)
        moved = {key: placed[key] for key in KEPT if placed.get(key) is not None}
        moved.update({"x_m": round(target[0], 3), "y_m": round(target[1], 3),
                      "uncertainty_m": PLACED_SIGMA_M, "error_major_m": PLACED_SIGMA_M,
                      "error_minor_m": PLACED_SIGMA_M, "error_major_deg": 0.0,
                      "viewpoints": 1, "rays_agreeing": 0,
                      "moved_from": [round(origin[0], 3), round(origin[1], 3)],
                      "moved_at": round(now, 3)})
        moved["stated_uncertainty_m"] = locate.stated_uncertainty(moved)
        exemplars = entity["exemplars"] or b""
        alone = entity["exemplars_alone"] or b""
        for i in present[:ADDED_EXEMPLARS]:
            if rows[i]["dino_blob"] and len(rows[i]["dino_blob"]) == WIDTH:
                exemplars = (exemplars if len(exemplars) % WIDTH == 0 else b"") + rows[i]["dino_blob"]
                if rows[i]["dino_alone_blob"] and len(rows[i]["dino_alone_blob"]) == WIDTH:
                    alone = (alone if len(alone) % WIDTH == 0 else b"") + rows[i]["dino_alone_blob"]
        count = db.execute("SELECT COUNT(*), MAX(observed_at) FROM observations"
                           " WHERE entity_id = ?", (record,)).fetchone()
        db.execute("UPDATE entities SET placement_json = ?, placement_uncertainty_m = ?,"
                   " placement_map_session = ?, placement_updated_at = ?, exemplars = ?,"
                   " exemplars_alone = ?, observation_count = ?, last_seen_at = ? WHERE id = ?",
                   (json.dumps(moved), PLACED_SIGMA_M, session, now,
                    exemplars[-WIDTH * EXEMPLARS:] or None, alone[-WIDTH * EXEMPLARS:] or None,
                    count[0], count[1] or now, record))
    return _said("moved", f"{record} (likeness {own:.2f}, placed {gap:.2f} m from the"
                 f" departure) to the arrival; filed {len(filed)}, folded {folded or 'none'}",
                 entity_id=record, move_id=move_id, filed=filed, folded=folded)


def _fold(db, store, move_id: int, keep: str, gone: str) -> bool:
    """Fold a record made of the arrival into the moved one, journalled."""
    row = db.execute("SELECT * FROM entities WHERE id = ?", (gone,)).fetchone()
    if row is None:
        return False
    looks = db.execute("SELECT id, note FROM observations WHERE entity_id = ?",
                       (gone,)).fetchall()
    result = store._merge_rows(keep, gone)
    if not result.get("ok"):
        return False
    fields = {k: row[k] for k in row.keys() if k not in ("exemplars", "exemplars_alone")}
    db.execute("INSERT OR REPLACE INTO move_folds(move_id, entity_id, row_json, exemplars,"
               " exemplars_alone) VALUES(?,?,?,?,?)",
               (move_id, gone, json.dumps(fields), row["exemplars"], row["exemplars_alone"]))
    for look in looks:
        db.execute("INSERT OR IGNORE INTO move_looks(move_id, observation_id, entity_before,"
                   " note_before) VALUES(?,?,?,?)", (move_id, look["id"], gone, look["note"]))
    return True


def undo(store, move_id: int) -> dict[str, Any]:
    """Put one move back: its looks where they were, its folded records back,
    the record's place and exemplars as they stood."""
    with store._lock, store.db:
        db = store.db
        move = db.execute("SELECT * FROM moves WHERE id = ?", (int(move_id),)).fetchone()
        if move is None:
            return {"ok": False, "error": f"no move {move_id}"}
        if move["rolled_back_at"] is not None:
            return {"ok": False, "error": f"move {move_id} was already put back"}
        for fold in db.execute("SELECT * FROM move_folds WHERE move_id = ?", (move["id"],)):
            fields = json.loads(fold["row_json"])
            fields["exemplars"], fields["exemplars_alone"] = fold["exemplars"], fold["exemplars_alone"]
            names = ",".join(fields)
            db.execute(f"INSERT OR REPLACE INTO entities({names}) VALUES"
                       f"({','.join('?' * len(fields))})", list(fields.values()))
        touched = {move["entity_id"]}
        for look in db.execute("SELECT * FROM move_looks WHERE move_id = ?", (move["id"],)):
            db.execute("UPDATE observations SET entity_id = ?, note = ? WHERE id = ?",
                       (look["entity_before"], look["note_before"], look["observation_id"]))
            if look["entity_before"]:
                touched.add(look["entity_before"])
        before = _placement({"placement_json": move["placement_before"]})
        db.execute("UPDATE entities SET placement_json = ?, placement_uncertainty_m = ?,"
                   " exemplars = ?, exemplars_alone = ? WHERE id = ?",
                   (move["placement_before"], (before or {}).get("uncertainty_m"),
                    move["exemplars_before"], move["exemplars_alone_before"], move["entity_id"]))
        for entity_id in touched:
            count = db.execute("SELECT COUNT(*), MAX(observed_at) FROM observations"
                               " WHERE entity_id = ?", (entity_id,)).fetchone()
            db.execute("UPDATE entities SET observation_count = ?, last_seen_at ="
                       " COALESCE(?, last_seen_at) WHERE id = ?", (count[0], count[1], entity_id))
        db.execute("UPDATE moves SET rolled_back_at = ? WHERE id = ?", (time.time(), move["id"]))
    return {"ok": True, "move_id": int(move_id), "entity_id": move["entity_id"]}


def _said(done: str, why: str, **more) -> dict[str, Any]:
    return {"ok": True, "done": done, "why": why, **more}
