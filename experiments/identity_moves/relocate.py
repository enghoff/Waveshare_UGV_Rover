"""A record that follows its thing when the thing is moved: step 3's moved event
acted on in the world store (R-WS-17, M5 criterion 4).

`relocate(store, event, session)` takes one of step 3's moved events (from,
to, the arrival's regions, the remembered regions at the old place) and

1. finds the record the remembered regions are filed under -- the thing that
   left -- and requires it to have been placed within FROM_M of the departure;
2. abstains when another record filed before the move looks as much like the
   arrival as the record does (within RIVAL_COS, and at least PAIR_COS, against
   masked exemplars) and stands nearer the arrival by NEARER_M: two look-alikes
   and the nearer one is the likelier to have come (the shakedown paired the charger armchair's departure with the
   other armchair's arrival, 1.8 m from it against 0.9 m);
3. files the arrival's regions under the record, folding in any record made
   only of looks taken since the departure began, places the record at the
   arrival, and gives it the arrival's crops as exemplars.

`moved_since` is what the record's placement is refitted from afterwards: only
its looks since the move (`refit_since`), so the old place's looks do not pull
it back.
"""
import json
import math

import numpy as np

#: How near the departure the record must have been placed, in metres.
FROM_M = 1.0
#: How alike an arrival and a record must look to be look-alikes (step 3's
#: PAIR_COS).
PAIR_COS = 0.7
#: How much less alike than the record a rival may look and still be a rival.
RIVAL_COS = 0.05
#: How much nearer the arrival another look-alike must stand to be preferred.
NEARER_M = 0.3
#: Where a relocated record is said to be, until its own looks refit it.
PLACED_SIGMA_M = 0.3
#: How many of the arrival's crops become exemplars.
EXEMPLARS = 3
#: A masked DINOv2 vector's width in bytes (float32, 384).
WIDTH = 384 * 4

#: Record -> the time it was last moved. The experiment's stand-in for a column.
moved_since = {}


def _unit(blob):
    v = np.frombuffer(blob, dtype=np.float32).astype(np.float64)
    n = np.linalg.norm(v)
    return v / n if n > 0 else v


def _placed(entity):
    p = entity["placement_json"]
    p = json.loads(p) if isinstance(p, str) else p
    return p if isinstance(p, dict) and p.get("x_m") is not None else None


def relocate(store, event, session, now):
    """Act on one moved event; returns (what was done, why)."""
    db = store.db
    remembered = [int(i) for i in event.get("remembered", [])]
    arrival = [int(i) for i in event.get("regions", [])]
    rows = {r["id"]: dict(r) for r in db.execute(
        "SELECT id, entity_id, observed_at, dino_alone_blob, dino_blob FROM observations"
        " WHERE id IN (%s)" % ",".join("?" * len(remembered + arrival)),
        remembered + arrival)}
    held = [rows[i]["entity_id"] for i in remembered if i in rows and rows[i]["entity_id"]]
    if not held:
        return "abstained", "the old place's pictures are on no record"
    counts = sorted(((held.count(e), e) for e in set(held)), reverse=True)
    if len(counts) > 1 and counts[0][0] == counts[1][0]:
        return "abstained", "the old place's pictures are split evenly over %s" % [e for _, e in counts[:2]]
    record = counts[0][1]
    entity = db.execute("SELECT * FROM entities WHERE id = ?", (record,)).fetchone()
    placed = _placed(entity) if entity else None
    if placed is None:
        return "abstained", "%s has no place" % record
    fx, fy = event["from"]
    tx, ty = event["to"]
    gap = math.hypot(placed["x_m"] - fx, placed["y_m"] - fy)
    if gap > FROM_M:
        return "abstained", "%s was placed %.2f m from the departure" % (record, gap)
    vecs = [_unit(rows[i]["dino_alone_blob"]) for i in arrival
            if i in rows and rows[i]["dino_alone_blob"]]
    if not vecs:
        return "abstained", "the arrival has no masked crops"
    arrival_vecs = np.stack(vecs)

    def likeness(row):
        blob = row["exemplars_alone"] or b""
        ex = [_unit(blob[k:k + WIDTH]) for k in range(0, len(blob) - WIDTH + 1, WIDTH)]
        return float((arrival_vecs @ np.stack(ex).T).max()) if ex else None
    own = likeness(entity)
    if own is None or own < PAIR_COS:
        return "abstained", "%s does not look like the arrival (%s)" % (record, own)
    began = min(rows[i]["observed_at"] for i in arrival if i in rows)
    mine = math.hypot(placed["x_m"] - tx, placed["y_m"] - ty)
    for other in db.execute("SELECT * FROM entities WHERE id != ? AND placement_json IS NOT NULL"
                            " AND placement_map_session = ?", (record, session)):
        p = _placed(other)
        if p is None:
            continue
        d = math.hypot(p["x_m"] - tx, p["y_m"] - ty)
        if d >= mine - NEARER_M:
            continue
        first = db.execute("SELECT MIN(observed_at) FROM observations WHERE entity_id = ?",
                           (other["id"],)).fetchone()[0]
        if first is None or first >= began - 600.0:
            continue  # made from the arrival itself, or the moment's pictures
        cos = likeness(other)
        if cos is not None and cos >= PAIR_COS and cos >= own - RIVAL_COS:
            return "abstained", ("%s looks like the arrival as much (%.2f against %.2f) and stands"
                                 " %.2f m from it, against %.2f m for %s"
                                 % (other["id"], cos, own, d, mine, record))
    folded, filed = [], []
    for i in arrival:
        if i not in rows:
            continue
        current = db.execute("SELECT entity_id FROM observations WHERE id = ?", (i,)).fetchone()[0]
        if current == record:
            continue
        if current:
            first = db.execute("SELECT MIN(observed_at) FROM observations WHERE entity_id = ?",
                               (current,)).fetchone()[0]
            if first is not None and first >= began - 1.0 and current not in folded:
                result = store.merge(record, current)
                if result.get("ok"):
                    folded.append(current)
                    continue
        look = db.execute("SELECT inference_id FROM observations WHERE id = ?", (i,)).fetchone()[0]
        if db.execute("SELECT 1 FROM observations WHERE entity_id = ? AND inference_id = ?",
                      (record, look)).fetchone():
            continue  # a thing holds one region of any one picture
        filed.append(i)
    if filed:
        store.attach(record, filed, "moved with the thing it shows: %s to %s, known %.0f"
                     % (event["from"], event["to"], now))
    store.place(record, dict(placed, x_m=round(float(tx), 3), y_m=round(float(ty), 3),
                             uncertainty_m=PLACED_SIGMA_M, error_major_m=PLACED_SIGMA_M,
                             error_minor_m=PLACED_SIGMA_M, moved_from=[fx, fy]), session)
    for i in arrival[:EXEMPLARS]:
        if i in rows and rows[i]["dino_blob"]:
            store.add_exemplar(record, rows[i]["dino_blob"], alone=rows[i]["dino_alone_blob"] or b"")
    moved_since[record] = began
    return "moved", ("%s (likeness %.2f) from %.2f m of the departure to the arrival; filed %d,"
                     " folded %s" % (record, own, gap, len(filed), folded))


def refit_since(original):
    """Wrap resolve._replace_placement so a moved record refits from its looks
    since the move only."""
    def replace(store, entity_id, session, reach=None):
        since = moved_since.get(entity_id)
        if since is None:
            return original(store, entity_id, session, reach)
        keep = store.observations

        def recent(entity_id=None, limit=200, **kw):
            rows = keep(entity_id=entity_id, limit=limit, **kw)
            return [r for r in rows if (r.get("observed_at") or 0) >= since]
        store.observations = recent
        try:
            return original(store, entity_id, session, reach)
        finally:
            store.observations = keep
    return replace
