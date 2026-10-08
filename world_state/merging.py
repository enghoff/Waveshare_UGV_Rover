"""Join things the resolver split, as a person's act that can be undone.

    world_state_merge                                   what it would join, and nothing else
    world_state_merge {"apply": [[keep, gone], ...]}    join those pairs, journalling first
    world_state_merge {"rollback": true}                put back the last run applied
    world_state_merge {"runs": true}                    every run, and whether it was put back

All four are a control call on the daemon (`rover_world._tool_world_state_merge`),
which holds the rover's own looks and its resolver off while it writes. Nothing here
does that itself.

**What it is for.** The resolver commits each look the moment it is offered and never
compares two placed things with each other again, so a thing founded twice stays
twice: on 2026-10-04 the cow painting was `object:249` and `object:340`, and the
painting of a building by the sea `object:383` and `object:400`. This scores every pair
of placed things in the current map session by one number, the log-likelihood ratio
that they are one object rather than two, and proposes the pairs above zero. **A person
decides which to join**: the proposals come with the looks to draw them from, and
`merge_sheet.py` draws them.

**The score is two parts that add.**

- *Appearance*: the mean, over pairs of their looks, of the evidence that two looks
  show one object. That evidence is a weighted sum of three cosines -- the plain crop,
  the masked crop and SigLIP -- with weights fitted by logistic regression on 412
  labelled looks of map session 67 (`APPEARANCE_WEIGHTS`).
- *Geometry*: the track-to-track log-likelihood ratio of the two placements -- a
  Gaussian on their error ellipses, each widened by half the thing's width, under "one
  object", and the session's own density of placed things under "two". Things stand
  so close together here that this can rule a merge out but never adds more than about
  2 for one.

Never proposed: a pair whose crops do not on their own favour one object
(`LOOKS_ALIKE_ABOVE`), a pair sharing a picture (two regions of one picture are two
things, and `WorldStore.merge` refuses it), a pair further apart than `MAX_APART_M`, a
thing not placed in this map session, and a pair of looks from different perception
backends, whose vectors are not comparable (R-WS-4). Each thing is in at most one proposal; asking
again after joining gives the next round.

Measured on that session ([the progress entry](../docs/progress/2026-10-04-one-score-for-appearance-and-position.md)):
of 26 proposals drawn at random and judged on contact sheets, 20 were plainly one object
and none plainly two; run after the resolver in a replay, it took the look-alike pairs
within 0.5 m from 24 to 1 and put no two labelled objects together. It does nothing for
looks filed under the wrong thing.

**Undoing it is exact for what it touched.** Applying writes a journal first, in the
same transaction: both things of every pair, row for row, and every look moved, with
the thing and the note it had. Rolling back puts those rows back and those looks back.
A look recorded since stays where the resolver put it. Only the last run not yet rolled
back can be, and not across a clear or a change of map.
"""
from __future__ import annotations

import json
import math
import time
from itertools import combinations
from typing import Any

#: The weights of the plain DINO, masked DINO and SigLIP cosines of two looks, and the
#: offset, in the log-likelihood ratio that the two show one object. Fitted on
#: 2026-10-04 by logistic regression over 65,885 pairs of labelled looks of map session
#: 67 (4,212 of one object), with the training sample's prior odds taken out of the
#: offset so that zero means even evidence. It told one object from two with an AUC of
#: 0.956 there, against 0.944 for the plain cosine alone. The labels are mostly the
#: coding agent's; see `captures/2026-10-04-association-likelihood/pairwise.py`.
APPEARANCE_WEIGHTS = (5.760, 2.959, 15.276)
APPEARANCE_OFFSET = -16.529

#: How many of a thing's looks its appearance is judged from, spread evenly through
#: its history. Forty against forty is 1,600 pairs, which is plenty for a mean and
#: keeps a pass over every pair of a few hundred things to seconds.
SAMPLE = 40

#: Further apart than this, two placements are not offered as one thing whatever they
#: look like. The farthest true duplicate seen was 2.4 m apart as stored.
MAX_APART_M = 4.0

#: The floor on either axis of a placement's error ellipse, in metres. A fit over
#: many rays can claim a centimetre, and two such claims a few centimetres apart would
#: otherwise read as two objects.
ELLIPSE_FLOOR_M = 0.05

# Nominal 99.9% two-dimensional Gaussian distance. The recorded ellipses are
# not calibrated probabilities; this is a hard compatibility threshold (R-WS-8).
MAX_ELLIPSE_DISTANCE2 = 13.815510557964274

#: A pair is proposed when its score is above this, and its appearance on its own is
#: above `LOOKS_ALIKE_ABOVE`. The second rule is the measurement that geometry can rule
#: a merge out but cannot make the case for one: in this room two placements a few
#: centimetres apart score +1 to +2 on position alone, and on the store of 2026-10-04
#: that carried four pairs whose crops said, if anything, two objects.
PROPOSE_ABOVE = 0.0
LOOKS_ALIKE_ABOVE = 0.0

#: How many looks of each side a proposal names, for a person to look at.
SHOWN = 6

#: Co-fit: two records are proposed as one object when at least this share of
#: each one's looks would also have been filed to the other -- its allowance,
#: height and appearance at filing's own bar, the test `aimed.also_fits` applies
#: to a look aimed at a thing. Fixed on 2026-10-08 against the frozen labels: at
#: 0.3 both ways it joined 7 of 8 joinable one-object pairs of the independent
#: sets, where `propose` joined 3, with 1 wrong of 98, and on the rover's store it
#: takes the rug under the dining table from seven records to two
#: (docs/progress/2026-10-08-merging-by-cofit.md). It compares one look with the
#: other record, so it reaches what record-to-record appearance cannot: an object
#: seen in pieces from different sides, and records that claim their positions
#: more tightly than they hold them.
COFIT_SHARE = 0.3
#: How many of a record's looks co-fit tries, spread through its history.
COFIT_SAMPLE = 40

#: The entity columns held as bytes, which go into the journal as they are.
_BLOBS = ("exemplars", "exemplars_alone")


def _unit(blob):
    import numpy as np

    if not blob:
        return None
    vector = np.frombuffer(blob, dtype="<f4").astype("float64")
    length = float(np.linalg.norm(vector))
    return None if length < 1e-9 else vector / length


class _Thing:
    """What the score needs of one placed thing: where, how sure, and what it looks
    like, from a sample of its looks."""

    def __init__(self, row: dict[str, Any], looks: list[dict[str, Any]]):
        import numpy as np

        self.id = row["id"]
        self.created_at = row["created_at"]
        self.placement = json.loads(row["placement_json"])
        self.count = len(looks)
        self.pictures = {look["inference_id"] for look in looks
                         if look["inference_id"] is not None}
        usable = [look for look in looks if look.get("dino_blob") and look.get("siglip_blob")]
        if len(usable) > SAMPLE:
            step = (len(usable) - 1) / (SAMPLE - 1)
            usable = [usable[round(i * step)] for i in range(SAMPLE)]
        self.shown = [look["id"] for look in looks[:: max(1, len(looks) // SHOWN)]][:SHOWN]
        plain = [_unit(look["dino_blob"]) for look in usable]
        masked = [_unit(look.get("dino_alone_blob")) for look in usable]
        semantic = [_unit(look["siglip_blob"]) for look in usable]
        keep = [i for i in range(len(usable))
                if plain[i] is not None and semantic[i] is not None]
        self.backend = np.array([str(usable[i].get("vectors_from") or "") for i in keep])
        self.plain = np.array([plain[i] for i in keep]) if keep else None
        self.masked = (np.array([masked[i] if masked[i] is not None
                                 and len(masked[i]) == len(plain[i]) else plain[i]
                                 for i in keep]) if keep else None)
        self.semantic = np.array([semantic[i] for i in keep]) if keep else None

    def covariance(self):
        import numpy as np

        p = self.placement
        major = max(float(p.get("error_major_m") or p.get("uncertainty_m") or 0.3),
                    ELLIPSE_FLOOR_M)
        minor = max(float(p.get("error_minor_m") or major), ELLIPSE_FLOOR_M)
        angle = math.radians(float(p.get("error_major_deg") or 0.0))
        turn = np.array([[math.cos(angle), -math.sin(angle)],
                         [math.sin(angle), math.cos(angle)]])
        half = float(p.get("extent_m") or 0.0) / 2.0
        return turn @ np.diag([major * major, minor * minor]) @ turn.T + np.eye(2) * half * half


def appearance(a: _Thing, b: _Thing) -> float | None:
    """Mean evidence over pairs of their looks that the two show one object; None
    when no pair of looks could be compared."""
    if a.plain is None or b.plain is None:
        return None
    if a.plain.shape[1] != b.plain.shape[1] or a.semantic.shape[1] != b.semantic.shape[1]:
        return None
    w_plain, w_masked, w_semantic = APPEARANCE_WEIGHTS
    evidence = (w_plain * (a.plain @ b.plain.T) + w_masked * (a.masked @ b.masked.T)
                + w_semantic * (a.semantic @ b.semantic.T) + APPEARANCE_OFFSET)
    comparable = a.backend[:, None] == b.backend[None, :]
    if not comparable.any():
        return None
    return float(evidence[comparable].mean())


def geometry(a: _Thing, b: _Thing, density: float) -> float:
    """The track-to-track log-likelihood ratio of the two placements: one object
    seen twice, against two independent things in a room this full of them."""
    import numpy as np

    apart = np.array([a.placement["x_m"] - b.placement["x_m"],
                      a.placement["y_m"] - b.placement["y_m"]])
    both = a.covariance() + b.covariance()
    squared = float(apart @ np.linalg.solve(both, apart))
    if not math.isfinite(squared) or squared > MAX_ELLIPSE_DISTANCE2:
        return -math.inf
    return (-0.5 * squared - math.log(2.0 * math.pi * math.sqrt(np.linalg.det(both)))
            - math.log(density))


def _things(store, session: int) -> list[_Thing]:
    with store._lock:
        rows = [dict(row) for row in store.db.execute(
            "SELECT * FROM entities WHERE placement_json IS NOT NULL"
            " AND placement_map_session = ? ORDER BY created_at, id", (session,))]
        looks: dict[str, list] = {}
        for row in store.db.execute(
                "SELECT id, entity_id, inference_id, observed_at, dino_blob,"
                " dino_alone_blob, siglip_blob, vectors_from FROM observations"
                " WHERE entity_id IS NOT NULL ORDER BY observed_at, id"):
            looks.setdefault(row["entity_id"], []).append(dict(row))
    return [_Thing(row, looks[row["id"]]) for row in rows if row["id"] in looks]


def _cofit_share(store, looks: list[dict[str, Any]], other: "_Thing") -> float | None:
    """Share of `looks` that would also have been filed to `other`."""
    from . import locate, resolve

    tried = hits = 0
    for row in looks:
        ray = resolve.ray_of(row, None)
        if ray is None:
            continue
        tried += 1
        if resolve._allowance_used(other.placement, ray) is None:
            continue
        if not locate.stands_as_high(other.placement, ray):
            continue
        seen = resolve.appearance(store, other.id, row.get("dino_blob") or b"")
        if seen is not None and seen >= resolve.DIFFERENT_THING:
            hits += 1
    return hits / tried if tried else None


def cofit_pairs(store, share: float = COFIT_SHARE) -> list[dict[str, Any]]:
    """Pairs of this map session's things whose looks fit each other, strongest first.

    Never a pair sharing a picture or further apart than `MAX_APART_M`, as for
    `propose`; the keeper is the side with more looks. Each thing is in at most
    one pair, so that the list can be applied as it stands; ask again after
    joining for the next round.
    """
    session = store.map_session()
    things = {thing.id: thing for thing in _things(store, session)}
    sampled = {}
    for thing_id in things:
        rows = [row for row in store.observations(thing_id, vectors=True)
                if row.get("bearing_deg") is not None]
        if len(rows) > COFIT_SAMPLE:
            step = (len(rows) - 1) / (COFIT_SAMPLE - 1)
            rows = [rows[round(i * step)] for i in range(COFIT_SAMPLE)]
        sampled[thing_id] = rows
    scored = []
    for a, b in combinations(sorted(things), 2):
        first, second = things[a], things[b]
        if first.pictures & second.pictures:
            continue
        if math.hypot(first.placement["x_m"] - second.placement["x_m"],
                      first.placement["y_m"] - second.placement["y_m"]) > MAX_APART_M:
            continue
        one = _cofit_share(store, sampled[a], second)
        if one is None or one < share:
            continue
        other = _cofit_share(store, sampled[b], first)
        if other is None or other < share:
            continue
        keep, gone = (a, b) if first.count >= second.count else (b, a)
        scored.append({"keep": keep, "gone": gone, "cofit": round(min(one, other), 3),
                       "apart_m": round(math.hypot(
                           first.placement["x_m"] - second.placement["x_m"],
                           first.placement["y_m"] - second.placement["y_m"]), 2)})
    scored.sort(key=lambda pair: -pair["cofit"])
    used: set = set()
    pairs = []
    for pair in scored:
        if pair["keep"] in used or pair["gone"] in used:
            continue
        used.update((pair["keep"], pair["gone"]))
        pairs.append(pair)
    return pairs


def propose(store) -> dict[str, Any]:
    """Which pairs of this map session's things look like one object. Writes nothing."""
    began = time.time()
    session = store.map_session()
    things = _things(store, session)
    if len(things) < 2:
        return {"ok": True, "map_session": session, "things": len(things),
                "proposals": [], "seconds": round(time.time() - began, 2)}
    xs = [thing.placement["x_m"] for thing in things]
    ys = [thing.placement["y_m"] for thing in things]
    # Placed things per square metre of the ground they cover, with a metre's margin
    # so that a handful of things in a line does not read as infinitely dense.
    density = len(things) / ((max(xs) - min(xs) + 1.0) * (max(ys) - min(ys) + 1.0))
    scored = []
    for a, b in combinations(things, 2):
        apart = math.hypot(a.placement["x_m"] - b.placement["x_m"],
                           a.placement["y_m"] - b.placement["y_m"])
        if apart > MAX_APART_M or a.pictures & b.pictures:
            continue
        looks_alike = appearance(a, b)
        if looks_alike is None or looks_alike <= LOOKS_ALIKE_ABOVE:
            continue
        stands = geometry(a, b, density)
        if looks_alike + stands > PROPOSE_ABOVE:
            scored.append((looks_alike + stands, looks_alike, stands, apart, a, b))
    scored.sort(key=lambda one: -one[0])
    taken: set = set()
    proposals = []
    for score, looks_alike, stands, apart, a, b in scored:
        if a.id in taken or b.id in taken:
            continue
        taken.update((a.id, b.id))
        # The thing with more looks keeps its name; on a tie, the older one.
        keep, gone = sorted((a, b), key=lambda thing: (-thing.count, thing.created_at))
        proposals.append({
            "n": len(proposals) + 1, "keep": keep.id, "gone": gone.id,
            "score": round(score, 2), "appearance": round(looks_alike, 2),
            "geometry": round(stands, 2), "apart_m": round(apart, 2),
            "looks": [keep.count, gone.count],
            "shown": {keep.id: keep.shown, gone.id: gone.shown}})
    return {"ok": True, "map_session": session, "things": len(things),
            "density_per_m2": round(density, 2), "proposals": proposals,
            "seconds": round(time.time() - began, 2)}


def _refusal(store, keep: str, gone: str, session: int, used: set) -> str | None:
    """Why this pair cannot be joined now, or None."""
    if keep == gone:
        return "a thing cannot be joined to itself"
    if keep in used or gone in used:
        return "one of them is already in another pair of this run"
    for entity_id in (keep, gone):
        row = store.db.execute("SELECT placement_map_session FROM entities WHERE id = ?",
                               (entity_id,)).fetchone()
        if row is None:
            return f"{entity_id} is no longer there"
        if row["placement_map_session"] != session:
            return f"{entity_id} is not placed in this map session"
    shared = store.db.execute(
        "SELECT a.inference_id FROM observations a JOIN observations b"
        " ON a.inference_id = b.inference_id WHERE a.entity_id = ? AND b.entity_id = ?"
        " LIMIT 1", (keep, gone)).fetchone()
    if shared is not None:
        return f"both have a region in look {shared[0]}, so they are two things"
    return None


def apply(store, pairs, reach=None) -> dict[str, Any]:
    """Join these pairs, each `[keep, gone]`, journalling both rows and every moved
    look first. One transaction; then each kept thing's placement is worked out again
    from everything it now holds, as the resolver would after a look."""
    from . import resolve

    try:
        wanted = [(str(keep), str(gone)) for keep, gone in pairs]
    except (TypeError, ValueError):
        return {"ok": False, "error": "apply wants a list of [keep, gone] pairs"}
    if not wanted:
        return {"ok": False, "error": "no pairs given; nothing was changed"}
    now = time.time()
    session = store.map_session()
    refused, joined = [], []
    with store._lock, store.db:
        used: set = set()
        valid = []
        for keep, gone in wanted:
            why = _refusal(store, keep, gone, session, used)
            if why:
                refused.append({"keep": keep, "gone": gone, "why": why})
                continue
            used.update((keep, gone))
            valid.append((keep, gone))
        if not valid:
            return {"ok": False, "error": "none of the pairs can be joined; nothing was "
                                          "changed", "refused": refused}
        run_id = store.db.execute(
            "INSERT INTO merge_runs(applied_at, map_session, generation, pairs_json)"
            " VALUES(?,?,?,?)", (now, session, store.generation(), json.dumps(valid))
        ).lastrowid
        for keep, gone in valid:
            for entity_id in (keep, gone):
                row = dict(store.db.execute("SELECT * FROM entities WHERE id = ?",
                                            (entity_id,)).fetchone())
                scalars = {key: value for key, value in row.items() if key not in _BLOBS}
                store.db.execute(
                    "INSERT INTO merge_entities(run_id, entity_id, row_json, exemplars,"
                    " exemplars_alone) VALUES(?,?,?,?,?)",
                    (run_id, entity_id, json.dumps(scalars), row.get("exemplars"),
                     row.get("exemplars_alone")))
            moving = store.db.execute("SELECT id, note FROM observations WHERE entity_id = ?",
                                      (gone,)).fetchall()
            for look in moving:
                store.db.execute(
                    "INSERT INTO merge_looks(run_id, observation_id, entity_before,"
                    " note_before) VALUES(?,?,?,?)", (run_id, look["id"], gone, look["note"]))
            done = store._merge_rows(keep, gone)
            if not done.get("ok"):
                # `_refusal` asked the same questions a moment ago inside this lock, so
                # this is a fault rather than a refusal: undo the whole transaction.
                raise RuntimeError(f"joining {gone} to {keep} failed: {done.get('why')}")
            for look in moving:
                store.db.execute("UPDATE observations SET note = ? WHERE id = ?",
                                 (f"joined to {keep} from {gone} by merge {run_id}",
                                  look["id"]))
            joined.append({"keep": keep, "gone": gone, "looks_moved": len(moving)})
    for one in joined:
        resolve._replace_placement(store, one["keep"], session, reach)
    count = store.db.execute(
        "SELECT COUNT(*) AS n FROM entities WHERE placement_map_session = ?",
        (session,)).fetchone()["n"]
    return {"ok": True, "applied": True, "run": run_id, "joined": joined,
            "refused": refused, "things_now": count}


def runs(store) -> list[dict[str, Any]]:
    """Every merge run applied to this store, newest first."""
    with store._lock:
        rows = [dict(row) for row in store.db.execute(
            "SELECT id, applied_at, map_session, pairs_json, rolled_back_at"
            " FROM merge_runs ORDER BY id DESC")]
    for row in rows:
        row["pairs"] = json.loads(row.pop("pairs_json") or "[]")
    return rows


def rollback(store) -> dict[str, Any]:
    """Put back the last merge run applied, and leave alone what came after it."""
    now = time.time()
    with store._lock, store.db:
        run = store.db.execute(
            "SELECT * FROM merge_runs WHERE rolled_back_at IS NULL"
            " ORDER BY id DESC LIMIT 1").fetchone()
        if run is None:
            return {"ok": False, "error": "no merge run is applied; nothing to roll back"}
        run = dict(run)
        if store.generation() != run["generation"]:
            return {"ok": False, "error": f"the world state was cleared since merge "
                                          f"{run['id']}; there is nothing to put back"}
        if store.map_session() != run["map_session"]:
            return {"ok": False, "error": f"the map changed since merge {run['id']}; "
                                          f"nothing was changed"}
        columns = {row["name"] for row in store.db.execute("PRAGMA table_info(entities)")}
        restored = []
        for kept in store.db.execute("SELECT * FROM merge_entities WHERE run_id = ?",
                                     (run["id"],)).fetchall():
            row = {key: value for key, value in json.loads(kept["row_json"]).items()
                   if key in columns}
            for blob in _BLOBS:
                if blob in columns:
                    row[blob] = kept[blob]
            store.db.execute("DELETE FROM entities WHERE id = ?", (kept["entity_id"],))
            names = list(row)
            store.db.execute(
                f"INSERT INTO entities({', '.join(names)})"
                f" VALUES({', '.join('?' * len(names))})", [row[name] for name in names])
            restored.append(kept["entity_id"])
        moved = 0
        for look in store.db.execute("SELECT * FROM merge_looks WHERE run_id = ?",
                                     (run["id"],)).fetchall():
            moved += store.db.execute(
                "UPDATE observations SET entity_id = ?, note = ? WHERE id = ?",
                (look["entity_before"], look["note_before"], look["observation_id"])
            ).rowcount
        # Each restored row says how many looks it had then. A look recorded since may
        # have joined the kept one, and only there is the row counted again: anywhere
        # else it is its own row as it was, which is what makes the restore exact.
        for entity_id in restored:
            got = store.db.execute(
                "SELECT COUNT(*) AS n, MAX(observed_at) AS last FROM observations"
                " WHERE entity_id = ?", (entity_id,)).fetchone()
            store.db.execute(
                "UPDATE entities SET observation_count = ?,"
                " last_seen_at = MAX(last_seen_at, COALESCE(?, last_seen_at))"
                " WHERE id = ? AND observation_count != ?",
                (got["n"], got["last"], entity_id, got["n"]))
        store.db.execute("UPDATE merge_runs SET rolled_back_at = ? WHERE id = ?",
                         (now, run["id"]))
        count = store.db.execute(
            "SELECT COUNT(*) AS n FROM entities WHERE placement_map_session = ?",
            (run["map_session"],)).fetchone()["n"]
    return {"ok": True, "rolled_back": run["id"], "things_restored": len(restored),
            "looks_moved_back": moved, "things_now": count}
