"""Re-solve one map session's things all at once by EM, as an act that can be undone.

    world_state_consolidate                      what it would change, and nothing else
    world_state_consolidate {"apply": true}      change it, keeping what it changed
    world_state_consolidate {"rollback": true}   put back the last one applied

All three are a control call on the daemon (`rover_world._tool_world_state_consolidate`),
which holds the rover's own looks and its resolver off while this runs. Nothing here
does that itself.

**What it is for.** The resolver commits each look the moment it is offered and never
looks at two placed things together again, so a thing founded on a bad picture stays
split from the rest of itself: on 2026-10-03 the door was `object:96`, founded on a
picture with a chair in front of it, and `object:102`. This hands every look of the
current map session to [cluster.py](cluster.py)'s expectation-maximisation at once,
starting from the things the resolver holds, and lets the looks settle where they fit
best. It is the equal-shares, look-alike-merging variant that
[the replay of that session](../docs/progress/2026-10-03-whole-session-em.md) measured
least damaging, and **it was not good enough to adopt**: of 117 things it merged 20
into others, twelve of fifteen merges right, and dropped 27, while putting 7% of crops
in the wrong thing. It is here so that a person can look at what it does to the room
they are standing in, and undo it.

**Undoing it is exact for what it touched, and leaves alone what came after.** Applying
writes a journal first: every thing it changed or removed, row for row, and every look
it moved, with where that look was before. Rolling back puts those rows back and moves
those looks back. A look recorded since stays where the resolver put it, because its
thing still exists -- EM keeps the identifier of every thing it keeps, so nothing it
wrote is a name the old world did not have. A thing the resolver founded since, out of
looks EM had let go, is left with no looks by the rollback and goes. Only the last
consolidation applied can be rolled back, and not across a clear or a change of map.

What changes besides the looks, for the things it keeps: the position (a fit over every
look that claims the thing, ranges included), the appearance kept for it (the newest
crops among those looks) and the counts. Every moved look's note says it was moved here
and why, which is what the console's popup shows.
"""
from __future__ import annotations

import json
import math
import time
from collections import Counter
from typing import Any

from . import cluster, locate, resolve
from .store import EXEMPLARS, _readable

#: The variant applied: shares held equal, and a merge refused unless the two never
#: shared a look and their crops look alike in the median. Re-estimating the shares as
#: `cluster.py` does took 117 things to 24 on the session it was measured on.
SHARES = False
LOOKALIKE = True

#: What two things' crops have to score in the median to be merged when `LOOKALIKE`.
#: `replay.JOIN`'s figure, kept here so the daemon does not import the replay harness.
JOIN = 0.55

_ROOT_2PI = math.sqrt(2.0 * math.pi)

#: The entity columns held as bytes, which go into the journal as they are.
_BLOBS = ("exemplars", "exemplars_alone")


def _wrap(degrees):
    return (degrees + 180.0) % 360.0 - 180.0


class Session:
    """Every ray of one map session, held as arrays for the E-step.

    `rows` are observation rows as the store holds them, vectors included. Rows
    without a usable ray are carried but never claimed.
    """

    def __init__(self, rows, reach=None):
        import numpy as np

        self.rows = rows
        self.rays = []
        for row in rows:
            ray = resolve.ray_of(_readable(dict(row), vectors=True), reach)
            if ray is not None:
                self.rays.append((ray, row))
        rays = [ray for ray, _row in self.rays]
        self.index = {ray["observation_id"]: i for i, ray in enumerate(rays)}

        def column(key, missing=np.nan):
            return np.array([missing if ray.get(key) is None else float(ray[key])
                             for ray in rays])

        self.x, self.y = column("x_m"), column("y_m")
        self.bearing = column("bearing_deg")
        self.sigma = np.array([locate.sigma_of(ray) for ray in rays])
        self.origin = column("origin_sigma_m", 0.0)
        self.range = column("range_m")
        self.ranged = ~np.isnan(self.range)
        stated = column("range_sigma_m", 0.0)
        self.range_sigma = np.where(stated > 0.0, stated, locate.RANGE_SIGMA_M)
        self.reach = column("reach_m", np.inf) + locate.SEE_PAST_M
        self.look = [ray.get("inference_id") for ray in rays]
        looks: dict = {}
        for index, key in enumerate(self.look):
            looks.setdefault(key, []).append(index)
        self.looks = list(looks.values())
        nothing = cluster.CLUTTER_PER_DEG * cluster.CLUTTER_PRIOR
        self.clutter = np.where(self.ranged, nothing * cluster.CLUTTER_PER_M, nothing)

        vectors = [np.frombuffer(row.get("dino_blob") or b"", dtype="<f4")
                   for _ray, row in self.rays]
        width = max((len(v) for v in vectors), default=0)
        self.has_vector = np.array([len(v) == width and width > 0 for v in vectors],
                                   dtype=bool)
        unit = np.zeros((len(vectors), max(width, 1)))
        for index, vector in enumerate(vectors):
            if self.has_vector[index]:
                unit[index] = vector / max(float(np.linalg.norm(vector)), 1e-9)
        # Not a number across two perception backends, whose vectors are not
        # comparable (R-WS-4): silence rather than a score, as everywhere else.
        backend = np.array([str(row.get("vectors_from") or "") for _ray, row in self.rays])
        self.similar = np.where(backend[:, None] == backend[None, :],
                                unit @ unit.T, np.nan)

    def ray(self, index):
        return self.rays[index][0]

    def row(self, index):
        return self.rays[index][1]

    def scores(self, places):
        """`cluster._likelihood` times share, every ray against every place.

        Three per-ray refusals that cluster.py makes of a whole thing at the end are
        made here of the ray: past the wall in front of it, nearer than
        `locate.MIN_RANGE_M`, further than `locate.MAX_RANGE_M`. Asked of the thing, a
        thing seen 81 times from 15 places nearly always has one look from 0.7 m, and
        that one look dropped it.
        """
        import numpy as np

        px = np.array([p["x_m"] for p in places])
        py = np.array([p["y_m"] for p in places])
        extent = np.array([max(0.0, p.get("extent_m") or 0.0) for p in places])
        share = np.array([p.get("share") or 1.0 for p in places])
        dx = px[None, :] - self.x[:, None]
        dy = py[None, :] - self.y[:, None]
        distance = np.hypot(dx, dy)
        near = np.maximum(distance, locate.MIN_RANGE_M)
        noise = np.hypot(self.sigma[:, None],
                         np.degrees(np.arctan2(self.origin[:, None], near)))
        half = np.degrees(np.arctan2(extent[None, :] / 2.0, near))
        off = np.abs(_wrap(np.degrees(np.arctan2(dy, dx)) - self.bearing[:, None]))
        missed = np.maximum(0.0, off - half)
        got = np.exp(-0.5 * (missed / noise) ** 2) / (noise * _ROOT_2PI)
        short = np.maximum(0.0, np.abs(distance - np.nan_to_num(self.range)[:, None])
                           - extent[None, :] / 2.0)
        sigma_m = self.range_sigma[:, None]
        along = np.exp(-0.5 * (short / sigma_m) ** 2) / (sigma_m * _ROOT_2PI)
        got = np.where(self.ranged[:, None], got * along, got) * share[None, :]
        got[(distance > self.reach[:, None]) | (distance < locate.MIN_RANGE_M)
            | (distance > locate.MAX_RANGE_M)] = 0.0
        # The appearance veto as cluster.py asks it: refused only where the ray looks
        # like none of the thing's known crops.
        for column, place in enumerate(places):
            known = [i for i in place["exemplars"] if self.has_vector[i]]
            if known:
                block = self.similar[:, known]
                said = ~np.isnan(block).all(axis=1)
                best = np.where(np.isnan(block), -np.inf, block).max(axis=1)
                got[self.has_vector & said & (best < resolve.DIFFERENT_THING),
                    column] = 0.0
        return got

    def weigh(self, places):
        """The E-step: each look's single best arrangement, as 0/1 weights.

        A thing may take one region of a picture, so a look is shared out as an
        assignment, as `resolve._by_look` does; a region stays nothing wherever no
        thing beats the hypothesis that it is scenery.
        """
        import numpy as np
        from scipy.optimize import linear_sum_assignment

        got = self.scores(places)
        weights = np.zeros_like(got)
        for members in self.looks:
            mine, nothing = got[members], self.clutter[members][:, None]
            gain = np.where(mine > nothing,
                            np.log(np.maximum(mine, 1e-300)) - np.log(nothing),
                            -np.inf)
            columns = np.where(np.isfinite(gain).any(axis=0))[0]
            if not len(columns):
                continue
            gain = gain[:, columns]
            top = float(gain[np.isfinite(gain)].max()) + 1.0
            cost = np.where(np.isfinite(gain), top - gain, top)
            for row, picked in zip(*linear_sum_assignment(cost)):
                if cost[row, picked] < top:
                    weights[members[row], columns[picked]] = 1.0
        return weights


def _share(places, weights):
    """`cluster._share`, on a weight matrix."""
    totals = weights.sum(axis=0)
    average = float(totals.mean()) if len(totals) else 0.0
    for place, total in zip(places, totals):
        place["share"] = ((total + cluster.SHARE_PRIOR) / (average + cluster.SHARE_PRIOR)
                          if average > 0.0 else 1.0)


def _stands(session, place, claimed):
    """`cluster._survives`, less the per-ray checks `Session.scores` now makes.

    A thing seen from one place stands if a look claiming it measured its range,
    which is what `resolve._place_from_range` already accepts.
    """
    claiming = [session.ray(i) for i in claimed]
    if not claiming:
        return False
    if locate.standing_places(claiming) < 2:
        if not any(ray.get("range_m") is not None for ray in claiming):
            return False
    elif len(claiming) < cluster.MIN_CLAIMING:
        return False
    return place.get("error_major_m", 0.0) <= cluster.MAX_UNCERTAINTY_M


def _merge(session, places, lookalike):
    """`cluster._merge`, or the same refusing two things that do not look alike."""
    import numpy as np

    kept = []
    for place in sorted(places, key=lambda one: one["error_major_m"]):
        twin = None
        for other in kept:
            if math.hypot(place["x_m"] - other["x_m"],
                          place["y_m"] - other["y_m"]) >= cluster.SAME_PLACE_M:
                continue
            if lookalike:
                if ({session.look[i] for i in place["claimers"]}
                        & {session.look[i] for i in other["claimers"]}):
                    continue
                mine = [i for i in place["claimers"] if session.has_vector[i]]
                theirs = [i for i in other["claimers"] if session.has_vector[i]]
                block = session.similar[np.ix_(mine, theirs)] if mine and theirs else None
                if (block is not None and not np.isnan(block).all()
                        and float(np.nanmedian(block)) < JOIN):
                    continue
            twin = other
            break
        if twin is None:
            kept.append(place)
        else:
            twin["exemplars"] = sorted(set(twin["exemplars"]) | set(place["exemplars"]))
            twin["absorbed"] = (twin.get("absorbed", []) + [place["id"]]
                                + place.get("absorbed", []))
    return kept


def consolidate(session, seeds, *, shares=SHARES, lookalike=LOOKALIKE):
    """cluster.discover's loop, from the resolver's things, over every ray.

    `seeds` are the things to start from: an `id`, a position, an `extent_m`, an
    `error_major_m`, and `exemplars`, the indices of the rays they hold. Answers the
    things left standing, the final claims as a 0/1 matrix of rays by things, and how
    many rounds it ran.
    """
    import numpy as np

    places = [dict(seed) for seed in seeds]
    rounds = 0
    for rounds in range(1, cluster.MAX_ROUNDS + 1):
        weights = session.weigh(places)
        if shares:
            _share(places, weights)
            places = [p for p in places if p["share"] >= cluster.MIN_SHARE]
        weights = session.weigh(places)
        moved, fitted = 0.0, []
        for column, place in enumerate(places):
            claimed = np.where(weights[:, column] > 0)[0]
            if not len(claimed):
                continue
            rays = [session.ray(i) for i in claimed]
            extent = cluster._extent(place, rays, [1.0] * len(rays))
            got = locate.fit_over(rays, [1.0] * len(rays),
                                  (place["x_m"], place["y_m"]), extent)
            if got is None:
                continue
            moved = max(moved, math.hypot(got["x_m"] - place["x_m"],
                                          got["y_m"] - place["y_m"]))
            got.update(id=place["id"], extent_m=extent, share=place.get("share", 1.0),
                       exemplars=sorted(set(place["exemplars"]) | set(claimed.tolist())),
                       claimers=claimed.tolist(), absorbed=place.get("absorbed", []))
            fitted.append(got)
        places = _merge(session, fitted, lookalike)
        if moved < cluster.SETTLED_M:
            break
    # Backward selection, as cluster.discover does it.
    while places:
        weights = session.weigh(places)
        if shares:
            _share(places, weights)
        failing = [(place.get("share", 1.0), column) for column, place in enumerate(places)
                   if not _stands(session, place, np.where(weights[:, column] > 0)[0])]
        if not failing:
            break
        places.pop(min(failing)[1])
    return places, session.weigh(places), rounds


# --- against the store ---------------------------------------------------------

class Plan:
    """What a consolidation would do to the store, worked out and not yet written."""

    def __init__(self, map_session, generation, entities, owner, kept, seconds, rounds,
                 rows):
        self.map_session = map_session
        self.generation = generation
        #: The things it started from, as the store holds them: id -> row.
        self.entities = entities
        #: Where each of the session's looks ends: observation id -> thing or None.
        self.owner = owner
        #: The things left standing: id -> (placement, claimer rows).
        self.kept = kept
        self.seconds = seconds
        self.rounds = rounds
        #: Every observation row the session holds, by id.
        self.rows = rows

    def moves(self) -> dict[int, tuple[Any, Any]]:
        """observation id -> (thing before, thing after), for every look that moves."""
        moved = {}
        for observation_id, after in self.owner.items():
            before = self.rows[observation_id]["entity_id"]
            if before != after:
                moved[observation_id] = (before, after)
        return moved

    def fates(self) -> dict[str, dict[str, Any]]:
        """For every thing that goes, where its looks went.

        A thing goes two ways and the fit does not tell them apart: merged by
        position, or emptied because every one of its looks fitted somewhere else
        better. Read from the looks, both are one question -- did most of them go to
        one thing that stays? -- and the answer is what a person reviewing it needs.
        """
        went: dict[str, Counter] = {}
        for observation_id, row in self.rows.items():
            if row["entity_id"] in self.entities and row["entity_id"] not in self.kept:
                went.setdefault(row["entity_id"], Counter())[
                    self.owner.get(observation_id)] += 1
        fates = {}
        for entity_id in self.entities:
            if entity_id in self.kept:
                continue
            counts = went.get(entity_id, Counter())
            looks = sum(counts.values())
            into, most = next(((one, n) for one, n in counts.most_common()
                               if one is not None), (None, 0))
            fates[entity_id] = {"looks": looks,
                                "into": into if most * 2 > looks else None,
                                "looks_into": most,
                                "let_go": counts.get(None, 0)}
        return fates

    def summary(self) -> dict[str, Any]:
        moves = self.moves()
        fates = self.fates()
        merged: dict[str, list] = {}
        for entity_id, fate in fates.items():
            if fate["into"] is not None:
                merged.setdefault(fate["into"], []).append(entity_id)
        dropped = sorted((one for one, fate in fates.items() if fate["into"] is None),
                         key=lambda one: -fates[one]["looks"])
        return {
            "map_session": self.map_session,
            "things_before": len(self.entities),
            "things_after": len(self.kept),
            "merges": [{"kept": keeper, "absorbed": sorted(absorbed)}
                       for keeper, absorbed in sorted(merged.items())],
            "dropped": [{"id": one, "looks": fates[one]["looks"],
                         "looks_kept_elsewhere": fates[one]["looks_into"]}
                        for one in dropped],
            "looks_moved": sum(1 for before, after in moves.values()
                               if before is not None and after is not None),
            "looks_let_go": sum(1 for before, after in moves.values() if after is None),
            "looks_taken_up": sum(1 for before, after in moves.values()
                                  if before is None),
            "rounds": self.rounds,
            "seconds": round(self.seconds, 1),
        }


def _placement(session, place, claimed):
    """The position in the shape the store and the console already read."""
    rays = [session.ray(i) for i in claimed]
    out = cluster._placement(place, rays, [1.0] * len(rays))
    out.pop("members", None)
    out.update(locate.height_fields(out, rays))
    return out


def _names(session, claims, known) -> list[str]:
    """Which existing name each fitted thing should carry: the one most of its looks
    had before.

    **The fit carries a seed's name wherever the seed drifts, and that is not the
    same thing.** On the session of 2026-10-03 two things swapped every one of their
    24 looks, so the name the console showed for one was standing on the other's
    pictures. Naming each result after where its looks came from keeps a name on the
    object it meant, and an assignment keeps the names distinct.
    """
    import numpy as np
    from scipy.optimize import linear_sum_assignment

    column = {name: index for index, name in enumerate(known)}
    overlap = np.zeros((len(claims), len(known)))
    for row, claimed in enumerate(claims):
        for index in claimed:
            before = session.row(index)["entity_id"]
            if before in column:
                overlap[row, column[before]] += 1.0
    rows, picked = linear_sum_assignment(-overlap)
    names = [""] * len(claims)
    for row, choice in zip(rows, picked):
        names[row] = known[choice]
    return names


def plan(store, reach=None, *, shares=SHARES, lookalike=LOOKALIKE) -> Plan:
    """Work out what consolidating the current map session would do. Writes nothing."""
    import numpy as np

    began = time.time()
    session_id = store.map_session()
    with store._lock:
        rows = [dict(row) for row in store.db.execute(
            "SELECT * FROM observations WHERE map_session = ? AND bearing_deg IS NOT NULL"
            " ORDER BY observed_at, id", (session_id,))]
        # In a fixed order, because the fit's ties are broken by order: unordered, the
        # answer changed after a rollback, which re-inserts the rows.
        entities = {row["id"]: dict(row) for row in store.db.execute(
            "SELECT * FROM entities WHERE placement_json IS NOT NULL"
            " AND placement_map_session = ? ORDER BY created_at, id", (session_id,))}
        generation = store.generation()
    session = Session(rows, reach)
    seeds = []
    for entity_id, entity in entities.items():
        try:
            where = json.loads(entity["placement_json"])
        except (TypeError, ValueError):
            continue
        members = [session.index[row["id"]] for row in rows
                   if row["entity_id"] == entity_id and row["id"] in session.index]
        seeds.append({"id": entity_id, "x_m": float(where["x_m"]),
                      "y_m": float(where["y_m"]),
                      "extent_m": where.get("extent_m") or 0.0,
                      "error_major_m": where.get("error_major_m",
                                                 where.get("uncertainty_m") or 0.0),
                      "exemplars": members})
    places, weights, rounds = consolidate(session, seeds, shares=shares,
                                          lookalike=lookalike)
    standing = [(place, np.where(weights[:, column] > 0)[0])
                for column, place in enumerate(places)]
    standing = [(place, claimed) for place, claimed in standing if len(claimed)]
    names = _names(session, [claimed for _place, claimed in standing], list(entities))
    owner = {row["id"]: None for _ray, row in session.rays}
    kept = {}
    for (place, claimed), name in zip(standing, names):
        for index in claimed:
            owner[session.ray(index)["observation_id"]] = name
        kept[name] = (_placement(session, place, claimed),
                      [session.row(i) for i in claimed])
    # A ray-less look keeps whatever it had, unless its thing is going.
    for row in rows:
        if row["id"] not in owner and row["entity_id"] is not None:
            owner[row["id"]] = row["entity_id"] if row["entity_id"] in kept else None
    return Plan(session_id, generation, entities, owner, kept, time.time() - began,
                rounds, {row["id"]: row for row in rows})


def _why(run_id, after, before):
    if after is None:
        return (f"let go by consolidation {run_id}: fitted with every look of the "
                f"session, no thing explains it better than scenery")
    if before is None:
        return (f"taken up by consolidation {run_id}: fitted with every look of the "
                f"session, it is part of {after}")
    return (f"moved from {before} by consolidation {run_id}: fitted with every look "
            f"of the session, it is part of {after}")


def _recount(store, entity_ids) -> None:
    """Each thing's count and last sighting, from what is attached to it now."""
    for entity_id in entity_ids:
        row = store.db.execute(
            "SELECT COUNT(*) AS n, MAX(observed_at) AS last FROM observations"
            " WHERE entity_id = ?", (entity_id,)).fetchone()
        store.db.execute(
            "UPDATE entities SET observation_count = ?,"
            " last_seen_at = COALESCE(?, last_seen_at) WHERE id = ?",
            (row["n"], row["last"], entity_id))


def apply(store, worked: Plan) -> dict[str, Any]:
    """Write a plan, journalling everything it changes first. One transaction.

    Refused if the store has moved under the plan: another map session, another
    generation, or a thing it started from that is no longer there.
    """
    now = time.time()
    summary = worked.summary()
    with store._lock, store.db:
        if (store.map_session() != worked.map_session
                or store.generation() != worked.generation):
            return {"ok": False, "error": "the world state changed map or was cleared "
                                          "since this was worked out; nothing was changed"}
        current = {row["id"]: dict(row) for row in store.db.execute(
            "SELECT * FROM entities WHERE placement_json IS NOT NULL"
            " AND placement_map_session = ?", (worked.map_session,))}
        if set(current) != set(worked.entities):
            return {"ok": False, "error": "the things changed since this was worked out; "
                                          "nothing was changed"}
        run_id = store.db.execute(
            "INSERT INTO consolidations(applied_at, map_session, generation, summary_json)"
            " VALUES(?,?,?,?)", (now, worked.map_session, worked.generation,
                                 json.dumps(summary))).lastrowid
        # Every thing it started from, as it stood. All of them, not only the ones
        # that change, so that the rollback is a restore rather than a calculation.
        for entity_id, row in current.items():
            scalars = {key: value for key, value in row.items() if key not in _BLOBS}
            store.db.execute(
                "INSERT INTO consolidation_entities(run_id, entity_id, row_json,"
                " exemplars, exemplars_alone) VALUES(?,?,?,?,?)",
                (run_id, entity_id, json.dumps(scalars), row.get("exemplars"),
                 row.get("exemplars_alone")))
        gone = [one for one in current if one not in worked.kept]
        moves = dict(worked.moves())
        # Looks of a thing that is going which are not in the session's rays -- taken
        # under another map and adopted, or without a bearing -- go to nobody.
        if gone:
            marks = ",".join("?" * len(gone))
            for row in store.db.execute(
                    f"SELECT id, entity_id FROM observations WHERE entity_id IN ({marks})",
                    gone):
                if row["id"] not in moves and row["id"] not in worked.owner:
                    moves[row["id"]] = (row["entity_id"], None)
        for observation_id, (before, after) in moves.items():
            note = store.db.execute("SELECT note FROM observations WHERE id = ?",
                                    (observation_id,)).fetchone()
            store.db.execute(
                "INSERT INTO consolidation_looks(run_id, observation_id, entity_before,"
                " note_before, entity_after) VALUES(?,?,?,?,?)",
                (run_id, observation_id, before, None if note is None else note["note"],
                 after))
            store.db.execute("UPDATE observations SET entity_id = ?, note = ? WHERE id = ?",
                             (after, _why(run_id, after, before), observation_id))
        for entity_id in gone:
            store.db.execute("DELETE FROM entities WHERE id = ?", (entity_id,))
        into = {}
        for gone_id, fate in worked.fates().items():
            if fate["into"] is not None:
                into.setdefault(fate["into"], []).append(gone_id)
        for entity_id, (placement, claimers) in worked.kept.items():
            absorbed = into.get(entity_id, [])
            placement = {**placement, "consolidation": run_id,
                         "stated_uncertainty_m": locate.stated_uncertainty(placement)}
            # The newest crops of the newest look's backend, so that what is kept can
            # be compared with what comes next (R-WS-4).
            newest = sorted(claimers, key=lambda row: (row["observed_at"], row["id"]))
            backend = newest[-1].get("vectors_from") if newest else None
            newest = [row for row in newest if row.get("vectors_from") == backend]
            plain = b"".join(row["dino_blob"] for row in newest if row.get("dino_blob"))
            alone = b"".join(row["dino_alone_blob"] for row in newest
                             if row.get("dino_alone_blob"))
            width = len(next((row["dino_blob"] for row in newest
                              if row.get("dino_blob")), b"")) or 1
            alone_width = len(next((row["dino_alone_blob"] for row in newest
                                    if row.get("dino_alone_blob")), b"")) or 1
            born = min([current[entity_id]["created_at"]]
                       + [current[one]["created_at"] for one in absorbed if one in current])
            store.db.execute(
                "UPDATE entities SET placement_json = ?, placement_uncertainty_m = ?,"
                " placement_map_session = ?, placement_updated_at = ?, created_at = ?,"
                " exemplars = ?, exemplars_alone = ? WHERE id = ?",
                (json.dumps(placement), placement.get("uncertainty_m"),
                 worked.map_session, now, born,
                 plain[-width * EXEMPLARS:] or current[entity_id].get("exemplars"),
                 alone[-alone_width * EXEMPLARS:] or current[entity_id].get("exemplars_alone"),
                 entity_id))
        _recount(store, worked.kept)
    return {"ok": True, "applied": True, "run": run_id, **summary}


def runs(store) -> list[dict[str, Any]]:
    """Every consolidation applied to this store, newest first."""
    with store._lock:
        rows = [dict(row) for row in store.db.execute(
            "SELECT id, applied_at, map_session, rolled_back_at, summary_json"
            " FROM consolidations ORDER BY id DESC")]
    for row in rows:
        summary = json.loads(row.pop("summary_json") or "{}")
        row.update(things_before=summary.get("things_before"),
                   things_after=summary.get("things_after"))
    return rows


def rollback(store) -> dict[str, Any]:
    """Put back the last consolidation applied, and leave alone what came after it."""
    now = time.time()
    with store._lock, store.db:
        run = store.db.execute(
            "SELECT * FROM consolidations WHERE rolled_back_at IS NULL"
            " ORDER BY id DESC LIMIT 1").fetchone()
        if run is None:
            return {"ok": False, "error": "no consolidation is applied; nothing to roll back"}
        run = dict(run)
        if store.generation() != run["generation"]:
            return {"ok": False, "error": f"the world state was cleared since consolidation "
                                          f"{run['id']}; there is nothing to put back"}
        if store.map_session() != run["map_session"]:
            return {"ok": False, "error": f"the map changed since consolidation "
                                          f"{run['id']}; nothing was changed"}
        snapshot = [dict(row) for row in store.db.execute(
            "SELECT * FROM consolidation_entities WHERE run_id = ?", (run["id"],))]
        columns = {row["name"] for row in store.db.execute("PRAGMA table_info(entities)")}
        for kept in snapshot:
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
        moved = 0
        for look in store.db.execute(
                "SELECT * FROM consolidation_looks WHERE run_id = ?", (run["id"],)).fetchall():
            moved += store.db.execute(
                "UPDATE observations SET entity_id = ?, note = ? WHERE id = ?",
                (look["entity_before"], look["note_before"], look["observation_id"])
            ).rowcount
        # Things the resolver founded since out of looks this run had let go, which the
        # lines above have just taken back: nothing is left in them.
        restored = {kept["entity_id"] for kept in snapshot}
        emptied = [row["id"] for row in store.db.execute(
            "SELECT id FROM entities WHERE created_at >= ? AND NOT EXISTS"
            " (SELECT 1 FROM observations WHERE observations.entity_id = entities.id)",
            (run["applied_at"],)) if row["id"] not in restored]
        for entity_id in emptied:
            store.db.execute("DELETE FROM entities WHERE id = ?", (entity_id,))
        # A look recorded since whose thing is not there now waits again.
        store.db.execute(
            "UPDATE observations SET entity_id = NULL WHERE entity_id IS NOT NULL"
            " AND observed_at >= ? AND entity_id NOT IN (SELECT id FROM entities)",
            (run["applied_at"],))
        # Counted again only where something arrived since: every other thing is
        # its own row as it was, which already says how many looks it had.
        _recount(store, [row["id"] for row in store.db.execute(
            "SELECT DISTINCT entity_id AS id FROM observations"
            " WHERE entity_id IS NOT NULL AND observed_at >= ?", (run["applied_at"],))])
        store.db.execute("UPDATE consolidations SET rolled_back_at = ? WHERE id = ?",
                         (now, run["id"]))
        count = store.db.execute(
            "SELECT COUNT(*) AS n FROM entities WHERE placement_json IS NOT NULL"
            " AND placement_map_session = ?", (run["map_session"],)).fetchone()["n"]
    return {"ok": True, "rolled_back": run["id"], "things_restored": len(snapshot),
            "looks_moved_back": moved, "founded_since_and_emptied": emptied,
            "things_now": count}

