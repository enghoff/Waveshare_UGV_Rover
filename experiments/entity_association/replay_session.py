"""A whole map session replayed through today's resolver, keeping observation ids.

`world_state/replay.py` renumbers observations as it inserts them, which cuts
them loose from any label written against the rover's store. This inserts each
recorded look with its own identifiers, in the order the rover stored them, and
calls the resolver after each look as the inspector does. A founding -- a new
thing placed from two crossing bearings or from one measured range -- is logged
with every thing already placed within reach of it, so the cause of a duplicate
can be read off: whether a look-alike thing already stood there, and which gate
kept the founding looks from joining it.

    python experiments/entity_association/replay_session.py --database <store.db>
        --session 67 --grid <map.json> --output <new-directory>
        [--limit-looks N] [--variant name]

It reproduces the rover's memberships only approximately: the resolver has
changed during the session, the map is one archived grid rather than the map as
it grew, and passes ran at different moments. It is a bench for comparing
resolver variants under identical conditions, not a reproduction of the drive.
`--variant` names a change registered in `VARIANTS` below; `control` runs the
resolver as committed.
"""
from __future__ import annotations

import argparse
import functools
import json
import math
import sqlite3
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from world_state import appearance, locate, replay, resolve  # noqa: E402
from world_state.store import WorldStore  # noqa: E402

#: Resolver changes this bench can run, by name. Each is a context manager
#: factory: it patches `resolve` on entry and puts it back on exit.
VARIANTS: dict = {}


def variant(name):
    def register(factory):
        VARIANTS[name] = factory
        return factory
    return register


class _Nothing:
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


@variant("control")
def _control():
    return _Nothing()


class fast_appearance:
    """The resolver's cosine similarity done in numpy, for this bench only.

    The same arithmetic -- float32 vectors unpacked, scaled to unit length in
    double precision, one dot product -- with the summation order changed, so a
    score can differ in the sixteenth decimal place. Pure Python spent half of a
    session replay's time here (94 million multiply-adds in 150 looks). Every arm
    of a comparison runs with it, so comparisons stay like for like; it is never
    what the rover runs.
    """

    def __enter__(self):
        import numpy as np
        units: dict = {}

        def unit(blob):
            got = units.get(blob)
            if got is None:
                values = np.frombuffer(blob, dtype="<f4").astype(np.float64)
                length = float(np.sqrt(values @ values)) if len(values) else 0.0
                got = values / length if length > 1e-9 else False
                units[blob] = got
            return got

        def similarity(left, right):
            if not left or not right or len(left) != len(right):
                return 0.0
            a, b = unit(left), unit(right)
            if a is False or b is False:
                return 0.0
            return float(a @ b)

        self.saved = (appearance.similarity, resolve.similarity)
        appearance.similarity = resolve.similarity = similarity
        return self

    def __exit__(self, *exc):
        appearance.similarity, resolve.similarity = self.saved
        return False


#: A look's heading correction, by look, found by `_look_alignment`.
LOOK_SHIFT: dict = {}


@variant("look_alignment")
def _look_alignment():
    """Correct a whole look's bearings by how far it misses things already known.

    Measured on the labelled looks of map session 67 (2026-10-07): about 70% of
    the bearing error is shared by every region of one picture -- regions of two
    different objects in one look miss in the same direction, correlation 0.49 --
    which is what a heading that is off at the moment of the picture does. Where
    a look holds regions that match well-established things, those things say
    how far off it is, and every region of the look is turned back by that much
    before it is matched, paired or used to found anything.

    Thresholds fixed before any replay of this variant was scored:
    an anchor is a thing seen from three or more places and placed to within
    0.35 m; a region matches it when it looks like it (0.70, `RECOGNISED`), agrees
    on height and points within 12 degrees; a look is corrected only when two or
    more regions match different anchors and agree on the shift to within 2.5
    degrees of their median.
    """
    MAX_SHIFT_DEG, AGREE_DEG = 12.0, 2.5
    original_ray, original_by_look = resolve.ray_of, resolve._by_look

    def shifted_ray(observation, reach=None):
        shift = LOOK_SHIFT.get(observation.get("inference_id"))
        if shift and observation.get("bearing_deg") is not None:
            observation = dict(observation)
            observation["bearing_deg"] = float(observation["bearing_deg"]) - shift
        return original_ray(observation, reach)

    def by_look(store, group, entities, session, taken_in, reach=None):
        look = group[0].get("inference_id")
        if look is not None and look not in LOOK_SHIFT:
            anchors = [e for e in entities
                       if (e.get("placement") or {}).get("viewpoints", 0) >= 3
                       and (e.get("placement") or {}).get("uncertainty_m", 9) <= 0.35]
            found = []
            for observation in group:
                ray = original_ray(observation, None)
                vector = observation.get("dino_blob") or b""
                if ray is None or not vector:
                    continue
                best = None
                for entity in anchors:
                    p = entity["placement"]
                    to = math.degrees(math.atan2(p["y_m"] - ray["y_m"], p["x_m"] - ray["x_m"]))
                    miss = (ray["bearing_deg"] - to + 180.0) % 360.0 - 180.0
                    if abs(miss) > MAX_SHIFT_DEG or not locate.stands_as_high(p, ray):
                        continue
                    seen = resolve.appearance(store, entity["id"], vector)
                    if seen is None or seen < resolve.RECOGNISED:
                        continue
                    if best is None or seen > best[0]:
                        best = (seen, entity["id"], miss)
                if best:
                    found.append(best)
            # One region per anchor: the best-looking claim on it.
            per_anchor = {}
            for seen, entity_id, miss in found:
                if entity_id not in per_anchor or seen > per_anchor[entity_id][0]:
                    per_anchor[entity_id] = (seen, miss)
            misses = sorted(m for _, m in per_anchor.values())
            if len(misses) >= 2:
                middle = misses[len(misses) // 2] if len(misses) % 2 else                     (misses[len(misses) // 2 - 1] + misses[len(misses) // 2]) / 2.0
                agreeing = [m for m in misses if abs(m - middle) <= AGREE_DEG]
                if len(agreeing) >= 2:
                    LOOK_SHIFT[look] = sum(agreeing) / len(agreeing)
        return original_by_look(store, group, entities, session, taken_in, reach)

    class Patch:
        def __enter__(self):
            LOOK_SHIFT.clear()
            resolve.ray_of, resolve._by_look = shifted_ray, by_look
            return self

        def __exit__(self, *exc):
            resolve.ray_of, resolve._by_look = original_ray, original_by_look
            return False
    return Patch()


#: The largest lidar blob, in metres across in both directions, that the camera
#: is taken to see past: legs of tables and chairs, lamp poles, a person's legs.
#: Fixed on 2026-10-07 before the variant was scored, from the dining area's map,
#: where every leg is one to three 5 cm cells and every wall is metres long.
LEG_SIZE_M = 0.30


def reach_past_legs(path):
    """`replay.reach_from`, except that small separate blobs do not stop the view.

    The lidar sees about 20 cm off the floor, so the dining table and its chairs
    are a ring of legs on the map, and the first leg along a bearing used to end
    the camera's view there -- 2.3 m short of the painting on the wall behind
    them, which the depth camera measured (2026-10-07). Walls, cabinets and
    sofas are larger than `LEG_SIZE_M` and still stop it, so a bearing still
    cannot see into the next room.
    """
    import base64
    import zlib
    import numpy as np
    from scipy import ndimage
    payload = json.load(open(path))
    width, height = int(payload["width"]), int(payload["height"])
    resolution = float(payload["resolution_m"])
    origin_x, origin_y = float(payload["origin_x_m"]), float(payload["origin_y_m"])
    cells = np.frombuffer(zlib.decompress(base64.b64decode(payload["data"])),
                          dtype=np.int8).reshape(height, width)
    occupied = cells >= 50
    labels, _count = ndimage.label(occupied, structure=np.ones((3, 3)))
    blocking = np.zeros_like(occupied)
    limit = int(round(LEG_SIZE_M / resolution))
    for index, box in enumerate(ndimage.find_objects(labels), start=1):
        rows, cols = box
        if (rows.stop - rows.start) > limit or (cols.stop - cols.start) > limit:
            blocking[box] |= labels[box] == index
    step = resolution / 2.0

    def reach(x_m, y_m, bearing_deg):
        dx = math.cos(math.radians(bearing_deg)) * step
        dy = math.sin(math.radians(bearing_deg)) * step
        got = 0.0
        for n in range(1, int(12.0 / step) + 1):
            ix = math.floor((x_m + dx * n - origin_x) / resolution)
            iy = math.floor((y_m + dy * n - origin_y) / resolution)
            if not (0 <= ix < width and 0 <= iy < height):
                break
            if blocking[iy, ix]:
                break
            got = step * n
        return got
    return reach


@variant("legs_transparent")
def _legs_transparent():
    patch = _Nothing()
    patch.reach_from = reach_past_legs
    return patch


def looks_in_order(database: Path, session: int) -> list[list[dict]]:
    """The session's observations grouped by the look that took them, ordered as
    the rover stored them (by identifier: 37 regions carry invalid clocks)."""
    con = sqlite3.connect(database.resolve().as_uri() + "?mode=ro", uri=True)
    con.row_factory = sqlite3.Row
    rows = [dict(r) for r in con.execute(
        "SELECT * FROM observations WHERE map_session = ? ORDER BY id", (session,))]
    con.close()
    groups: dict = {}
    for row in rows:
        groups.setdefault(row["inference_id"], []).append(row)
    return sorted(groups.values(), key=lambda g: g[0]["id"])


def cached_reach(reach):
    """`reach` answered once per position and bearing to the centimetre and tenth
    of a degree: the same ray is asked for on every pass it stays pending."""
    if reach is None:
        return None
    memo: dict = {}

    def answer(x_m, y_m, bearing_deg):
        key = (round(x_m, 2), round(y_m, 2), round(bearing_deg, 1))
        if key not in memo:
            memo[key] = reach(x_m, y_m, bearing_deg)
        return memo[key]
    return answer


def founding_log(log, reach, history=None):
    """Wrap the two founding paths to log each new thing and its neighbours."""
    originals = {"_place_one": resolve._place_one,
                 "_place_from_range": resolve._place_from_range}

    def wrap(name):
        function = originals[name]

        @functools.wraps(function)
        def logged(store, available, session, entities, reach_=None):
            before = [dict(e) for e in entities]
            placed = function(store, available, session, entities, reach_)
            if placed is None:
                return placed
            decision, taken = placed
            row = store.db.execute("SELECT placement_json FROM entities WHERE id = ?",
                                   (decision.entity_id,)).fetchone()
            here = json.loads(row["placement_json"]) if row else None
            looks = {o["id"]: o for o in available if o["id"] in taken}
            near = []
            for other in before:
                p = other.get("placement") or {}
                if here is None or "x_m" not in p:
                    continue
                apart = math.hypot(p["x_m"] - here["x_m"], p["y_m"] - here["y_m"])
                if apart > (p.get("uncertainty_m") or 0.0) + here["uncertainty_m"]:
                    continue
                gates = []
                for look in looks.values():
                    ray = resolve.ray_of(look, reach_)
                    if ray is None:
                        gates.append("no ray")
                    elif locate.beyond_reach(ray, (p["x_m"], p["y_m"])):
                        gates.append("reach")
                    elif resolve._allowance_used(p, ray) is None:
                        gates.append("geometry")
                    else:
                        seen = resolve.appearance(store, other["id"],
                                                  look.get("dino_blob") or b"")
                        fell = resolve.collapsed(store, other["id"], look, seen)
                        framed = other["id"] in store.entities_in_frame(
                            look.get("inference_id"))
                        if seen is not None and seen < resolve.DIFFERENT_THING:
                            gates.append("unlike %.2f" % seen)
                        elif fell is not None and fell >= resolve.COLLAPSED_ALONE:
                            gates.append("collapses %.2f" % fell)
                        elif framed:
                            gates.append("frame taken")
                        else:
                            gates.append("open %s" % ("%.2f" % seen if seen is not None
                                                      else "-"))
                near.append({"entity_id": other["id"], "apart_m": round(apart, 3),
                             "height_m": p.get("height_above_floor_m"),
                             "gates": gates})
            log.append({"path": name, "entity_id": decision.entity_id,
                        "taken": list(taken), "placement": here,
                        "already_there": near,
                        "earlier_decisions": {str(i): (history or {}).get(i, [])[-3:]
                                              for i in taken}})
            return placed
        return logged

    class Patch:
        def __enter__(self):
            for name in originals:
                setattr(resolve, name, wrap(name))
            return self

        def __exit__(self, *exc):
            for name, function in originals.items():
                setattr(resolve, name, function)
            return False
    return Patch()


def run(database: Path, session: int, grid: Path | None, output: Path,
        limit_looks: int | None = None, name: str = "control",
        fast: bool = True, drop_share: float = 0.0, seed: int = 0) -> dict:
    assert not output.exists(), "choose a new output directory"
    output.mkdir(parents=True)
    groups = looks_in_order(database, session)
    if limit_looks:
        groups = groups[:limit_looks]
    if drop_share:
        # Noise, not a change: leave out a random share of the unlabelled looks'
        # regions, to see how far the scores move when nothing real changes.
        import random
        import score_session
        labelled = {i for s in score_session.label_sets().values()
                    for ids in [*s["objects"].values(), s["other"]] for i in ids}
        rng = random.Random(seed)
        groups = [kept for kept in ([row for row in g if row["id"] in labelled
                                     or rng.random() >= drop_share] for g in groups)
                  if kept]
    context = VARIANTS[name]()
    builder = getattr(context, "reach_from", replay.reach_from)
    reach = cached_reach(builder(str(grid)) if grid else None)
    columns = ["id", *replay.COLUMNS]
    log: list = []
    history: dict = {}
    started = time.time()
    with tempfile.TemporaryDirectory(prefix="ugv-session-replay-") as tmp:
        store = WorldStore(tmp)
        try:
            store.db.execute("REPLACE INTO meta(key, value) VALUES('map_session', ?)",
                             (str(session),))
            store.db.commit()
            # Every attachment and every placement, with the look that caused
            # it, so how a thing came to hold what it holds can be read back.
            look_number = [0]
            attached, placed = [], []
            attach, place = store.attach, store.place

            def logged_attach(entity_id, ids, why=""):
                attached.append([look_number[0], entity_id, list(ids), (why or "")[:120]])
                return attach(entity_id, ids, why)

            def logged_place(entity_id, placement, map_session):
                placed.append([look_number[0], entity_id,
                               {k: placement.get(k) for k in (
                                   "x_m", "y_m", "uncertainty_m", "height_above_floor_m",
                                   "height_sigma_m", "rays_agreeing")}])
                return place(entity_id, placement, map_session)
            store.attach, store.place = logged_attach, logged_place
            with context, founding_log(log, reach, history),                     (fast_appearance() if fast else _Nothing()):
                for count, group in enumerate(groups, 1):
                    look_number[0] = count
                    with store._lock, store.db:
                        for row in group:
                            store.db.execute(
                                "INSERT INTO observations (" + ",".join(columns)
                                + ") VALUES (" + ",".join("?" * len(columns)) + ")",
                                tuple(row.get(k) for k in columns))
                    outcome = resolve.resolve(store, reach=reach)
                    for one in outcome["decisions"]:
                        if one["outcome"] != resolve.NEW:
                            history.setdefault(one["observation_id"], []).append(
                                [count, one["outcome"], one["entity_id"],
                                 (one["why"] or "")[:160]])
                    if count % 100 == 0:
                        print("  %d/%d looks, %.0f s" % (count, len(groups),
                                                         time.time() - started),
                              flush=True)
            owners = dict(store.db.execute(
                "SELECT id, entity_id FROM observations"))
            things = {r["id"]: json.loads(r["placement_json"]) if r["placement_json"]
                      else None for r in store.db.execute(
                          "SELECT id, placement_json FROM entities")}
            with sqlite3.connect(output / "store.db") as dest:
                store.db.backup(dest)
        finally:
            store.close()
    result = {"variant": name, "fast_appearance": fast, "session": session,
              "looks": len(groups),
              "observations": sum(len(g) for g in groups),
              "seconds": round(time.time() - started, 1),
              "things": len(things), "owners": owners, "placements": things,
              "foundings": log, "attached": attached, "placed": placed,
              "look_shift": {str(k): round(v, 3) for k, v in LOOK_SHIFT.items()}}
    (output / "result.json").write_text(json.dumps(result) + "\n")
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--database", type=Path, required=True)
    p.add_argument("--session", type=int, required=True)
    p.add_argument("--grid", type=Path)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--limit-looks", type=int)
    p.add_argument("--variant", default="control", choices=sorted(VARIANTS))
    p.add_argument("--drop-share", type=float, default=0.0,
                   help="leave out this share of unlabelled regions, for a noise estimate")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--exact-appearance", action="store_true",
                   help="the resolver's own pure-Python similarity (slow)")
    a = p.parse_args()
    result = run(a.database, a.session, a.grid, a.output, a.limit_looks, a.variant,
                 fast=not a.exact_appearance, drop_share=a.drop_share, seed=a.seed)
    on_top = [f for f in result["foundings"] if f["already_there"]]
    print("%d looks, %d observations, %d things, %.0f s; %d foundings, %d beside a "
          "thing already placed there" % (result["looks"], result["observations"],
                                          result["things"], result["seconds"],
                                          len(result["foundings"]), len(on_top)))


if __name__ == "__main__":
    main()
