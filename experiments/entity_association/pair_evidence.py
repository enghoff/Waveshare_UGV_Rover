"""Which evidence tells two records of one object from records of two?

The merge proposer (`world_state/merging.py`) scores a pair of records on
appearance plus position. On 2026-10-08 it proposed none of the six joins the rug's
seven records need and one of the black cabinet's six: the cabinet's records
looked alike but each claimed to know its position better than it does, and the
rug's did not even look alike, seen in pieces from different sides. Aimed looks
had named both groups as same-object suspects, because a suspect compares one
picture region with each record rather than one record with another.

This lists every pair of placed records whose identity the labels settle, with:

    apart_m        distance between the two placements
    appearance     merging.appearance, the record-to-record log-likelihood ratio
    geometry       merging.geometry, the position log-likelihood ratio
    shared         both records hold a region of one picture (then two things)
    cofit_ab/_ba   share of A's looks that would also have fitted B -- B's
                   allowance, height and appearance at filing's own bar, the test
                   `aimed.also_fits` applies -- and the other way round

Labels: the three frozen sets `score_session.py` reads, a record taking the
object most of its labelled looks show (at least two, two thirds agreeing), and
the groups verified by photograph on 2026-10-08. Two records are one object when
their objects are the same (`score_session.CROSS_TIME` joins names across sets);
two when both names come from the same set and differ; otherwise unknown and
left out. Dining chairs, the owner and the floor are in no set.

    python experiments/entity_association/pair_evidence.py --store <copy of a store> \\
        --output <new-file>

Read-only: the store is copied to a temporary directory first.
"""
from __future__ import annotations

import argparse
import itertools
import json
import math
import sqlite3
import statistics
import sys
import tempfile
from collections import Counter, defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from world_state import locate, merging, resolve  # noqa: E402
from world_state.store import WorldStore  # noqa: E402
from score_session import CROSS_TIME, label_sets  # noqa: E402

#: Groups verified by photograph on 2026-10-08 (docs/progress/2026-10-08-claims-and-suspects.md).
VERIFIED_20261008 = {
    "rug-under-dining-table": ["object:338", "object:417", "object:514", "object:336",
                               "object:471", "object:353", "object:348"],
    "black-cabinet-living-room": ["object:461", "object:372", "object:396", "object:323",
                                  "object:242", "object:316", "object:414"],
}
#: Looks sampled per record for the co-fit test.
SAMPLE = 40


def identities(db) -> dict[str, tuple[str, str]]:
    """record -> (label set, unified object name)."""
    unify = {(s, n): name for name, places in CROSS_TIME.items() for s, n in places}
    counts: dict[str, Counter] = defaultdict(Counter)
    for key, labels in label_sets().items():
        for name, ids in labels["objects"].items():
            q = ",".join("?" * len(ids))
            for (owner,) in db.execute(
                    "SELECT entity_id FROM observations WHERE id IN (%s)" % q, ids):
                if owner:
                    counts[owner][(key, unify.get((key, name), name))] += 1
    out = {}
    for record, c in counts.items():
        (who, n), = c.most_common(1)
        if sum(c.values()) >= 2 and n / sum(c.values()) >= 2 / 3:
            out[record] = who
    for name, records in VERIFIED_20261008.items():
        for record in records:
            out[record] = ("photographs 2026-10-08", name)
    return out


def relation(a, b) -> str | None:
    (set_a, obj_a), (set_b, obj_b) = a, b
    if obj_a == obj_b:
        return "one"
    if set_a == set_b:
        return "two"
    return None


def cofit(store, looks, partner) -> float | None:
    """Share of `looks` that would also have fitted `partner`."""
    placement = partner.placement
    hits = tried = 0
    for row in looks:
        ray = resolve.ray_of(row, None)
        if ray is None:
            continue
        tried += 1
        if resolve._allowance_used(placement, ray) is None:
            continue
        if not locate.stands_as_high(placement, ray):
            continue
        seen = resolve.appearance(store, partner.id, row.get("dino_blob") or b"")
        if seen is not None and seen >= resolve.DIFFERENT_THING:
            hits += 1
    return round(hits / tried, 3) if tried else None


def main():
    a = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    a.add_argument("--store", type=Path, required=True)
    a.add_argument("--output", type=Path, required=True)
    args = a.parse_args()
    assert not args.output.exists(), "choose a new output file"
    tmp = tempfile.mkdtemp(prefix="pair-evidence-")
    store = WorldStore(tmp)
    with sqlite3.connect(args.store.resolve().as_uri() + "?mode=ro", uri=True) as src:
        src.backup(store.db)
    store._create()
    session = store.map_session()
    things = {t.id: t for t in merging._things(store, session)}
    xs = [t.placement["x_m"] for t in things.values()]
    ys = [t.placement["y_m"] for t in things.values()]
    density = len(things) / ((max(xs) - min(xs) + 1.0) * (max(ys) - min(ys) + 1.0))
    who = {r: i for r, i in identities(store.db).items() if r in things}
    looks = {}
    for record in who:
        rows = [r for r in store.observations(record, vectors=True)
                if r.get("bearing_deg") is not None]
        if len(rows) > SAMPLE:
            step = (len(rows) - 1) / (SAMPLE - 1)
            rows = [rows[round(i * step)] for i in range(SAMPLE)]
        looks[record] = rows
    pairs = []
    for x, y in itertools.combinations(sorted(who), 2):
        rel = relation(who[x], who[y])
        if rel is None:
            continue
        A, B = things[x], things[y]
        apart = math.hypot(A.placement["x_m"] - B.placement["x_m"],
                           A.placement["y_m"] - B.placement["y_m"])
        if apart > merging.MAX_APART_M:
            continue
        app = merging.appearance(A, B)
        pairs.append({
            "a": x, "b": y, "relation": rel, "objects": [who[x][1], who[y][1]],
            "apart_m": round(apart, 2),
            "appearance": None if app is None else round(app, 2),
            "geometry": round(merging.geometry(A, B, density), 2),
            "shared": bool(A.pictures & B.pictures),
            "cofit_ab": cofit(store, looks[x], B), "cofit_ba": cofit(store, looks[y], A)})
    store.close()
    args.output.write_text(json.dumps({"store": str(args.store), "map_session": session,
                                       "records_labelled": len(who), "density": density,
                                       "pairs": pairs}, indent=1) + "\n")
    for rel in ("one", "two"):
        part = [p for p in pairs if p["relation"] == rel]
        print(rel, "object:", len(part), "pairs")
        for key in ("apart_m", "appearance", "geometry", "cofit_ab", "cofit_ba"):
            values = [p[key] for p in part if p[key] is not None]
            if values:
                print("   %-10s median %6.2f" % (key, statistics.median(values)))
        print("   shared picture", sum(p["shared"] for p in part))


if __name__ == "__main__":
    main()
