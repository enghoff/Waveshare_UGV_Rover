"""A merge pass over a replayed session's final store, scored like the replay.

Run after `replay_session.py`: reads its `store.db`, joins records by one rule,
and writes a `result.json` whose owners are the joined ones, for
`score_session.py` to score against the frozen labels. The store is copied;
nothing written goes back.

    cofit     join two placed records when at least `--threshold` of each one's
              looks would also have fitted the other (B's allowance, height and
              appearance at filing's own bar -- `aimed.also_fits`'s test; see
              `pair_evidence.py`), strongest pairs first
    propose   `merging.propose` applied round after round until it proposes
              nothing, the reviewed merge the store already has
    both      propose's rounds, then cofit over what is left

Neither joins two records that hold regions of one picture: a thing holds one
region of a picture, and `WorldStore.merge` refuses such a pair. Joins are
transitive (a group takes every record joined to any of its members), and a join
that would bring two of one picture's regions into one group is skipped.

    python experiments/entity_association/merge_after_session.py --replay <replay dir>
        --rule cofit --threshold 0.1 --output <new directory>
"""
from __future__ import annotations

import argparse
import itertools
import json
import math
import sqlite3
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from world_state import merging  # noqa: E402
from world_state.store import WorldStore  # noqa: E402
from pair_evidence import SAMPLE, cofit  # noqa: E402


def _sampled(store, record):
    rows = [r for r in store.observations(record, vectors=True)
            if r.get("bearing_deg") is not None]
    if len(rows) > SAMPLE:
        step = (len(rows) - 1) / (SAMPLE - 1)
        rows = [rows[round(i * step)] for i in range(SAMPLE)]
    return rows


def cofit_pairs(store, session, threshold):
    things = {t.id: t for t in merging._things(store, session)}
    looks = {r: _sampled(store, r) for r in things}
    scored = []
    for a, b in itertools.combinations(sorted(things), 2):
        A, B = things[a], things[b]
        if math.hypot(A.placement["x_m"] - B.placement["x_m"],
                      A.placement["y_m"] - B.placement["y_m"]) > merging.MAX_APART_M:
            continue
        if A.pictures & B.pictures:
            continue
        ab = cofit(store, looks[a], B)
        if not ab or ab < threshold:
            continue
        ba = cofit(store, looks[b], A)
        if not ba or ba < threshold:
            continue
        scored.append((min(ab, ba), a, b))
    scored.sort(reverse=True)
    return things, [(a, b, s) for s, a, b in scored]


def join(things, pairs):
    """Union the pairs in order, skipping any join that would put two regions of
    one picture in a group. Returns record -> group root and the joins made."""
    root = {r: r for r in things}
    pictures = {r: set(things[r].pictures) for r in things}

    def find(r):
        while root[r] != r:
            root[r] = root[root[r]]
            r = root[r]
        return r
    made = []
    for a, b, score in pairs:
        ra, rb = find(a), find(b)
        if ra == rb or pictures[ra] & pictures[rb]:
            continue
        keep, gone = sorted((ra, rb), key=lambda r: -len(pictures[r]))
        root[gone] = keep
        pictures[keep] |= pictures.pop(gone)
        made.append([a, b, score])
    return {r: find(r) for r in things}, made


def propose_rounds(store):
    made = []
    for _ in range(20):
        got = merging.propose(store)
        pairs = [[p["keep"], p["gone"]] for p in got.get("proposals") or []]
        if not pairs:
            break
        for keep, gone in pairs:
            if store.merge(keep, gone).get("ok", True) is not False:
                made.append([keep, gone, None])
    return made


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--replay", type=Path, required=True)
    p.add_argument("--rule", choices=("cofit", "propose", "both"), required=True)
    p.add_argument("--threshold", type=float, default=0.1)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    assert not a.output.exists(), "choose a new output directory"
    a.output.mkdir(parents=True)
    replayed = json.loads((a.replay / "result.json").read_text())
    started = time.time()
    tmp = tempfile.mkdtemp(prefix="merge-after-")
    store = WorldStore(tmp)
    with sqlite3.connect((a.replay / "store.db").resolve().as_uri() + "?mode=ro",
                         uri=True) as src:
        src.backup(store.db)
    store._create()
    session = replayed["session"]
    if a.rule == "cofit":
        things, pairs = cofit_pairs(store, session, a.threshold)
        group, made = join(things, pairs)
        owners = {k: (group.get(v, v) if v else v) for k, v in replayed["owners"].items()}
    else:
        made = propose_rounds(store)
        owners = {str(k): v for k, v in store.db.execute(
            "SELECT id, entity_id FROM observations")}
        if a.rule == "both":
            things, pairs = cofit_pairs(store, session, a.threshold)
            group, more = join(things, pairs)
            owners = {k: (group.get(v, v) if v else v) for k, v in owners.items()}
            made += more
    store.close()
    result = {**{k: v for k, v in replayed.items() if k not in ("owners",)},
              "variant": "%s+merge-%s-%s" % (replayed.get("variant"), a.rule, a.threshold),
              "owners": owners, "merges": made,
              "merge_seconds": round(time.time() - started, 1)}
    (a.output / "result.json").write_text(json.dumps(result) + "\n")
    print("%s: %d joins in %.0f s" % (result["variant"], len(made), result["merge_seconds"]))


if __name__ == "__main__":
    main()
