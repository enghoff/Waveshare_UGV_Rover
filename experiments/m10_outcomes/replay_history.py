"""The world model's records as they stood after every visit, by replaying it.

    python replay_history.py LOOKS.npz OUT.npz RECORDING.npz ...

The rover's world model (world_state/visit_model/model.py, version 4) files
looks a visit at a time and keeps only the latest state of each record, so how
much one look improved a record's placement cannot be read back from it. This
runs the same loop as `model.run` and, after each visit, notes every record
the visit filed into or merged: its place, its spread (the placement's
standard deviation) and how many visits it has been seen on. A look's effect on
a record is then its spread at the last note before the look's visit against
its first note from that visit on.

Writes `visit` (index), `t`, `oid` (as resolved at the end), `x`, `y`,
`spread`, `visits` per note, and `region`/`record`: every region's final
filing. Run on the desk from the repository root, on a looks file built by
experiments/visit_model/prepare_session.sh.
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "world_state", "visit_model"))
import data  # noqa: E402
import model  # noqa: E402
import recorded  # noqa: E402


def replay(a, recs):
    L = model.Looks(a)
    mem = model.Memory()
    filing, decisions = {}, []
    notes = []
    for k, idx in enumerate(model.windows(a["t"])):
        mem.visit += 1
        clusters = model.cluster(L, idx)
        t = float(a["t"][idx].max())
        before = len(decisions)
        filed = model.reconcile(L, mem, clusters, t, decisions)
        if recs:
            model.solid_votes(L, mem, filed, recs)
            model.absent(L, mem, idx, filed, recs, decisions)
        model.looked_for(L, mem, idx, filed)
        model.consolidate(mem, sorted({mem.resolve(v) for v in filed.values()}), t, decisions)
        model.settle_pending(mem, t, decisions)
        filing.update({int(a["id"][i]): oid for i, oid in filed.items()})
        touched = {mem.resolve(v) for v in filed.values()}
        touched |= {mem.resolve(d["into"]) for d in decisions[before:] if "into" in d}
        by_id = {o["id"]: o for o in mem.objects}
        for oid in sorted(touched):
            o = by_id.get(oid)
            if o is None:
                continue
            notes.append((k, t, oid, float(o["x"]), float(o["y"]),
                          math.sqrt(max(float(o["var"]), 0.0)), len(o["visits"])))
    resolved = {r: mem.resolve(v) for r, v in filing.items()}
    return notes, resolved, mem


def main():
    looks, out = sys.argv[1], sys.argv[2]
    a = data.load(looks)
    recs = [recorded.Recording(p) for p in sys.argv[3:]]
    notes, resolved, mem = replay(a, recs)
    final = {}
    for o in mem.objects:
        final[o["id"]] = o["id"]
    regions = np.array(sorted(resolved), dtype=np.int64)
    np.savez_compressed(
        out,
        visit=np.array([n[0] for n in notes], np.int32),
        t=np.array([n[1] for n in notes]),
        oid=np.array([mem.resolve(n[2]) for n in notes]),
        oid_then=np.array([n[2] for n in notes]),
        x=np.array([n[3] for n in notes]), y=np.array([n[4] for n in notes]),
        spread=np.array([n[5] for n in notes]),
        visits=np.array([n[6] for n in notes], np.int32),
        region=regions, record=np.array([resolved[int(r)] for r in regions]))
    print("%d notes over %d visits, %d regions filed into %d records" % (
        len(notes), (notes[-1][0] + 1) if notes else 0, len(regions), len(set(resolved.values()))))


if __name__ == "__main__":
    main()
