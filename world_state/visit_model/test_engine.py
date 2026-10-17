"""Run in pieces, the engine files every region as one run over the history does.

    python test_engine.py LOOKS.npz FILING.json PIECES RECORDING.npz...

LOOKS is a looks file the desk pipeline made (data.py, wallplace.py,
extend_looks.py), FILING version 4's run over it (`run_versions.sh`) with the
same recordings in the same order. The looks are fed to the engine in PIECES
batches in the order they were taken, as the store adds them, each followed by
a step up to the newest region read; then the last visit is closed. Exit
status 0 when the two group the regions identically. Record names are compared
too, but only as a report: a picture whose clock was wrong when it was taken
(1970 after a cold boot) is filed where its time puts it in one run and where
it arrived in the other, and every later record is numbered on from there.
"""
import json
import math
import sys
import time

import numpy as np

import data
import engine
import recorded


def main():
    looks, filing_path, pieces = sys.argv[1], sys.argv[2], int(sys.argv[3])
    recs = [recorded.Recording(p) for p in sys.argv[4:]]
    a = data.load(looks)
    want = json.load(open(filing_path))
    order = np.argsort(a["t"], kind="stable")
    cuts = np.array_split(order, pieces)
    e = engine.Engine()
    t0 = time.time()
    for k, rows in enumerate(cuts):
        rows = np.sort(rows)
        e.add({key: v[rows] for key, v in a.items()})
        n = e.step(float(a["t"][rows].max()), recs)
        print("piece %d: %d regions read, %d filed, %d waiting" % (k, len(rows), n, len(e.pending["id"])),
              flush=True)
    e.step(math.inf, recs)
    got = {str(k): v for k, v in e.resolved().items()}
    same = groups(got) == groups(want)
    print("%d regions filed in %.0f s; grouped as the single run groups them: %s; same names: %s" % (
        len(got), time.time() - t0, same, got == want))
    if not same:
        diff = groups(got) ^ groups(want)
        print(len(diff), "groups differ, e.g.", [sorted(g)[:6] for g in list(diff)[:4]])
    sys.exit(0 if same else 1)


def groups(filing):
    out = {}
    for rid, oid in filing.items():
        out.setdefault(oid, set()).add(rid)
    return {frozenset(g) for g in out.values()}


if __name__ == "__main__":
    main()
