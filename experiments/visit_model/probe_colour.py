"""Why a record let a differently coloured thing in: the model run over every
look with each colour comparison against one record logged.

    python probe_colour.py LOOKS.npz RECORD SINCE REC.npz...

Prints, for each comparison made against RECORD after SINCE (epoch seconds),
the score the veto saw and, per colour sample the record held, how well the
cluster matched it, with the regions each sample came from.
"""
import sys
import time

import numpy as np

import data
import model

looks, record, since = sys.argv[1], sys.argv[2], float(sys.argv[3])
clock = {"t": 0.0}
origin = {}

real_reconcile = model.reconcile


def reconcile(L, mem, clusters, t, decisions):
    clock["t"] = t
    return real_reconcile(L, mem, clusters, t, decisions)


real_colour = model.Memory.colour


def colour(self, cols, o):
    s = real_colour(self, cols, o)
    if o["id"] == record and clock["t"] >= since and cols is not None and len(cols) and len(o["col"]):
        per = (cols @ o["col"].T).max(axis=0)
        print("%s  veto score %.2f  per sample: %s" % (
            time.strftime("%m-%d %H:%M:%S", time.localtime(clock["t"])), s,
            " ".join("%.2f" % v for v in per)), flush=True)
    return s


model.reconcile = reconcile
model.Memory.colour = colour
a = data.load(looks)
by_bytes = {}
for i, c in zip(a["id"], a["colour"]):
    by_bytes.setdefault(np.asarray(c, np.float32).tobytes(), []).append(int(i))
shown = set()
real_colour2 = model.Memory.colour


def colour2(self, cols, o):
    if o["id"] == record and clock["t"] >= since and "x" not in shown:
        shown.add("x")
        for k, c in enumerate(o["col"]):
            print("   sample %d from regions %s" % (k, by_bytes.get(np.asarray(c, np.float32).tobytes(), "?")[:4]))
    return real_colour2(self, cols, o)


model.Memory.colour = colour2
recs = [model.recorded.Recording(p) for p in sys.argv[4:]]
print("running", flush=True)
model.run(a, recs)
