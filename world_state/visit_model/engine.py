"""Version 4 of the world model kept per visit, run a batch at a time.

`model.run` files a whole history in one pass: one visit window after another,
each clustered, matched against memory, checked for absence and consolidated.
`Engine` does the same visits in the same order, but takes the regions as the
store adds them and keeps its memory between batches, so that the job on the
rover (`job.py`) never files the history twice. A visit is taken in only once it
has closed: no region of it within GAP_S of the newest one read, so a later
region cannot join it. Run in pieces it files every region as one run over the
whole history does (`test_engine.py`).

Nothing here decides anything `model.py` does not: the loop below is
`model.run`'s, and model.py, data.py, lidar.py, recorded.py, poses.py, walls.py
and wallplace.py are version 4's as frozen on branch visit-model (`cabe003`), with
batch entry points added to the last two and to colour.py.
"""
import os
import pickle
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import colour  # noqa: E402
import data  # noqa: E402
import likeness  # noqa: E402
import model  # noqa: E402
import wallplace  # noqa: E402
import walls  # noqa: E402

METRIC = os.path.join(HERE, "metric-1015.npz")
WALLS = os.path.join(HERE, "walls.npz")


def _concat(a, b):
    if a is None:
        return b
    return {k: np.concatenate([a[k], b[k]]) for k in a}


def _take(a, rows):
    return {k: v[rows] for k, v in a.items()}


def prepare(db_path, recordings=(), after_id=None):
    """The store's regions after `after_id` as the model reads them: the
    loader's arrays, wall places, colour and the learned likeness, as
    data.py, wallplace.py, colour.py and extend_looks.py make them on the desk."""
    a = data.build(db_path, recordings, after_id=after_id)
    wallplace.place(a, walls.load(WALLS))
    ids, cols, _n = colour.colours(db_path, data.MAP_SESSION, after_id=after_id)
    at = {int(i): k for k, i in enumerate(ids)}
    rows = np.array([at.get(int(i), -1) for i in a["id"]], dtype=np.int64)
    have = rows >= 0
    col = np.zeros((len(rows), colour.WIDTH), np.float32)
    col[have] = cols[rows[have]]
    a["colour"], a["has_colour"] = col, have
    m = dict(np.load(METRIC))
    a["lvec"] = likeness.transform(m, likeness.features(a)) if len(rows) else \
        np.zeros((0, m["M"].shape[0]), np.float32)
    return a


class Engine:
    def __init__(self):
        self.mem = model.Memory()
        self.filing = {}
        self.pending = None
        self.last_id = 0
        self.taken_until = None
        #: What names these records (job.mint); None until the job names it.
        self.generation = None
        #: Whether every visit it filed counted looks for each record
        #: (model.looked_for); one saved before that files the history again.
        self.counts_looks = True

    def add(self, a):
        """Regions read from the store, waiting for their visit to close."""
        if len(a["id"]):
            self.last_id = max(self.last_id, int(a["id"].max()))
            self.pending = _concat(self.pending, a)

    def closed(self, until):
        """The pending regions' visit windows that have closed when nothing
        later than `until` has been read: every window but the last, and the
        last when its newest region is GAP_S older than `until`."""
        if self.pending is None or not len(self.pending["id"]):
            return []
        wins = list(model.windows(self.pending["t"]))
        if wins and float(self.pending["t"][wins[-1]].max()) > until - model.GAP_S:
            wins = wins[:-1]
        return wins

    def step(self, until, recs=(), decisions=None):
        """Files every visit that has closed, in model.run's order; returns how
        many regions it filed. `decisions` collects what each one decided."""
        wins = self.closed(until)
        if not wins:
            return 0
        decisions = [] if decisions is None else decisions
        a = self.pending
        rows = np.concatenate(wins)
        sub = _take(a, rows)
        L = model.Looks(sub)
        start = 0
        for w in wins:
            idx = np.arange(start, start + len(w))
            start += len(w)
            mem = self.mem
            mem.visit += 1
            clusters = model.cluster(L, idx)
            t = float(sub["t"][idx].max())
            filed = model.reconcile(L, mem, clusters, t, decisions)
            if recs:
                model.solid_votes(L, mem, filed, recs)
                model.absent(L, mem, idx, filed, recs, decisions)
            model.looked_for(L, mem, idx, filed)
            model.consolidate(mem, sorted({mem.resolve(v) for v in filed.values()}), t, decisions)
            model.settle_pending(mem, t, decisions)
            self.filing.update({int(sub["id"][i]): oid for i, oid in filed.items()})
        keep = np.ones(len(a["id"]), bool)
        keep[rows] = False
        self.pending = _take(a, np.flatnonzero(keep))
        self.taken_until = float(sub["t"].max())
        return len(rows)

    def resolved(self):
        """{region id: record} with every merge so far followed, as model.run
        returns it."""
        return {k: self.mem.resolve(v) for k, v in self.filing.items()}

    def save(self, path):
        tmp = path + ".tmp"
        with open(tmp, "wb") as f:
            pickle.dump(self, f, protocol=pickle.HIGHEST_PROTOCOL)
            # On disk before it replaces the last one, so that a power cut
            # leaves one or the other whole rather than an empty file.
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)

    @staticmethod
    def load(path):
        with open(path, "rb") as f:
            return pickle.load(f)
