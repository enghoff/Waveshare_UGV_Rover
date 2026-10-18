"""Which of the world model's records are of a person, for the planner only.

On 2026-10-18 8 of the 149 records an autonomous run could aim a look at were
people -- someone sitting on the bed, someone crossing the living room -- and
looks at them land on nothing, since the person has gone. A logistic
regression on the mean of a record's regions' DINOv2 and SigLIP2 vectors
(`person.json`, frozen at ebb5e66e3f00, trained on 152 labelled records) flagged
9 of the 166 held out, all of them people, of 13 people there, and none of 50
records drawn at random from those nobody had labelled was a person
(docs/progress/2026-10-18-records-of-people-are-not-looked-at.md). A
classifier of every kind of non-object, on the same vectors, failed its blind
tests; this one is for people alone.

The job scores each record as it writes them (`job.once`); `autonomy/goals.py`
does not aim a look at one flagged. Nothing the filing reads, nothing the voice
or the console reads. A record's sums are kept between passes and only the
regions filed to it since are read; one that has lost regions (a merge into
another, or a filing started afresh) is summed again.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
import sqlite3

import numpy as np

MODEL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "person.json")
#: Regions read in one query.
CHUNK = 500


def load(path=MODEL):
    """The frozen model, checked, with its digest; None when missing or unfit."""
    try:
        with open(path) as f:
            m = json.load(f)
        dino, siglip = int(m["dims"]["dino"]), int(m["dims"]["siglip"])
        coef = np.asarray(m["coef"], np.float64)
        cut = float(m["threshold"])
        if coef.shape != (dino + siglip,) or not np.all(np.isfinite(coef)) or \
                not math.isfinite(float(m["intercept"])) or not 0.0 < cut < 1.0:
            return None
    except (OSError, ValueError, KeyError, TypeError):
        return None
    with open(path, "rb") as f:
        digest = hashlib.sha256(f.read()).hexdigest()[:12]
    return {"dino": dino, "siglip": siglip, "coef": coef, "intercept": float(m["intercept"]),
            "threshold": cut, "digest": digest}


def _unit(v):
    n = float(np.linalg.norm(v))
    return v / n if n > 0 else v


class People:
    """Scores records against the frozen model, keeping each record's sums of
    unit vectors between passes."""

    def __init__(self, db_path, model_path=MODEL):
        self.db_path = db_path
        self.model = load(model_path)
        self.generation = None
        self.sums = {}

    @property
    def digest(self):
        return None if self.model is None else self.model["digest"]

    def score(self, records, generation=None):
        """{record id: chance it is a person, or None when it holds no vectors
        or there is no model}. `records` as `job.things` writes them."""
        if self.model is None:
            return {r["id"]: None for r in records}
        if generation != self.generation:
            self.generation, self.sums = generation, {}
        need = {}
        for r in records:
            held = set(int(i) for i in r["observations"])
            have = self.sums.get(r["id"])
            if have is None or not have[0] <= held:
                have = (set(), np.zeros(self.model["dino"]), np.zeros(self.model["siglip"]), 0)
            self.sums[r["id"]] = have
            for i in held - have[0]:
                need[i] = r["id"]
        if need:
            self._read(need)
        alive = {r["id"] for r in records}
        for rid in list(self.sums):
            if rid not in alive:
                del self.sums[rid]
        m = self.model
        out = {}
        for r in records:
            _, d, s, n = self.sums[r["id"]]
            if not n:
                out[r["id"]] = None
                continue
            x = np.concatenate([_unit(d), _unit(s)])
            out[r["id"]] = 1.0 / (1.0 + math.exp(-(float(m["coef"] @ x) + m["intercept"])))
        return out

    def flag(self, p):
        return None if p is None or self.model is None else p >= self.model["threshold"]

    def _read(self, need):
        dw, sw = self.model["dino"] * 4, self.model["siglip"] * 4
        ids = sorted(need)
        con = sqlite3.connect("file:%s?mode=ro" % self.db_path, uri=True)
        try:
            for k in range(0, len(ids), CHUNK):
                part = ids[k:k + CHUNK]
                rows = con.execute("SELECT id, dino_alone_blob, siglip_blob FROM observations WHERE id IN (%s)"
                                   % ",".join("?" * len(part)), part).fetchall()
                for oid, d, s in rows:
                    rid = need[int(oid)]
                    covered, dsum, ssum, n = self.sums[rid]
                    covered.add(int(oid))
                    if d is None or s is None or len(d) != dw or len(s) != sw:
                        continue
                    dsum += _unit(np.frombuffer(d, np.float32).astype(np.float64))
                    ssum += _unit(np.frombuffer(s, np.float32).astype(np.float64))
                    self.sums[rid] = (covered, dsum, ssum, n + 1)
                for oid in part:
                    self.sums[need[oid]][0].add(oid)
        finally:
            con.close()
