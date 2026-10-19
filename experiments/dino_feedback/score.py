#!/usr/bin/env python3
"""Score the settling loop against the bar in PREREGISTRATION.md.

    python experiments/dino_feedback/score.py <cache.npz> [--json out.json]

`<cache.npz>` is what `replay.py` wrote. Every number the bar asks for is
printed, followed by the verdict the bar gives; nothing here chooses a setting.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from collections import defaultdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import settle  # noqa: E402
from replay import MANIFEST  # noqa: E402

#: Records that are one physical object, from the labels and the photographs.
SAME = {"object:351": "object:246", "object:340": "object:249", "object:400": "object:383"}
CHAIRS = {"object:247", "object:302", "object:350"}
NOT_AN_OBJECT = {"object:259"}
#: Pairs of records left out of T1 because they may or may not be one object.
UNSURE = [({"object:332"}, {"object:385"}),
          ({"object:343"}, {"object:249", "object:340", "object:335"}),
          ({"object:387"}, {"object:383", "object:400"})]
#: The benchmark's own exclusion (perception_bench/README.md): a look labelled as the
#: cow painting whose frame shows a different painting at the edge, disputed.
EXCLUDED = {61656}
BOOTSTRAPS = 2000
SEED = 20261019


def unit(v):
    v = np.asarray(v, dtype=np.float64)
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else None


def auc(pos, neg, wpos=None, wneg=None):
    """Weighted area under the ROC curve, ties counted half."""
    pos, neg = np.asarray(pos, float), np.asarray(neg, float)
    wpos = np.ones(len(pos)) if wpos is None else np.asarray(wpos, float)
    wneg = np.ones(len(neg)) if wneg is None else np.asarray(wneg, float)
    if wpos.sum() == 0 or wneg.sum() == 0:
        return float("nan")
    scores = np.concatenate([pos, neg])
    labels = np.concatenate([np.ones(len(pos)), np.zeros(len(neg))])
    weights = np.concatenate([wpos, wneg])
    order = np.argsort(scores, kind="mergesort")
    scores, labels, weights = scores[order], labels[order], weights[order]
    total = 0.0
    neg_below = 0.0
    i = 0
    while i < len(scores):
        j = i
        while j < len(scores) and scores[j] == scores[i]:
            j += 1
        block_pos = (weights[i:j] * labels[i:j]).sum()
        block_neg = (weights[i:j] * (1 - labels[i:j])).sum()
        total += block_pos * (neg_below + 0.5 * block_neg)
        neg_below += block_neg
        i = j
    return total / (wpos.sum() * wneg.sum())


def load(path):
    # Each array read once: indexing the archive itself decompresses it every time.
    data = dict(np.load(path))
    looks = {}
    for k, oid in enumerate(data["id"]):
        looks[int(oid)] = {key: data[key][k] for key in data if key != "id"}
    return looks


def embeddings(looks, temperatures=(0.05, 0.1, 0.2)):
    """Every method's vector for every look; None where a method has none."""
    out = defaultdict(dict)
    for oid, look in looks.items():
        tokens = look["tokens"].astype(np.float32)
        out["B1"][oid] = unit(look["stored_dino"])
        out["B2"][oid] = unit(look["stored_alone"])
        out["C3"][oid] = unit(look["settled_alone"])
        for t in temperatures:
            got = settle.settle(tokens[1:], look["share"], look["real"], temperature=t)
            name = "C2" if t == settle.TEMPERATURE else f"C2@{t}"
            out[name][oid] = None if got.embedding is None else unit(got.embedding)
            if t == settle.TEMPERATURE:
                out["C1"][oid] = None if got.pooled is None else unit(got.pooled)
                look["r"] = got.r
    return out


def labels():
    manifest = json.load(open(MANIFEST))
    records = {}
    for item in manifest["items"]:
        if item["kind"] == "per_look_identity":
            truth = item["ground_truth"]
            records[item["id"].split("/")[1]] = (
                [o for o in truth["main_looks"] if o not in EXCLUDED],
                [o for o in truth["odd"] if o not in EXCLUDED])
    return records


def unsure(a, b):
    return any((a in x and b in y) or (a in y and b in x) for x, y in UNSURE)


def t1_pairs(records):
    """(look, look, same?, group a, group b) for every scored pair of main looks."""
    mains = [(oid, rec) for rec, (main, _) in records.items() if rec not in NOT_AN_OBJECT
             for oid in main]
    pairs = []
    for i in range(len(mains)):
        for j in range(i + 1, len(mains)):
            (a, ra), (b, rb) = mains[i], mains[j]
            if ra in CHAIRS and rb in CHAIRS:
                continue
            if unsure(ra, rb):
                continue
            ga, gb = SAME.get(ra, ra), SAME.get(rb, rb)
            pairs.append((a, b, ga == gb, ga, gb))
    return pairs


class Sorted:
    """One method's scores in order, so a reweighted AUC is two cumulative sums."""

    def __init__(self, scores, positive, keys_a, keys_b=None):
        order = np.argsort(scores, kind="mergesort")
        self.scores = np.asarray(scores)[order]
        self.positive = np.asarray(positive, bool)[order]
        self.a = np.asarray(keys_a)[order]
        self.b = None if keys_b is None else np.asarray(keys_b)[order]

    def auc(self, weight_of=None):
        if weight_of is None:
            w = np.ones(len(self.scores))
        elif self.b is None:
            w = weight_of[self.a]
        else:     # a same-object pair counts once per copy of its object
            w = np.where(self.positive, weight_of[self.a], weight_of[self.a] * weight_of[self.b])
        wp, wn = w * self.positive, w * ~self.positive
        if wp.sum() == 0 or wn.sum() == 0:
            return float("nan")
        below = np.cumsum(wn) - wn
        return float((wp * below).sum() / (wp.sum() * wn.sum()))


def t1_sorted(vectors, pairs, group_index):
    scores, same, ga, gb = [], [], [], []
    for a, b, is_same, g1, g2 in pairs:
        scores.append(float(vectors[a] @ vectors[b]))
        same.append(is_same)
        ga.append(group_index[g1])
        gb.append(group_index[g2])
    return Sorted(scores, same, ga, gb)


def t2_scores(vectors, records):
    """(record, is_main, median cosine to the record's other main looks) per look."""
    rows = []
    for rec, (main, odd) in records.items():
        if not odd:
            continue
        for oid, is_main in [(m, True) for m in main] + [(o, False) for o in odd]:
            v = vectors.get(oid)
            others = [vectors.get(m) for m in main if m != oid]
            others = [u for u in others if u is not None]
            if v is None or not others:
                rows.append((rec, is_main, None))
                continue
            rows.append((rec, is_main, float(np.median([v @ u for u in others]))))
    return rows


def t2_sorted(rows, record_index):
    kept = [r for r in rows if r[2] is not None]
    return Sorted([r[2] for r in kept], [r[1] for r in kept], [record_index[r[0]] for r in kept])


def held_out(rows_by_method, methods):
    """Leave-one-record-out logistic regression on these methods' T2 scores."""
    from sklearn.linear_model import LogisticRegression
    from sklearn.preprocessing import StandardScaler
    base = rows_by_method[methods[0]]
    recs = np.array([r[0] for r in base])
    y = np.array([0 if r[1] else 1 for r in base])           # 1 = stray
    X = np.array([[rows_by_method[m][k][2] for m in methods] for k in range(len(base))], float)
    ok = ~np.isnan(X).any(axis=1)
    recs, y, X = recs[ok], y[ok], X[ok]
    pred = np.zeros(len(y))
    for rec in np.unique(recs):
        train, test = recs != rec, recs == rec
        scaler = StandardScaler().fit(X[train])
        model = LogisticRegression().fit(scaler.transform(X[train]), y[train])
        pred[test] = model.predict_proba(scaler.transform(X[test]))[:, 1]
    return auc(pred[y == 1], pred[y == 0])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("cache")
    parser.add_argument("--json")
    args = parser.parse_args()

    looks = load(args.cache)
    vectors = embeddings(looks)
    records = labels()
    methods = ["B1", "B2", "C1", "C2", "C3", "C2@0.05", "C2@0.2"]
    missing = {m: sum(1 for v in vectors[m].values() if v is None) for m in methods}
    # A look a method cannot embed (no mask cell to pool) falls back to the
    # stored summary token, so every method is scored on the same looks.
    for m in methods:
        for oid, v in vectors[m].items():
            if v is None:
                vectors[m][oid] = vectors["B1"][oid]

    pairs = t1_pairs(records)
    groups = sorted({g for p in pairs for g in (p[3], p[4])})
    group_index = {g: k for k, g in enumerate(groups)}
    rows = {m: t2_scores(vectors[m], records) for m in methods}
    strays = sorted({r[0] for r in rows["B1"]})
    record_index = {r: k for k, r in enumerate(strays)}
    s1 = {m: t1_sorted(vectors[m], pairs, group_index) for m in methods}
    s2 = {m: t2_sorted(rows[m], record_index) for m in methods}
    # Point estimates with ties counted half; the bootstrap ignores ties, which
    # among continuous cosines it can afford to.
    t1_auc = {m: auc(s1[m].scores[s1[m].positive], s1[m].scores[~s1[m].positive]) for m in methods}
    t2_auc = {m: auc(s2[m].scores[s2[m].positive], s2[m].scores[~s2[m].positive]) for m in methods}

    rng = np.random.default_rng(SEED)
    boot1 = defaultdict(list)
    boot2 = defaultdict(list)
    for _ in range(BOOTSTRAPS):
        gw = np.bincount(rng.integers(0, len(groups), len(groups)), minlength=len(groups)).astype(float)
        rw = np.bincount(rng.integers(0, len(strays), len(strays)), minlength=len(strays)).astype(float)
        for m in methods:
            boot1[m].append(s1[m].auc(gw))
            boot2[m].append(s2[m].auc(rw))
    ci = lambda xs: (float(np.nanpercentile(xs, 2.5)), float(np.nanpercentile(xs, 97.5)))
    diff2 = {(a, b): ci(np.array(boot2[a]) - np.array(boot2[b]))
             for a in ("C1", "C2", "C3") for b in ("B1", "B2", "C1") if a != b}
    diff1 = {(a, b): ci(np.array(boot1[a]) - np.array(boot1[b]))
             for a in ("C1", "C2", "C3") for b in ("B1", "B2", "C1") if a != b}

    joint = {"B1+B2": held_out(rows, ["B1", "B2"])}
    for m in ("C1", "C2", "C3"):
        joint[f"B1+B2+{m}"] = held_out(rows, ["B1", "B2", m])

    npos = sum(p[2] for p in pairs)
    print(f"T1: {npos} same-object pairs, {len(pairs) - npos} different, {len(groups)} objects")
    nm = sum(1 for r in rows["B1"] if r[1]); ns = len(rows["B1"]) - nm
    print(f"T2: {nm} main looks, {ns} strays, {len(strays)} records")
    print(f"looks a method could not embed (fell back to B1): {missing}")
    print("\nmethod    T1 AUC  [95%]            T2 AUC  [95%]")
    for m in methods:
        print(f"{m:8s}  {t1_auc[m]:.3f}  [{ci(boot1[m])[0]:.3f}, {ci(boot1[m])[1]:.3f}]   "
              f"{t2_auc[m]:.3f}  [{ci(boot2[m])[0]:.3f}, {ci(boot2[m])[1]:.3f}]")
    print("\ndifferences (point, 95% interval)")
    for (a, b), (lo, hi) in diff1.items():
        print(f"  T1 {a}-{b}: {t1_auc[a] - t1_auc[b]:+.3f} [{lo:+.3f}, {hi:+.3f}]")
    for (a, b), (lo, hi) in diff2.items():
        print(f"  T2 {a}-{b}: {t2_auc[a] - t2_auc[b]:+.3f} [{lo:+.3f}, {hi:+.3f}]")
    print("\nheld-out logistic regression on T2 (stray vs main):")
    for k, v in joint.items():
        print(f"  {k:12s} {v:.3f}")

    # The bar.
    def bar(m):
        best1 = max(t1_auc["B1"], t1_auc["B2"])
        best2 = max(t2_auc["B1"], t2_auc["B2"])
        c1 = t1_auc[m] >= best1 - 0.01
        c2 = (t2_auc[m] - best2 >= 0.03 and diff2[(m, "B1")][0] > 0 and diff2[(m, "B2")][0] > 0)
        c3 = joint[f"B1+B2+{m}"] - joint["B1+B2"] >= 0.03
        c4 = (m == "C1") or (t2_auc[m] > t2_auc["C1"] and diff2[(m, "C1")][0] > 0
                             and t1_auc[m] >= t1_auc["C1"] - 0.01)
        return bool(c1), bool(c2), bool(c3), bool(c4)
    print("\nbar: 1 no loss of identity, 2 strays caught better, 3 beyond B1+B2, 4 feedback is what helps")
    verdicts = {m: bar(m) for m in ("C1", "C2", "C3")}
    for m, v in verdicts.items():
        print(f"  {m}: " + "  ".join(f"{i + 1}:{'pass' if x else 'FAIL'}" for i, x in enumerate(v)))
    if all(verdicts["C2"]):
        verdict = "worth building: C2 passes all four"
    elif any(all(verdicts[m][:3]) for m in ("C1", "C3")):
        verdict = "feedback loop rejected; pooling/masking passes 1-3 and is a separate lead"
    else:
        verdict = "rejected"
    print(f"\nverdict: {verdict}")

    # Descriptive: how far the settled assignment moved from the mask.
    moved = {}
    for oid, look in looks.items():
        valid = look["real"] >= settle.REAL
        m, r = look["share"][valid], look["r"][valid]
        moved[oid] = 1.0 - float(np.minimum(m, r).sum() / max(np.maximum(m, r).sum(), 1e-9))
    mv_rows = [(rec, im, moved.get(oid)) for rec, (main, odd) in records.items() if odd
               for oid, im in [(x, True) for x in main] + [(x, False) for x in odd]]
    # Larger movement should mean stray, so the main looks are the negatives here.
    moved_auc = auc([s for _, im, s in mv_rows if not im], [s for _, im, s in mv_rows if im])
    print(f"\nhow far the loop moved the mask, as a stray flag: AUC {moved_auc:.3f} "
          f"(useful at 0.65); median movement main {np.median([s for _, im, s in mv_rows if im]):.3f}, "
          f"stray {np.median([s for _, im, s in mv_rows if not im]):.3f}")

    # Descriptive: top-1 retrieval of each main look's own object.
    group_of = {}
    for rec, (main, _) in records.items():
        if rec in NOT_AN_OBJECT:
            continue
        g = "chairs" if rec in CHAIRS else SAME.get(rec, rec)
        for oid in main:
            group_of[oid] = (g, rec)
    members = defaultdict(list)
    for oid, (g, rec) in group_of.items():
        members[g].append(oid)
    print("\ntop-1 retrieval of a main look's own object (median cosine):")
    for m in methods[:5]:
        hit = total = 0
        for oid, (g, rec) in group_of.items():
            best, best_g = -2, None
            for h, ids in members.items():
                recs_h = {group_of[i][1] for i in ids}
                if any(unsure(rec, x) for x in recs_h):
                    continue
                sims = [vectors[m][oid] @ vectors[m][i] for i in ids if i != oid]
                if not sims:
                    continue
                s = float(np.median(sims))
                if s > best:
                    best, best_g = s, h
            hit += best_g == g
            total += 1
        print(f"  {m}: {hit}/{total} = {hit / total:.3f}")

    print("\nper-record T2 AUC (B1, B2, C1, C2, C3):")
    for rec in strays:
        vals = [auc([r[2] for r in rows[m] if r[0] == rec and r[1]],
                    [r[2] for r in rows[m] if r[0] == rec and not r[1]])
                for m in ("B1", "B2", "C1", "C2", "C3")]
        n_odd = sum(1 for r in rows["B1"] if r[0] == rec and not r[1])
        print(f"  {rec} ({n_odd} strays): " + "  ".join(f"{v:.2f}" for v in vals))

    if args.json:
        json.dump({"t1": t1_auc, "t2": t2_auc, "joint": joint, "moved_auc": moved_auc,
                   "verdicts": {m: list(v) for m, v in verdicts.items()}, "verdict": verdict,
                   "t1_ci": {m: ci(boot1[m]) for m in methods},
                   "t2_ci": {m: ci(boot2[m]) for m in methods},
                   "t2_diff_ci": {f"{a}-{b}": v for (a, b), v in diff2.items()},
                   "t1_diff_ci": {f"{a}-{b}": v for (a, b), v in diff1.items()}},
                  open(args.json, "w"), indent=1,
                  default=lambda o: bool(o) if isinstance(o, np.bool_) else float(o))


if __name__ == "__main__":
    main()
