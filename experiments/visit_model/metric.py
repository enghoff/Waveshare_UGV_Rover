"""A likeness learned from the rover's own pictures, with no labels.

    python metric.py LOOKS.npz FILING.json OUT.npz

Two kinds of pair come free:
- **Different things**: two regions of one picture. The region finder draws
  one region per thing, so they are two things -- and the two armchairs are
  in one picture often, which is exactly the pair the image vectors cannot
  tell apart.
- **One thing**: two regions of different pictures within VISIT_S of each
  other that a filing (the model's own, never the labels) put in one object.

KISSME (Koestinger et al., 2012): the differences within each kind of pair are
taken as two Gaussians, and the distance is the log of their likelihood ratio,
d(x, y) = (x - y)' (S_one^-1 - S_two^-1) (x - y), in a PCA space of the masked
DINOv2 vector and the colour histogram. Scored on the labels: how well it
separates regions of one labelled object from regions of two, overall and for
the look-alike pairs.
"""
import json
import sys

import numpy as np

import data
import truth

VISIT_S = 120.0
PCA_DIMS = 64
COLOUR_WEIGHT = 0.5
MAX_PAIRS = 200000
SEED = 0


def features(a):
    x = np.hstack([a["vec"], COLOUR_WEIGHT * a["colour"]]).astype(np.float64)
    return x


def pairs(a, filing):
    rng = np.random.default_rng(SEED)
    fr = a["frame"].astype(str)
    order = np.argsort(fr, kind="stable")
    two = []
    bounds = np.flatnonzero(np.r_[True, fr[order][1:] != fr[order][:-1], True])
    for s, e in zip(bounds[:-1], bounds[1:]):
        idx = order[s:e]
        for p in range(len(idx)):
            for q in range(p + 1, len(idx)):
                two.append((idx[p], idx[q]))
    key = np.array([filing.get(int(i)) or "" for i in a["id"]], dtype=object)
    one = []
    by = {}
    for i, k in enumerate(key):
        if k:
            by.setdefault(k, []).append(i)
    for k, rows in by.items():
        rows = np.array(sorted(rows, key=lambda i: a["t"][i]))
        t = a["t"][rows]
        for p in range(len(rows)):
            q = p + 1
            while q < len(rows) and t[q] - t[p] <= VISIT_S:
                if fr[rows[p]] != fr[rows[q]]:
                    one.append((rows[p], rows[q]))
                q += 1
    one, two = np.array(one), np.array(two)
    if len(one) > MAX_PAIRS:
        one = one[rng.choice(len(one), MAX_PAIRS, replace=False)]
    if len(two) > MAX_PAIRS:
        two = two[rng.choice(len(two), MAX_PAIRS, replace=False)]
    return one, two


def fit(x, one, two):
    mu = x.mean(axis=0)
    xc = x - mu
    u, s, vt = np.linalg.svd(xc[np.random.default_rng(SEED).choice(len(x), min(20000, len(x)), replace=False)],
                             full_matrices=False)
    P = vt[:PCA_DIMS].T
    z = xc @ P
    d1 = z[one[:, 0]] - z[one[:, 1]]
    d2 = z[two[:, 0]] - z[two[:, 1]]
    S1 = d1.T @ d1 / len(d1) + 1e-6 * np.eye(PCA_DIMS)
    S2 = d2.T @ d2 / len(d2) + 1e-6 * np.eye(PCA_DIMS)
    M = np.linalg.inv(S1) - np.linalg.inv(S2)
    # Keep it a metric: clip negative eigenvalues.
    w, V = np.linalg.eigh(M)
    M = (V * np.clip(w, 0, None)) @ V.T
    return {"mu": mu, "P": P, "M": M}


def distance(m, x0, x1):
    z0 = (x0 - m["mu"]) @ m["P"]
    z1 = (x1 - m["mu"]) @ m["P"]
    d = z0 - z1
    return np.einsum("ij,jk,ik->i", d, m["M"], d)


def auc(pos, neg):
    """Probability a same-object pair is nearer than a two-object pair."""
    allv = np.concatenate([pos, neg])
    ranks = allv.argsort().argsort().astype(float)
    rp = ranks[:len(pos)]
    farther = (rp.sum() - len(pos) * (len(pos) - 1) / 2) / (len(pos) * len(neg))
    return float(1.0 - farther)


def transform(m, x):
    """Each region in the learned space, unit length, so that a dot product is
    the learned likeness: the KISSME metric's square root applied to its PCA
    coordinates."""
    w, V = np.linalg.eigh(m["M"])
    L = V * np.sqrt(np.clip(w, 0, None))
    y = ((x - m["mu"]) @ m["P"]) @ L
    n = np.linalg.norm(y, axis=1, keepdims=True)
    return (y / np.where(n > 0, n, 1.0)).astype(np.float32)


def evaluate(a, m):
    t_of = {int(i): float(t) for i, t in zip(a["id"], a["t"])}
    idx = {int(i): k for k, i in enumerate(a["id"])}
    objs = truth.regions_by_object(t_of)
    rows, lab = [], []
    for o, mine in objs.items():
        for r in mine:
            if r in idx and a["has_colour"][idx[r]]:
                rows.append(idx[r])
                lab.append(o)
    rows, lab = np.array(rows), np.array(lab)
    fr = a["frame"].astype(str)[rows]
    x = features(a)[rows]
    iu = np.triu_indices(len(rows), 1)
    same = (lab[:, None] == lab[None, :])[iu]
    other = (fr[:, None] != fr[None, :])[iu]
    i0, i1 = iu
    dl = distance(m, x[i0], x[i1])
    dc = 1.0 - np.sum(a["vec"][rows][i0] * a["vec"][rows][i1], axis=1)
    yl = transform(m, x)
    dl = 1.0 - np.sum(yl[i0] * yl[i1], axis=1)
    res = {}
    res["all, learned"] = auc(dl[same & other], dl[~same])
    res["all, DINOv2 cosine"] = auc(dc[same & other], dc[~same])
    for p, q in [("plain armchair", "throw armchair"), ("red bean bag", "black bean bag"),
                 ("animals painting", "village painting"), ("snowy-tree painting", "animals painting"),
                 ("blue painting", "forest painting")]:
        pair = ((lab[i0] == p) & (lab[i1] == q)) | ((lab[i0] == q) & (lab[i1] == p))
        within = same & other & ((lab[i0] == p) | (lab[i0] == q))
        res["%s / %s, learned" % (p, q)] = auc(dl[within], dl[pair])
        res["%s / %s, DINOv2" % (p, q)] = auc(dc[within], dc[pair])
    return res


def main():
    a = data.load(sys.argv[1])
    filing = {int(k): v for k, v in json.load(open(sys.argv[2])).items()}
    one, two = pairs(a, filing)
    print("pairs: %d of one thing, %d of two" % (len(one), len(two)))
    m = fit(features(a), one, two)
    np.savez(sys.argv[3], **m)
    for k, v in evaluate(a, m).items():
        print("  %-55s %.3f" % (k, v))
    if "--save" in sys.argv:
        a["lvec"] = transform(m, features(a))
        data.save(a, sys.argv[1])
        print("learned vectors saved into", sys.argv[1])


if __name__ == "__main__":
    main()
