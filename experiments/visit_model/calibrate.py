"""What a likeness and a place are worth, measured on the development labels.

    python calibrate.py LOOKS.npz

1. Likeness: masked DINOv2 cosine between labelled regions of one object in
   different pictures, against regions of two different labelled objects,
   as quantiles and a likelihood ratio per band.
2. Places: for each object that stayed put, every labelled region's place
   (depth outline, depth box, wall, lidar) against the object's median place,
   by source and range.
Development sessions only: the test's labels are not looked at.
"""
import sys

import numpy as np

import data
import model
import truth


def main():
    a = data.load(sys.argv[1])
    L = model.Looks(a)
    ss = truth.sessions()
    dev = {s.name for s in ss if s.role == "development"}
    t_of = {int(i): float(t) for i, t in zip(a["id"], a["t"])}
    idx = {int(i): k for k, i in enumerate(a["id"])}
    objs = truth.regions_by_object(t_of, ss)
    rows, labels = [], []
    for o, mine in objs.items():
        for r, s in mine.items():
            if s in dev and r in idx:
                rows.append(idx[r])
                labels.append(o)
    rows = np.array(rows)
    labels = np.array(labels)
    V = a["vec"][rows]
    S = V @ V.T
    fr = L.frame[rows]
    iu = np.triu_indices(len(rows), 1)
    same = (labels[:, None] == labels[None, :])[iu]
    other_frame = (fr[:, None] != fr[None, :])[iu]
    cos = S[iu]
    s_cos, d_cos = cos[same & other_frame], cos[~same]
    print("likeness, same object (%d pairs): %s" % (len(s_cos), np.round(np.percentile(s_cos, [5, 10, 25, 50, 75]), 3)))
    print("likeness, two objects (%d pairs): %s" % (len(d_cos), np.round(np.percentile(d_cos, [50, 90, 95, 99, 99.9]), 3)))
    edges = np.arange(0.3, 1.01, 0.05)
    print("band       same   diff   LR (pair base rates removed)")
    for lo, hi in zip(edges[:-1], edges[1:]):
        ps = np.mean((s_cos >= lo) & (s_cos < hi))
        pd = np.mean((d_cos >= lo) & (d_cos < hi))
        print("%.2f-%.2f  %.3f  %.4f  %s" % (lo, hi, ps, pd, ("%.1f" % (ps / pd)) if pd > 0 else "inf"))
    # The look-alikes: which pairs of objects come closest.
    print("closest two-object pairs (99th percentile of their likeness):")
    names = sorted(set(labels))
    out = []
    for i, p in enumerate(names):
        for q in names[i + 1:]:
            m = S[np.ix_(labels == p, labels == q)]
            out.append((float(np.percentile(m, 99)), p, q))
    for v, p, q in sorted(out, reverse=True)[:8]:
        print("   %.3f  %s / %s" % (v, p, q))
    # Places.
    print("places against the object's median, stayed-put objects:")
    stay = set(truth.stayed_put())
    src = np.array([("wall" if L.placed[i] and str(a["place_from"][i]) not in ("outline", "box")
                     else str(a["place_from"][i])) for i in rows])
    for kind in ("outline", "box", "wall", "lidar", "recording lidar"):
        res, rng = [], []
        for o in stay:
            m = (labels == o)
            pts = rows[m & np.isfinite(L.a["px"][rows])]
            if len(pts) < 3:
                continue
            mx, my = np.median(L.a["px"][pts]), np.median(L.a["py"][pts])
            sel = rows[m & (src == kind) & np.isfinite(L.a["px"][rows])]
            res += list(np.hypot(L.a["px"][sel] - mx, L.a["py"][sel] - my))
            rng += list(L.a["range"][sel])
        if res:
            res, rng = np.array(res), np.array(rng)
            near, far = rng < 2.5, rng >= 2.5
            print("   %-16s n=%4d  median %.2f  90th %.2f   under 2.5 m: %.2f/%.2f   over: %s" % (
                kind, len(res), np.median(res), np.percentile(res, 90),
                np.median(res[near]) if near.any() else np.nan,
                np.percentile(res[near], 90) if near.any() else np.nan,
                ("%.2f/%.2f" % (np.median(res[far]), np.percentile(res[far], 90))) if far.any() else "-"))


if __name__ == "__main__":
    main()
