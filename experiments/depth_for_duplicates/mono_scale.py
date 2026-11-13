"""Turn Depth Anything's relative disparity into metres, picture by picture, and
measure how good the metres are.

    python mono_scale.py LOOKS.npz MONO.npz OUT.npz [--anchors oak|oak+wall] [--z]

Per picture, inverse range is fitted as an affine function of the model's
disparity under each region's outline, on the regions the depth camera ranged
in that picture (and, with oak+wall, the raised regions placed on a wall from
2 m or more). With --z the fit is to inverse depth along the camera's axis
(range times the cosine of the angle off it), which is what a model trained on
ordinary pictures might be predicting.

Checks, all on readings the fit did not use:
- leave one out among the depth camera's ranges, in pictures with 3 or more;
- wall places, predicted from depth-camera anchors only (mostly far off-axis);
- by angle off the camera's axis.

Writes, for every region the fit can range, `range`, `sigma` (the picture's
leave-one-out spread, floored) and how far it extrapolates.
"""
import os
import sys

import numpy as np

HERE = os.environ["VISIT_MODEL"]
sys.path.insert(0, HERE)
import data  # noqa: E402

MIN_R, MAX_R = 0.3, 8.0
WALL_MIN_M, WALL_AGREE_M = 2.0, 0.6


def fit(d, r, cosang, z):
    """(s, t) with 1/r' = s d + t, r' = r cos (z) or r; None if it cannot."""
    q = 1.0 / (r * cosang if z else r)
    if len(d) < 2 or np.ptp(d) < 1e-3:
        return None
    A = np.vstack([d, np.ones_like(d)]).T
    sol, *_ = np.linalg.lstsq(A, q, rcond=None)
    if len(d) >= 4:  # one robust pass: drop the worst if it is far out
        res = np.abs(A @ sol - q)
        k = int(np.argmax(res))
        if res[k] > 3 * np.median(res) + 0.02:
            keep = np.arange(len(d)) != k
            sol, *_ = np.linalg.lstsq(A[keep], q[keep], rcond=None)
    s, t = sol
    if s <= 0:
        return None
    return s, t


def predict(st, d, cosang, z):
    s, t = st
    q = s * d + t
    with np.errstate(divide="ignore", invalid="ignore"):
        r = np.where(q > 0, 1.0 / q, np.nan)
    return r / cosang if z else r


def stats(err, truth, label):
    err, truth = np.asarray(err), np.asarray(truth)
    if not len(err):
        print("%-46s none" % label)
        return
    a = np.abs(err)
    print("%-46s n=%5d  median |err| %.2f m  within 0.3 m %3.0f%%  within 20%% %3.0f%%  bias %+.2f m" % (
        label, len(a), np.median(a), 100 * np.mean(a <= 0.3), 100 * np.mean(a <= 0.2 * truth), np.median(err)))


def main():
    looks, monop, outp = sys.argv[1:4]
    anchors = "oak+wall" if "oak+wall" in sys.argv else "oak"
    z = "--z" in sys.argv
    a = data.load(looks)
    m = np.load(monop)
    n = len(a["id"])
    row = {int(i): k for k, i in enumerate(a["id"])}
    d = np.full(n, np.nan)
    for i, v in zip(m["id"], m["d50"]):
        if int(i) in row:
            d[row[int(i)]] = v
    src = np.array([str(s) for s in a["place_from"]])
    posed = np.array([p is not None for p in a["pose_from"]])
    oak = posed & np.isin(src, ("outline", "box")) & np.isfinite(a["range"]) & np.isfinite(d)
    wall = posed & np.isfinite(a["wx"]) & (np.nan_to_num(a["wr"]) >= WALL_MIN_M) & ~oak & np.isfinite(d)
    h = (a["bearing"] - (a["heading"] - a["pan"]) + 180.0) % 360.0 - 180.0
    v = a["elevation"] - a["tilt"]
    cosang = np.clip(np.cos(np.radians(np.nan_to_num(h))) * np.cos(np.radians(np.nan_to_num(v))), 0.2, 1.0)
    off = np.degrees(np.arccos(cosang))
    frames = {}
    for k in np.nonzero(np.isfinite(d) & posed)[0]:
        frames.setdefault(a["frame"][k], []).append(k)

    loo_err, loo_true, loo_off = [], [], []
    wall_err, wall_true, wall_off = [], [], []
    out_r = np.full(n, np.nan)
    out_sig = np.full(n, np.nan)
    out_ext = np.full(n, np.nan)
    out_na = np.zeros(n, int)
    for fr, ks in frames.items():
        ks = np.array(ks)
        an = ks[oak[ks]]
        if anchors == "oak+wall":
            wa = ks[wall[ks]]
            ad, ar = np.r_[d[an], d[wa]], np.r_[a["range"][an], a["wr"][wa]]
            ac = np.r_[cosang[an], cosang[wa]]
        else:
            ad, ar, ac = d[an], a["range"][an], cosang[an]
        # leave one out among the depth camera's own ranges
        spread = []
        if len(an) >= 3:
            for j in range(len(an)):
                keep = np.arange(len(ad)) != j
                st = fit(ad[keep], ar[keep], ac[keep], z)
                if st is None:
                    continue
                p = predict(st, ad[j:j + 1], ac[j:j + 1], z)[0]
                if np.isfinite(p):
                    loo_err.append(p - ar[j]); loo_true.append(ar[j]); loo_off.append(off[an[j]])
                    spread.append(abs(p - ar[j]))
        st = fit(ad, ar, ac, z)
        if st is None:
            continue
        # wall places from depth-camera anchors only
        if anchors == "oak":
            wk = ks[wall[ks]]
            if len(wk):
                p = predict(st, d[wk], cosang[wk], z)
                ok = np.isfinite(p)
                wall_err += list(p[ok] - a["wr"][wk][ok]); wall_true += list(a["wr"][wk][ok])
                wall_off += list(off[wk][ok])
        p = predict(st, d[ks], cosang[ks], z)
        lo, hi = ad.min(), ad.max()
        ext = np.maximum(0, np.maximum(lo - d[ks], d[ks] - hi)) / max(hi - lo, 1e-6)
        good = np.isfinite(p) & (p >= MIN_R) & (p <= MAX_R)
        out_r[ks[good]] = p[good]
        out_ext[ks[good]] = ext[good]
        out_na[ks] = len(ad)
        out_sig[ks[good]] = max(0.25, float(np.median(spread))) if spread else np.nan

    print("fit: %s anchors, %s" % (anchors, "inverse axial depth" if z else "inverse range"))
    loo_err, loo_true, loo_off = map(np.array, (loo_err, loo_true, loo_off))
    stats(loo_err, loo_true, "leave one out, depth camera's ranges")
    if anchors == "oak":
        wall_err, wall_true, wall_off = map(np.array, (wall_err, wall_true, wall_off))
        stats(wall_err, wall_true, "wall places, from depth-camera anchors")
        for lo, hi in ((0, 30), (30, 50), (50, 70), (70, 91)):
            k = (wall_off >= lo) & (wall_off < hi)
            stats(wall_err[k], wall_true[k], "   wall places %2d-%2d deg off axis" % (lo, hi))
    tgt = posed & ~oak & ~(np.isfinite(a["wx"]) & (np.nan_to_num(a["wr"]) >= WALL_MIN_M))
    print("regions with no depth or wall place, ranged now: %d of %d with a reading (%d anchors >= 2)" % (
        np.sum(tgt & np.isfinite(out_r)), np.sum(tgt & np.isfinite(d)), np.sum(tgt & (out_na >= 2))))
    np.savez(outp, id=a["id"], range=out_r, sigma=out_sig, ext=out_ext, anchors=out_na, off=off)


if __name__ == "__main__":
    main()
