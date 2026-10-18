"""Will a look land on its target's record? M10's predictor, developed on
history and judged against the rule the scorer uses.

    python predict.py FEATURES.jsonl [MODEL_OUT.json]

Split by time (M10 criterion 1): trained on looks before SPLIT, judged on the
looks from SPLIT on, and again leaving out one day at a time. Judged by the
Brier score, with log loss and the area under the ROC curve beside it, against
two baselines (criterion 2): the training share that landed, as a constant,
and the scorer's rule since 2026-10-18 (half, for a record seen on six visits
or more, none otherwise). Calibration on the held-out looks is reported in
bins of the prediction (criterion 5). The model is a logistic regression on a
handful of features, small enough to carry to the rover as numbers in a file.
"""
import collections
import json
import math
import sys
import time

import numpy as np
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import brier_score_loss, log_loss, roc_auc_score

SPLIT = time.mktime((2026, 10, 10, 0, 0, 0, 0, 0, -1))
FEATURES = ("log_visits", "spread_m", "rival", "crowd", "nearest_m", "range_m", "needs_movement",
            "log_since_h", "unseen_3h")


def x_of(r):
    return [math.log1p(r["visits"]), r["spread_m"], float(r["rival"]), float(r["crowd"]),
            r["nearest_m"], float(r["range_m"] or 0.0), float(bool(r["needs_movement"])),
            math.log1p(r["since_h"]), float(r["since_h"] >= 3.0)]


def scorer_rule(r):
    return 0.5 if r["visits"] >= 6 else 0.0


def fit(rows):
    X = np.array([x_of(r) for r in rows])
    y = np.array([r["lands"] for r in rows], int)
    mu, sd = X.mean(axis=0), X.std(axis=0) + 1e-9
    m = LogisticRegression(C=1.0, max_iter=1000).fit((X - mu) / sd, y)
    return {"features": list(FEATURES), "mean": mu.tolist(), "scale": sd.tolist(),
            "coef": m.coef_[0].tolist(), "intercept": float(m.intercept_[0])}


def prob(model, r):
    z = model["intercept"] + sum(c * (v - mu) / s for c, v, mu, s in zip(
        model["coef"], x_of(r), model["mean"], model["scale"]))
    return 1.0 / (1.0 + math.exp(-z))


def judge(name, p, y):
    p = np.clip(np.array(p, float), 1e-3, 1 - 1e-3)
    auc = roc_auc_score(y, p) if 0 < sum(y) < len(y) else float("nan")
    print("  %-26s Brier %.3f  log loss %.3f  AUC %.2f" % (name, brier_score_loss(y, p), log_loss(y, p, labels=[0, 1]), auc))


def calibration(p, y, edges=(0, .1, .2, .3, .5, 1.0001)):
    for lo, hi in zip(edges, edges[1:]):
        sel = [(pp, yy) for pp, yy in zip(p, y) if lo <= pp < hi]
        if sel:
            print("    predicted %.1f-%.1f: %3d looks, predicted mean %.2f, landed %.2f" % (
                lo, min(hi, 1), len(sel), np.mean([s[0] for s in sel]), np.mean([s[1] for s in sel])))


def main():
    rows = sorted((json.loads(l) for l in open(sys.argv[1])), key=lambda r: r["t_look"])
    train = [r for r in rows if r["t_look"] < SPLIT]
    test = [r for r in rows if r["t_look"] >= SPLIT]
    y = [int(r["lands"]) for r in test]
    print("train %d looks (%d landed), test %d (%d landed)" % (
        len(train), sum(r["lands"] for r in train), len(test), sum(y)))
    model = fit(train)
    base = float(np.mean([r["lands"] for r in train]))
    p_model = [prob(model, r) for r in test]
    print("held out, from %s on:" % time.strftime("%m-%d", time.localtime(SPLIT)))
    judge("constant %.2f" % base, [base] * len(test), y)
    judge("scorer's rule", [scorer_rule(r) for r in test], y)
    judge("learned", p_model, y)
    calibration(p_model, y)
    print("  coefficients (standardised):", {f: round(c, 2) for f, c in zip(FEATURES, model["coef"])},
          "intercept %.2f" % model["intercept"])

    days = sorted({time.strftime("%m-%d", time.localtime(r["t_look"])) for r in rows})
    out = collections.defaultdict(list)
    for d in days:
        tr = [r for r in rows if time.strftime("%m-%d", time.localtime(r["t_look"])) != d]
        te = [r for r in rows if time.strftime("%m-%d", time.localtime(r["t_look"])) == d]
        m = fit(tr)
        b = float(np.mean([r["lands"] for r in tr]))
        for r in te:
            out["y"].append(int(r["lands"]))
            out["learned"].append(prob(m, r))
            out["constant"].append(b)
            out["rule"].append(scorer_rule(r))
    print("leaving out one day at a time (%d days, %d looks):" % (len(days), len(out["y"])))
    judge("constant", out["constant"], out["y"])
    judge("scorer's rule", out["rule"], out["y"])
    judge("learned", out["learned"], out["y"])
    calibration(out["learned"], out["y"])
    if len(sys.argv) > 2:
        final = fit(rows)
        final["trained_on"] = len(rows)
        json.dump(final, open(sys.argv[2], "w"), indent=1)
        print("model on every look written to", sys.argv[2])


if __name__ == "__main__":
    main()
