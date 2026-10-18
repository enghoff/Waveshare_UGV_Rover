"""Can a record's pictures say it is not an object? (2026-10-18: no, not well
enough.) A logistic regression on the mean of a record's regions' DINOv2 and
SigLIP2 vectors, labelled thing / fixture against floor, wall, doorway,
person, mixed; frozen, then judged on records it had not seen.

    python objectness.py FEATURES.npz LABELS.json TESTS.json

FEATURES is record_features.py's output over every record ("all"); LABELS is
labels/records-1018.json; TESTS is labels/objectness-tests-1018.json, which
records which records each round flagged and which unflagged ones were drawn
at random, before any was labelled. Round 1 trains on the 149 records the
planner could aim at, its cut the lowest that flagged no object held out one
at a time; round 2 on those and round 1's test, its cut the lowest at which
objects are at most 5% of the flagged in 10-fold cross-validation. Each is
judged by the rules set before its test was labelled: at most 1 in 10 of the
records it flags an object, and at least half of the non-objects caught,
estimated from the random draw.
"""
import json
import sys

import numpy as np
from sklearn.linear_model import LogisticRegression
from sklearn.model_selection import LeaveOneOut, StratifiedKFold, cross_val_predict

NON = {"floor", "wall", "doorway", "person", "mixed"}


def fit(X, y, C):
    return LogisticRegression(C=C, max_iter=5000, class_weight="balanced").fit(X, y)


def judge(name, scores, cut, test, labels):
    flagged = [i for i in test["flagged"]]
    objects = [i for i in flagged if labels[i] not in NON]
    sample = test["unflagged_sample"]
    missed = sum(1 for i in sample if labels[i] in NON) / len(sample) * test["unflagged_total"]
    caught = len(flagged) - len(objects)
    print("%s: cut %.2f flags %d unseen records, %d of them objects (%.0f%%); catches about %.0f%% of the non-objects"
          % (name, cut, len(flagged), len(objects), 100.0 * len(objects) / max(1, len(flagged)),
             100.0 * caught / max(1e-9, caught + missed)))
    again = sorted(i for i, s in scores.items() if s >= cut)
    if again != sorted(flagged):
        print("   (refitted here it flags a different set: %d against %d)" % (len(again), len(flagged)))


def main():
    f = dict(np.load(sys.argv[1], allow_pickle=True))
    sets = json.load(open(sys.argv[2]))["sets"]
    tests = json.load(open(sys.argv[3]))
    ids = [str(i) for i in f["id"]]
    X = np.c_[f["dino"], f["siglip"]]
    row = {i: k for k, i in enumerate(ids)}

    def xy(labels):
        keys = [i for i in labels if i in row]
        return keys, X[[row[i] for i in keys]], np.array([labels[i] in NON for i in keys], int)

    one = dict(sets["eligible"])
    keys, X1, y1 = xy(one)
    p = cross_val_predict(LogisticRegression(C=1.0, max_iter=5000, class_weight="balanced"), X1, y1,
                          cv=LeaveOneOut(), method="predict_proba")[:, 1]
    cut1 = round(float(p[y1 == 0].max()) + 0.01, 2)
    m1 = fit(X1, y1, 1.0)
    s1 = {i: float(q) for i, q in zip(ids, m1.predict_proba(X)[:, 1]) if i not in one}
    judge("round 1", s1, cut1, tests["round 1"], sets["blind test 1"])

    two = {**one, **sets["blind test 1"]}
    keys, X2, y2 = xy(two)
    best = None
    for C in (0.3, 1.0, 3.0, 10.0):
        p = np.mean([cross_val_predict(LogisticRegression(C=C, max_iter=5000, class_weight="balanced"), X2, y2,
                                       cv=StratifiedKFold(10, shuffle=True, random_state=s),
                                       method="predict_proba")[:, 1] for s in range(5)], 0)
        for cut in np.arange(0.40, 0.99, 0.01):
            fl = p >= cut
            if fl.sum() and (fl & (y2 == 0)).sum() / fl.sum() <= 0.05:
                break
        caught = (fl & (y2 == 1)).sum() / y2.sum()
        if best is None or caught > best[2]:
            best = (C, round(float(cut), 2), caught)
    m2 = fit(X2, y2, best[0])
    s2 = {i: float(q) for i, q in zip(ids, m2.predict_proba(X)[:, 1]) if i not in two}
    judge("round 2", s2, best[1], tests["round 2"], sets["blind test 2"])


if __name__ == "__main__":
    main()
