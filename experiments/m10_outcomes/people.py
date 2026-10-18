"""The person model in world_state/visit_model/person.json, fitted and judged
again from the labels.

    python people.py FEATURES.npz LABELS.json TESTS.json [PERSON.json]

FEATURES is record_features.py's output over every record; LABELS is
labels/records-1018.json; TESTS is labels/people-test-1018.json. The labelled
records are split before anything is fitted: those whose id's sha256 starts
below 8 are development, the rest held out. On development a logistic
regression on the mean DINOv2 and SigLIP2 vectors, against "person", has its C
and cut chosen by cross-validation (the lowest cut at which non-people are at
most 5% of what it flags); fitted on all of development, it is judged once on
the held-out half by the rules set before: at most 1 in 10 of what it flags not
a person, at least half the people caught. Then the records nobody had
labelled: those it flags, and 50 of the rest at random, labelled blind.
"""
import hashlib
import json
import sys

import numpy as np
from sklearn.linear_model import LogisticRegression
from sklearn.model_selection import StratifiedKFold, cross_val_predict


def cut_for(p, y):
    for cut in np.sort(np.unique(np.round(p, 4))):
        fl = p >= cut
        if fl.sum() and (fl & (y == 0)).sum() / fl.sum() <= 0.05:
            return float(cut)
    return 1.0


def main():
    f = dict(np.load(sys.argv[1], allow_pickle=True))
    sets = json.load(open(sys.argv[2]))["sets"]
    tests = json.load(open(sys.argv[3]))
    lab = {}
    for name in ("eligible", "blind test 1", "blind test 2"):
        lab.update(sets[name])
    ids = [str(i) for i in f["id"]]
    row = {i: k for k, i in enumerate(ids)}
    X = np.c_[f["dino"], f["siglip"]]
    keys = [i for i in lab if i in row]
    dev = [i for i in keys if int(hashlib.sha256(i.encode()).hexdigest()[0], 16) < 8]
    held = [i for i in keys if i not in dev]
    yd = np.array([lab[i] == "person" for i in dev], int)
    Xd = X[[row[i] for i in dev]]
    best = None
    for C in (0.3, 1.0, 3.0, 10.0):
        p = np.mean([cross_val_predict(LogisticRegression(C=C, max_iter=5000, class_weight="balanced"), Xd, yd,
                                       cv=StratifiedKFold(5, shuffle=True, random_state=s),
                                       method="predict_proba")[:, 1] for s in range(5)], 0)
        cut = cut_for(p, yd)
        caught = int(((p >= cut) & (yd == 1)).sum())
        if best is None or caught > best[2]:
            best = (C, round(cut, 3), caught)
    C, cut, _ = best
    m = LogisticRegression(C=C, max_iter=5000, class_weight="balanced").fit(Xd, yd)
    frozen = json.load(open(sys.argv[4])) if len(sys.argv) > 4 else None
    if frozen is not None:
        same = np.allclose(np.round(m.coef_[0], 7), frozen["coef"]) and cut == frozen["threshold"]
        print("refitted as frozen in person.json:", same)
    p = m.predict_proba(X)[:, 1]
    fl = [i for i in held if p[row[i]] >= cut]
    people = [i for i in held if lab[i] == "person"]
    print("development %d records (%d people): C %s, cut %.3f" % (len(dev), yd.sum(), C, cut))
    print("held out %d records (%d people): flags %d, %d not people; catches %d"
          % (len(held), len(people), len(fl), sum(1 for i in fl if lab[i] != "person"),
             sum(1 for i in fl if lab[i] == "person")))
    blind = tests["labels"]
    print("unlabelled: %d flagged; of %d drawn at random from the rest, %d people"
          % (len(tests["flagged"]), len(tests["unflagged_sample"]),
             sum(1 for i in tests["unflagged_sample"] if blind[i] == "person")))


if __name__ == "__main__":
    main()
