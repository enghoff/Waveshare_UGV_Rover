"""Independent accounting for the entity experiments; never edits a world store.

The existing labels are development judgments, not independent acceptance truth.
Unknown identities remain unknown. A physical object, including duplicate records,
is held out as a unit. Each scored pair is assigned to exactly one test fold.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from itertools import combinations
import json
from pathlib import Path
import sqlite3

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
RECORDING = ROOT / "captures/2026-10-04-association-likelihood"
LABELS = ROOT / "captures/2026-10-03-merge-review/look_labels.json"
DUPLICATES = (("object:246", "object:351"), ("object:249", "object:340"),
              ("object:383", "object:400"))
CHAIRS = {"object:247", "object:302", "object:350"}


class Evidence:
    def __init__(self, database=RECORDING / "world.db", labels=LABELS, excluded=(),
                 same_person=False):
        self.database = Path(database)
        self.labels = json.loads(Path(labels).read_text())["things"]
        self.excluded = sorted(set(excluded))
        for lab in self.labels.values():
            for key in ("main_looks", "odd"):
                lab[key] = [i for i in lab[key] if i not in self.excluded]
        con = sqlite3.connect(self.database.resolve().as_uri() + "?mode=ro", uri=True)
        con.row_factory = sqlite3.Row
        self.rows = {r["id"]: dict(r) for r in con.execute(
            "SELECT * FROM observations WHERE map_session=67 ORDER BY observed_at,id")}
        con.close()
        duplicate = {a: min(a, b) for pair in DUPLICATES for a, b in (pair, pair[::-1])}
        self.object_of = {t: duplicate.get(t, t) for t in self.labels}
        objects = sorted(set(self.object_of.values()))
        self.fold_of = {o: n % 2 for n, o in enumerate(objects)}
        self.same_person = same_person
        if same_person:
            # Sensitivity hypothesis: head and body are views of one person.
            # Preserve all other folds so this does not reshuffle the experiment.
            self.object_of["object:385"] = self.object_of["object:332"]
            del self.fold_of["object:385"]
        self.items = sorted((i, t, kind) for t, lab in self.labels.items()
                            for kind, key in (("main", "main_looks"), ("odd", "odd"))
                            for i in lab[key])
        assert len({i for i, _, _ in self.items}) == len(self.items), "duplicate label IDs"
        assert all(i in self.rows for i, _, _ in self.items), "label missing from recording"
        self.pairs = []
        for (i, t, kind), (j, u, other) in combinations(self.items, 2):
            if kind == other == "odd":
                continue
            if "odd" in (kind, other):
                if t != u:
                    continue  # We know an odd look is not its original object only.
                same = False
            else:
                if t in CHAIRS and u in CHAIRS:
                    continue  # Physical chair identities have not been labelled.
                same = self.object_of[t] == self.object_of[u]
            anchor = min(self.object_of[t], self.object_of[u])
            self.pairs.append((i, j, same, self.fold_of[anchor], t, u))

    def score(self, owners, fold=None, visible=None):
        owners = {int(k): v for k, v in owners.items()}
        tp = fp = fn = tn = 0
        for i, j, same, assigned, _, _ in self.pairs:
            if fold is not None and fold != assigned:
                continue
            if visible is not None and (i not in visible or j not in visible):
                continue
            together = owners.get(i) is not None and owners.get(i) == owners.get(j)
            tp += same and together
            fn += same and not together
            fp += not same and together
            tn += not same and not together
        by_object = defaultdict(list)
        odd_kept = odd_count = main_count = waiting = 0
        for t, lab in self.labels.items():
            if fold is not None and self.fold_of[self.object_of[t]] != fold:
                continue
            main = [i for i in lab["main_looks"] if visible is None or i in visible]
            odd = [i for i in lab["odd"] if visible is None or i in visible]
            home = Counter(owners.get(i) for i in main if owners.get(i)).most_common(1)
            if home:
                odd_kept += sum(owners.get(i) == home[0][0] for i in odd)
            odd_count += len(odd)
            main_count += len(main)
            waiting += sum(owners.get(i) is None for i in main)
            if t not in CHAIRS:
                by_object[self.object_of[t]].extend(main)
        per_object = {}
        for obj, ids in by_object.items():
            if len(ids) < 2:
                continue
            counts = Counter(owners.get(i) for i in ids if owners.get(i))
            together = sum(n * (n - 1) // 2 for n in counts.values())
            per_object[obj] = {"looks": len(ids), "waiting": sum(owners.get(i) is None for i in ids),
                               "pieces_with_two_looks": sum(n >= 2 for n in counts.values()),
                               "pair_recall": together / (len(ids) * (len(ids) - 1) / 2)}
        precision = tp / (tp + fp) if tp + fp else None
        recall = tp / (tp + fn) if tp + fn else None
        return {"same_together": tp, "same_apart": fn, "different_together": fp,
                "different_apart": tn, "precision": precision, "recall": recall,
                "f1": 2 * tp / (2 * tp + fp + fn) if 2 * tp + fp + fn else None,
                "main_looks": main_count, "main_waiting": waiting,
                "odd_kept_in_original_home": odd_kept, "odd_count": odd_count,
                "objects_scored": len(per_object),
                "objects_split": sum(v["pieces_with_two_looks"] > 1 for v in per_object.values()),
                "macro_object_pair_recall": (sum(v["pair_recall"] for v in per_object.values())
                                              / len(per_object) if per_object else None),
                "per_object": per_object}

    def fit(self, test_fold):
        """Fit only on objects absent from the test fold, including duplicate halves."""
        from sklearn.linear_model import LogisticRegression
        x, y = [], []
        for i, j, same, _, t, u in self.pairs:
            if any(self.fold_of[self.object_of[v]] == test_fold for v in (t, u)):
                continue
            if self.rows[i]["inference_id"] == self.rows[j]["inference_id"]:
                continue
            a, b = self.rows[i], self.rows[j]
            if a["vectors_from"] != b["vectors_from"]:
                continue
            features = []
            for key in ("dino_blob", "dino_alone_blob", "siglip_blob"):
                va, vb = a[key], b[key]
                if key == "dino_alone_blob":
                    va, vb = va or a["dino_blob"], vb or b["dino_blob"]
                if not va or not vb:
                    break
                va, vb = (np.frombuffer(v, dtype="<f4").astype(float) for v in (va, vb))
                if va.shape != vb.shape or min(np.linalg.norm(va), np.linalg.norm(vb)) < 1e-9:
                    break
                features.append(float(va @ vb / (np.linalg.norm(va) * np.linalg.norm(vb))))
            if len(features) == 3:
                x.append(features)
                y.append(int(same))
        y = np.asarray(y)
        model = LogisticRegression(max_iter=2000).fit(np.asarray(x), y)
        prior = np.log(y.mean() / (1 - y.mean()))
        return {"coef": model.coef_[0].tolist(),
                "offset": float(model.intercept_[0] - prior),
                "training_pairs": len(y), "training_same": int(y.sum()),
                "held_out_objects": sorted(o for o, f in self.fold_of.items() if f == test_fold)}

    def audit(self):
        same_frame = []
        for i, j, same, _, t, u in self.pairs:
            if same and self.rows[i]["inference_id"] == self.rows[j]["inference_id"]:
                same_frame.append({"observations": [i, j], "records": [t, u],
                                   "inference": self.rows[i]["inference_id"]})
        old_folds = [set(sorted(self.labels)[::2]), set(sorted(self.labels)[1::2])]
        repeated = sum((t in old_folds[0]) != (u in old_folds[0]) for _, _, _, _, t, u in self.pairs)
        return {"labelled_observations": len(self.items), "canonical_label_groups": len(self.fold_of),
                "excluded_observations": self.excluded,
                "head_and_body_one_person": self.same_person,
                "unique_pairs": len(self.pairs), "unique_same_pairs": sum(p[2] for p in self.pairs),
                "old_cross_fold_pairs_counted_twice": repeated,
                "duplicate_objects_crossing_old_folds": [list(p) for p in DUPLICATES
                    if (p[0] in old_folds[0]) != (p[1] in old_folds[0])],
                "same_frame_same_object": same_frame,
                "non_wall_clock_rows": sum(r["observed_at"] < 1e9 for r in self.rows.values()),
                "limitations": ["393 labels are coding-agent judgments; only 19 were owner-reviewed",
                                "pending/unlabelled objects and misses are not covered",
                                "unknown odd-look identities and individual chairs remain unknown",
                                "pair and per-group scores include a floor/skirting label, not an acceptance object census",
                                "saved replay rescoring does not repair its original training leakage"]}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--exclude-observation", action="append", type=int, default=[])
    args = parser.parse_args()
    evidence = Evidence(excluded=args.exclude_observation)
    result = {"audit": evidence.audit(), "saved_replays": {}}
    for mode in ("none", "end", "300", "60", "q900", "q300", "q900norefit", "idle600"):
        # Old replays trained with entity-record folds. Allocate every pair once by
        # the original anchor fold; these are a diagnostic, not repaired held-out trials.
        old_folds = [set(sorted(evidence.labels)[::2]), set(sorted(evidence.labels)[1::2])]
        old_owner = [json.loads((RECORDING / f"owners_{mode}_{k}.json").read_text()) for k in range(2)]
        totals = Counter()
        for i, j, same, _, t, u in evidence.pairs:
            anchor = min(t, u)
            k = 1 if anchor in old_folds[0] else 0
            owner = old_owner[k]
            together = owner.get(str(i)) is not None and owner.get(str(i)) == owner.get(str(j))
            totals["tp" if same and together else "fn" if same else "fp" if together else "tn"] += 1
        tp, fp, fn = (totals[k] for k in ("tp", "fp", "fn"))
        result["saved_replays"][mode] = {**totals, "precision": tp/(tp+fp), "recall": tp/(tp+fn),
                                          "f1": 2*tp/(2*tp+fp+fn)}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result["audit"], indent=2))
    for mode, score in result["saved_replays"].items():
        print(mode, {k: round(v, 4) if isinstance(v, float) else v for k, v in score.items()})


if __name__ == "__main__":
    main()
