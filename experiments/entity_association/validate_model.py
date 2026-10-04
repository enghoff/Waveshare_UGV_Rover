"""Measure appearance ranking with BOTH physical objects excluded from training."""
import argparse
from collections import defaultdict
import json
from pathlib import Path

import numpy as np
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import roc_auc_score

from audit import Evidence


def validate(output, excluded=(), same_person=False):
    evidence = Evidence(excluded=excluded, same_person=same_person)
    ids = sorted({i for i, _, _ in evidence.items})
    index = {i: n for n, i in enumerate(ids)}
    matrices, valid = [], np.ones((len(ids), len(ids)), bool)
    for column in ("dino_blob", "dino_alone_blob", "siglip_blob"):
        vectors = []
        usable = []
        for i in ids:
            blob = evidence.rows[i][column]
            if column == "dino_alone_blob":
                blob = blob or evidence.rows[i]["dino_blob"]
            if not blob:
                raise ValueError(f"observation {i} has no {column}; abstain rather than invent a score")
            v = np.frombuffer(blob, dtype="<f4").astype(float)
            norm = np.linalg.norm(v)
            usable.append(norm > 1e-9)
            vectors.append(v / norm if norm > 1e-9 else v * 0)
        usable = np.array(usable)
        valid &= usable[:, None] & usable[None, :]
        v = np.array(vectors)
        matrices.append(v @ v.T)
    backend = np.array([evidence.rows[i]["vectors_from"] for i in ids])
    valid &= backend[:, None] == backend[None, :]
    pairs = [p for p in evidence.pairs if valid[index[p[0]], index[p[1]]]]
    x = np.array([[m[index[i], index[j]] for m in matrices] for i,j,*_ in pairs])
    y = np.array([same for _,_,same,*_ in pairs], int)
    a = np.array([evidence.object_of[p[4]] for p in pairs])
    b = np.array([evidence.object_of[p[5]] for p in pairs])
    same_frame = np.array([evidence.rows[p[0]]["inference_id"] == evidence.rows[p[1]]["inference_id"]
                           for p in pairs])
    odd_ids = {i for i, _, kind in evidence.items if kind == "odd"}
    odd = np.array([i in odd_ids or j in odd_ids for i,j,*_ in pairs])
    groups = defaultdict(list)
    for k in range(len(pairs)):
        groups[tuple(sorted({a[k], b[k]}))].append(k)
    result = {"labelled_pairs": len(pairs), "same_pairs": int(y.sum()),
              "excluded_observations": evidence.excluded,
              "head_and_body_one_person": same_person,
              "excluded_unusable_pairs": len(evidence.pairs)-len(pairs),
              "same_frame_pairs_excluded_from_training": int(same_frame.sum()), "models": {}}
    for name, columns in (("plain_dino", [0]), ("all_three", [0, 1, 2])):
        predicted = np.empty(len(pairs))
        for n, (held_out, indices) in enumerate(sorted(groups.items()), 1):
            train = ~np.isin(a, held_out) & ~np.isin(b, held_out) & ~same_frame
            assert not (set(a[train]) | set(b[train])) & set(held_out)
            model = LogisticRegression(max_iter=2000).fit(x[train][:, columns], y[train])
            # AUC compares ranking scores; subtract the sampling prior so models
            # trained on different object exclusions use a common evidence scale.
            prior = np.log(y[train].mean() / (1-y[train].mean()))
            predicted[indices] = model.decision_function(x[indices][:, columns]) - prior
            if n % 50 == 0:
                print(f"{name}: {n}/{len(groups)} object-pair holdouts", flush=True)
        known_other = (y == 0) & ~odd
        result["models"][name] = {
            "auc_all": float(roc_auc_score(y, predicted)),
            "auc_same_vs_wrong_in_original_object": float(roc_auc_score(y[(y == 1) | odd],
                                                               predicted[(y == 1) | odd])),
            "auc_same_vs_other_objects": float(roc_auc_score(y[(y == 1) | known_other],
                                                             predicted[(y == 1) | known_other])),
            "holdouts": len(groups)}
        print(name, result["models"][name], flush=True)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--exclude-observation", action="append", type=int, default=[])
    parser.add_argument("--same-person", action="store_true")
    args = parser.parse_args()
    validate(args.output, args.exclude_observation, args.same_person)
