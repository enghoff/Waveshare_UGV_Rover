"""Looks files each without a random 1% of the pictures that hold no labelled
region, for judging honesty over draws rather than one run.

    python draws.py LOOKS.npz OUT_PREFIX [SEEDS]

Writes OUT_PREFIX-drop0.npz ... for seeds 0..SEEDS-1 (5 by default). Labels
and their pictures are kept, so every draw is scored the same way. Version 3's
wrong merges came and went with such draws (0-2 over seven sessions); version
4 is judged on the full run and five draws (2026-10-16).
"""
import json
import os
import sys

import numpy as np

import data
import truth

HERE = os.path.dirname(os.path.abspath(__file__))
SHARE = 0.01


def labelled_ids(a):
    t_of = {int(i): float(t) for i, t in zip(a["id"], a["t"])}
    out = set()
    for per in truth.regions_by_object(t_of).values():
        out |= set(per)
    for per in json.load(open(os.path.join(HERE, "labels", "stayed-put.json")))["objects"].values():
        for ids in per.values():
            out |= set(ids)
    return out


def main():
    looks, prefix = sys.argv[1], sys.argv[2]
    seeds = int(sys.argv[3]) if len(sys.argv) > 3 else 5
    a = data.load(looks)
    labelled = labelled_ids(a)
    fr = a["frame"].astype(str)
    keep_frames = set(fr[[k for k, i in enumerate(a["id"]) if int(i) in labelled]])
    frames = sorted(set(fr) - keep_frames)
    n = int(len(set(fr)) * SHARE)
    for seed in range(seeds):
        rng = np.random.default_rng(seed)
        drop = set(rng.choice(frames, size=n, replace=False))
        keep = ~np.isin(fr, list(drop))
        data.save({f: (v[keep] if hasattr(v, "__len__") and len(v) == len(keep) else v) for f, v in a.items()},
                  "%s-drop%d.npz" % (prefix, seed))
        print("draw %d: %d pictures, %d regions left out" % (seed, len(drop), int((~keep).sum())))


if __name__ == "__main__":
    main()
