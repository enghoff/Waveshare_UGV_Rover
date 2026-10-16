"""Colour and a frozen learned likeness added to a looks file.

    python extend_looks.py LOOKS.npz COLOUR.npz METRIC.npz

COLOUR is colour.py's output (run on the rover); METRIC a frozen metric.py
fit. The colour arrays are read once: indexing an npz file's member reads and
decompresses the whole member each time, which made the first version of this
step take 20 minutes (2026-10-16).
"""
import sys

import numpy as np

import data
import metric


def main(looks_path, colour_path, metric_path):
    a = data.load(looks_path)
    c = np.load(colour_path)
    ids, colours = c["id"], c["colour"]
    at = {int(i): k for k, i in enumerate(ids)}
    rows = np.array([at.get(int(i), -1) for i in a["id"]])
    have = rows >= 0
    col = np.zeros((len(rows), colours.shape[1]), np.float32)
    col[have] = colours[rows[have]]
    a["colour"], a["has_colour"] = col, have
    m = dict(np.load(metric_path))
    a["lvec"] = metric.transform(m, metric.features(a))
    data.save(a, looks_path)
    print("%d regions, %d coloured, learned likeness from %s" % (len(rows), int(have.sum()), metric_path))


if __name__ == "__main__":
    main(*sys.argv[1:4])
