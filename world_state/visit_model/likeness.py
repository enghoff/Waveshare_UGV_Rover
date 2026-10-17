"""The likeness learned from the rover's own pictures, as version 4 applies it:
each region's masked DINOv2 vector and colour histogram mapped into the space
of `metric-1015.npz`, where a dot product is the likeness.

`features` and `transform` are experiments/visit_model/metric.py's (branch
visit-model, frozen with version 4 at cabe003), without the fitting and the
scoring, which read the labels and stay on the desk.
"""
import numpy as np

COLOUR_WEIGHT = 0.5


def features(a):
    x = np.hstack([a["vec"], COLOUR_WEIGHT * a["colour"]]).astype(np.float64)
    return x


def transform(m, x):
    """Each region in the learned space, unit length, so that a dot product is
    the learned likeness: the KISSME metric's square root applied to its PCA
    coordinates."""
    w, V = np.linalg.eigh(m["M"])
    L = V * np.sqrt(np.clip(w, 0, None))
    y = ((x - m["mu"]) @ m["P"]) @ L
    n = np.linalg.norm(y, axis=1, keepdims=True)
    return (y / np.where(n > 0, n, 1.0)).astype(np.float32)
