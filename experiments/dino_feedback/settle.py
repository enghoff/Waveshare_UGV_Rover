"""Let the region model's mask and DINOv2's patch tokens settle on one answer.

The rover runs the two one way: YOLOE proposes a region and a mask, DINOv2 embeds
the crop, and nothing DINOv2 sees goes back to the mask. This is the feedback
step, written as a two-class fit in DINOv2's patch-feature space with the mask as
the prior -- GrabCut's loop (Rother, Kolmogorov & Blake, 2004), with DINOv2
features where GrabCut had colour.

Each round:

1. the object is the mean of the patch tokens the current assignment calls
   object, and the background the mean of the rest of the crop;
2. every patch is reassigned by how much more it resembles the object than the
   background, added to what the mask said about it.

The mask is a prior, not a starting point to be forgotten: a patch the mask is
sure of needs strong evidence to leave, which is what stops the loop drifting
onto a wall that fills most of a loose crop. What comes out is the agreed
assignment and the object's embedding under it.

The numbers below were fixed before any labelled look was scored, and are not to
be tuned on the labels.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

#: How many units of log-odds one unit of cosine evidence is worth. Patch
#: tokens of one object and of its surroundings differ in cosine by roughly a
#: tenth to a third, so at 0.1 a clear patch moves one to three units -- enough
#: to overturn a mask cell of share 0.5 to 0.95, and not enough to overturn a
#: fully covered one against weak evidence.
TEMPERATURE = 0.1
#: Rounds before giving up on convergence; the change test below usually ends it.
ROUNDS = 10
CONVERGED = 1e-3
#: The mask's share is clipped before it becomes a prior, so no cell is certain.
PRIOR_FLOOR, PRIOR_CEILING = 0.05, 0.95
#: A cell counts as picture rather than letterbox padding at this share.
REAL = 0.5
#: Fewer object cells than this and there is nothing to embed.
MIN_CELLS = 1.0


@dataclass
class Settled:
    embedding: np.ndarray | None   # the object, under the agreed assignment
    pooled: np.ndarray | None      # the object under the mask alone (round 0)
    r: np.ndarray                  # each cell's agreed probability of being object
    rounds: int


def _unit(v):
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-9 else v


def settle(patches, share, real, temperature=TEMPERATURE, rounds=ROUNDS) -> Settled:
    """`patches` is DINOv2's 256 patch tokens; `share` and `real` each cell's share
    of mask and of picture."""
    f = np.asarray(patches, dtype=np.float32)
    f = f / np.maximum(np.linalg.norm(f, axis=1, keepdims=True), 1e-9)
    valid = (np.asarray(real) >= REAL).astype(np.float32)
    prior = np.clip(np.asarray(share, dtype=np.float32), PRIOR_FLOOR, PRIOR_CEILING)
    logit_prior = np.log(prior / (1.0 - prior))
    r = np.asarray(share, dtype=np.float32) * valid
    if r.sum() < MIN_CELLS:
        return Settled(None, None, r, 0)
    pooled = _unit((r[:, None] * f).sum(0))
    done = 0
    for done in range(1, rounds + 1):
        fg = _unit((r[:, None] * f).sum(0))
        back = (1.0 - r) * valid
        if back.sum() >= MIN_CELLS:
            bg = _unit((back[:, None] * f).sum(0))
            evidence = (f @ fg - f @ bg) / temperature
        else:                       # nothing in the crop to contrast against
            evidence = np.zeros(len(f), np.float32)
        new = valid / (1.0 + np.exp(-(evidence + logit_prior)))
        if new.sum() < MIN_CELLS:   # the loop talked itself out of the object
            break
        change = float(np.abs(new - r).max())
        r = new
        if change < CONVERGED:
            break
    return Settled(_unit((r[:, None] * f).sum(0)), pooled, r, done)


def refined_pixels(piece, r, real, keep_at=0.5, least=0.05):
    """The rover's mask, trimmed to the cells the settled assignment keeps.

    Trims only: a cell the loop adds is a 14-pixel block of the letterboxed crop,
    far coarser than the mask's own edge, so growth is not put back into pixels.
    If the trim would leave less than `least` of the mask, the mask is returned
    unchanged rather than an empty picture.
    """
    height, width = piece.shape[:2]
    if piece.size == 0 or not piece.any():
        return piece
    size = int(round(np.sqrt(len(r)))) * 14
    scale = min(size / width, size / height)
    new_w, new_h = int(round(width * scale)), int(round(height * scale))
    top, left = (size - new_h) // 2, (size - new_w) // 2
    grid = np.kron(np.asarray(r).reshape(size // 14, size // 14), np.ones((14, 14)))
    inner = grid[top:top + new_h, left:left + new_w]
    import cv2
    back = cv2.resize(inner.astype(np.float32), (width, height), interpolation=cv2.INTER_NEAREST)
    trimmed = piece & (back >= keep_at)
    if trimmed.sum() < least * piece.sum():
        return piece
    return trimmed
