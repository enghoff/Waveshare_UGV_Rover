"""Prediction errors in a "what" stream and a "where" stream, against memory.

Nothing is learned here and nothing is rendered. The region finder (YOLOE)
parses a look into regions; each region's "what" is its frozen DINOv2 vector
with the background blanked; its "where" is its map bearing and, where the
depth camera measured it, its range. Memory is everything the store held
before a cutoff, frozen.

The prediction for a posed look is the regions of the most recent remembered
looks taken from about the same place facing about the same way (its
references). Every score is graded, because the region finder crops the same
thing differently from look to look and a hard "matched or not" would call one
region in seven a change in an unchanged room.

- **what**: how unlike anything in the house's memory a region is: 1 - its
  best cosine against every remembered region.
- **where**: how much better a region matches something elsewhere in memory
  than anything the references hold where it was seen: (1 - what) - its best
  cosine among reference regions in a consistent place, never below 0. A thing
  that is familiar but not where the latest views had it scores high; a new
  thing scores on what, not here.
- **gone**: for each remembered region the look should have seen, how well
  the references agreed on it minus how well anything in the look matches it
  in a consistent place, never below 0.

A consistent place is a bearing within BEARING_DEG of where the remembered
region should appear from the look's camera (through its map point when it was
ranged) and, when both were ranged, a range within RANGE_M of it.
"""
import numpy as np

#: A reference look stood within this of the look and faced within this of it.
REF_M = 1.0
REF_DEG = 45.0
#: How many of the most recent reference looks make the prediction.
REF_LOOKS = 30
#: A consistent place (set on development; see protocol.md).
BEARING_DEG = 15.0
RANGE_M = 1.0
#: A remembered region is expected in the look if it lies within this of the
#: way the camera faced (the fisheye is 130 degrees wide; its edges are dim and
#: rarely yield a region) and, when ranged, at most this far away.
VIEW_HALF_DEG = 45.0
VIEW_MAX_M = 4.0
#: Two remembered regions from different looks, in one place, at or above this
#: cosine are one expectation, counted once (bookkeeping, not a score).
SAME_THING = 0.5


def _wrap(a):
    return (np.asarray(a) + 180.0) % 360.0 - 180.0


def what_scores(query, memory, chunk=2048):
    """1 - the best cosine of each query region against all of memory."""
    out = np.empty(query.n)
    for i in range(0, query.n, chunk):
        out[i:i + chunk] = 1.0 - (query.vec[i:i + chunk] @ memory.vec.T).max(axis=1)
    return out


class Looks:
    """Regions grouped by the picture they came from, oldest first."""

    def __init__(self, regions):
        frames, first = np.unique(regions.frame, return_index=True)
        order = np.argsort(regions.t[first], kind="stable")
        self.frames = frames[order]
        self.index = {f: np.flatnonzero(regions.frame == f) for f in self.frames}
        rows = np.array([self.index[f][0] for f in self.frames], dtype=int)
        self.t = regions.t[rows]
        self.posed = regions.posed[rows]
        self.cx = regions.cx[rows]
        self.cy = regions.cy[rows]
        self.yaw = regions.yaw[rows]


def references(cam, yaw, memory_looks):
    """Indices into memory_looks of the most recent looks from about here."""
    m = memory_looks
    ok = m.posed & (np.hypot(m.cx - cam[0], m.cy - cam[1]) <= REF_M) \
        & (np.abs(_wrap(m.yaw - yaw)) <= REF_DEG)
    idx = np.flatnonzero(ok)
    return idx[np.argsort(m.t[idx], kind="stable")[::-1][:REF_LOOKS]]


def _expected(cam, regions, rows):
    """Bearing and range at which remembered regions should appear from cam:
    through their map point when it was measured, else their stored bearing
    and no range."""
    b = regions.bearing[rows].copy()
    r = np.full(len(rows), np.nan)
    ranged = regions.ranged[rows]
    dx = regions.px[rows][ranged] - cam[0]
    dy = regions.py[rows][ranged] - cam[1]
    b[ranged] = np.degrees(np.arctan2(dy, dx))
    r[ranged] = np.hypot(dx, dy)
    return b, r


def _consistent(b1, r1, b2, r2):
    """Pairwise: is each of the first set in a consistent place with each of the
    second? Ranges are compared only where both exist."""
    ok = np.abs(_wrap(b1[:, None] - b2[None, :])) <= BEARING_DEG
    both = np.isfinite(r1)[:, None] & np.isfinite(r2)[None, :]
    far = np.abs(np.nan_to_num(r1)[:, None] - np.nan_to_num(r2)[None, :]) > RANGE_M
    return ok & ~(both & far)


def look_scores(rows, query, memory, memory_looks, familiar):
    """where for one posed look's regions, and gone for what it should have seen.

    familiar is 1 - what for these rows. Returns None when the look has no
    reference, else (where per region, [(memory row, gone score), ...])."""
    first = rows[0]
    cam = (query.cx[first], query.cy[first])
    yaw = query.yaw[first]
    refs = references(cam, yaw, memory_looks)
    if len(refs) == 0:
        return None
    ref_rows = np.concatenate([memory_looks.index[memory_looks.frames[i]] for i in refs])
    eb, er = _expected(cam, memory, ref_rows)
    qb = query.bearing[rows]
    qr = np.where(query.ranged[rows], query.range[rows], np.nan)

    sims = query.vec[rows] @ memory.vec[ref_rows].T
    here = _consistent(qb, qr, eb, er)
    held_here = np.where(here, sims, 0.0).max(axis=1)
    where = np.maximum(0.0, familiar - held_here)

    # What the look should have seen: furniture-sized remembered regions in
    # view, each scored by how well the other references agreed on it.
    floor = memory.floor_sized()[ref_rows]
    in_view = (np.abs(_wrap(eb - yaw)) <= VIEW_HALF_DEG) & ~(er > VIEW_MAX_M)
    cand = np.flatnonzero(floor & in_view)
    gone = []
    if len(cand):
        frames = memory.frame[ref_rows]
        rsims = memory.vec[ref_rows[cand]] @ memory.vec[ref_rows].T
        rhere = _consistent(eb[cand], er[cand], eb, er) & (frames[cand][:, None] != frames[None, :])
        agreed = np.where(rhere, rsims, 0.0).max(axis=1)
        qhere = _consistent(eb[cand], er[cand], qb, qr)
        seen = np.where(qhere, sims[:, cand].T, 0.0).max(axis=1)
        score = np.maximum(0.0, agreed - seen)
        # One remembered thing in several references is one expectation: keep
        # the most recent sighting of each group of rows from different looks,
        # in one place, that look alike.
        same = rhere[:, cand] & (rsims[:, cand] >= SAME_THING)
        kept = []
        for i in np.argsort(-memory.t[ref_rows[cand]], kind="stable"):
            if not any(same[i, j] for j in kept):
                kept.append(i)
        gone = [(int(ref_rows[cand[i]]), float(score[i])) for i in kept]
    return where, gone


def run(query, memory):
    """Every query region's what and where scores, and every posed look's gone
    scores, against memory frozen as given."""
    memory_looks = Looks(memory)
    looks = Looks(query)
    what = what_scores(query, memory)
    where = np.full(query.n, np.nan)
    gone = {}
    for f in looks.frames:
        rows = looks.index[f]
        if not query.posed[rows[0]]:
            continue
        out = look_scores(rows, query, memory, memory_looks, 1.0 - what[rows])
        if out is None:
            continue
        where[rows], gone[f] = out
    return {"what": what, "where": where, "gone": gone}


def gone_over_visit(gone, frames):
    """A remembered region is gone from a visit when no look that expected it
    matched it: its smallest gone score over the visit's looks, with how many
    looks expected it. frames are the visit's looks."""
    seen = {}
    for f in frames:
        for row, score in gone.get(f, []):
            low, n = seen.get(row, (np.inf, 0))
            seen[row] = (min(low, score), n + 1)
    return seen
