"""Step 3: the where stream against a memory of things in their latest state,
at measured places, and gone from space seen empty.

What is unchanged from step 1. The differences:

- **Places.** A region stands at its depth-measured point, else at the
  lidar's return across its width (lidar.place). A region with neither has no
  where score.
- **Memory of things.** Every remembered region with a place is a remembered
  thing there. One the lidar hit when it was seen (solid) is retired once the
  latest judged scans before the cutoff mostly see through its place: the
  room's latest state, not every arrangement it has had.
- **where**: (1 - what) minus the best cosine among remembered things still
  standing within PLACE_M of where the region stands, never below 0.
- **gone**: a remembered solid thing still standing at the cutoff whose place
  the visit's scans mostly see through, scored as the share of judged scans
  that pass, with at least GONE_MIN_SCANS judged.
"""
import numpy as np

import lidar
import streams

#: A remembered thing explains a region within this of where the region
#: stands (set on development; see step3.md).
PLACE_M = 0.6
#: The latest state of a remembered place: its newest this-many judged scans
#: before the cutoff, retired when more than half pass through it.
LATEST_SCANS = 20
RETIRE_SHARE = 0.5
MIN_JUDGED = 5
#: Gone needs at least this many judged scans in the visit.
GONE_MIN_SCANS = 5


def places(regions, recs):
    """(x, y, source) per region: depth, lidar or none."""
    px, py = regions.px.copy(), regions.py.copy()
    src = np.where(regions.ranged, "depth", "none").astype(object)
    for i in np.flatnonzero(regions.posed & ~regions.ranged):
        p = lidar.place(recs, regions.shutter[i], regions.bearing[i], regions.span[i])
        if p is not None:
            px[i], py[i], src[i] = p[0], p[1], "lidar"
    return px, py, src


class Things:
    """The memory of things: remembered regions with places, and which still
    stand at the cutoff."""

    def __init__(self, memory, recs, cutoff):
        self.memory = memory
        px, py, src = places(memory, recs)
        rows = np.flatnonzero(src != "none")
        self.rows, self.x, self.y = rows, px[rows], py[rows]
        self.vec = memory.vec[rows]
        self.t = memory.shutter[rows]
        self.floor = memory.floor_sized()[rows]
        self.solid = np.array([lidar.solid(recs, t, x, y)
                               for t, x, y in zip(self.t, self.x, self.y)], bool)
        s = np.flatnonzero(self.solid)
        passes, hits = lidar.outcomes(recs, self.x[s], self.y[s], -np.inf, cutoff,
                                      after=self.t[s], newest_first=True, limit=LATEST_SCANS)
        judged = passes + hits
        self.retired = np.zeros(len(rows), bool)
        self.retired[s] = (judged >= MIN_JUDGED) & (passes > RETIRE_SHARE * judged)
        self.standing = ~self.retired


def where_scores(query, qx, qy, familiar, things):
    """familiar minus the best cosine of a standing thing within PLACE_M."""
    out = np.full(query.n, np.nan)
    sx, sy = things.x[things.standing], things.y[things.standing]
    sv = things.vec[things.standing]
    for i in np.flatnonzero(np.isfinite(qx)):
        near = np.hypot(sx - qx[i], sy - qy[i]) <= PLACE_M
        held = float((sv[near] @ query.vec[i]).max()) if near.any() else 0.0
        out[i] = max(0.0, familiar[i] - held)
    return out


def gone_scores(things, recs, start, end):
    """Share of the visit's judged scans that see through each standing solid,
    furniture-sized thing; NaN where fewer than GONE_MIN_SCANS were judged."""
    cand = np.flatnonzero(things.standing & things.solid & things.floor)
    passes, hits = lidar.outcomes(recs, things.x[cand], things.y[cand], start, end)
    judged = passes + hits
    share = np.where(judged >= GONE_MIN_SCANS, passes / np.maximum(judged, 1), np.nan)
    return cand, share, judged


def run(query, memory, recs, cutoff, start, end):
    things = Things(memory, recs, cutoff)
    what = streams.what_scores(query, memory)
    qx, qy, qsrc = places(query, recs)
    where = where_scores(query, qx, qy, 1.0 - what, things)
    cand, share, judged = gone_scores(things, recs, start, end)
    return {"what": what, "where": where, "qx": qx, "qy": qy, "qsrc": qsrc,
            "things": things, "gone_rows": cand, "gone": share, "gone_judged": judged}
