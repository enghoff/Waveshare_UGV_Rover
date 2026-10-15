"""Where a staged thing can be put so that the record-moving step could follow it.

    python move_targets.py WORLD.db GRID.json THINGS.json OUT.png

Written for M5 session 4 (2026-10-15). Session 3's test gave the step nothing it
could act on: the armchair went to a place an older record of it already held,
and the cabinet was never photographed at its old place. This draws, for each
thing to be moved, the floor where `world_state/relocate.py` would not abstain
for a nearer look-alike, and which the session's drives stay clear of.

THINGS.json is {thing: {"at": [x, y], "regions": [ids of labelled crops of it],
"records": [records that may hold it at "at"]}}. A spot is open to a thing when:

- it is 1.0 to 3.0 m from where the thing stands (the pairing takes 0.5 to 4.0);
- no other record stands nearer the spot by `relocate.NEARER_M` than the
  thing's own and looks as much like it -- the step's rival rule, with the
  crops' best cosine against each record's masked exemplars, at least
  `relocate.PAIR_COS` and within `relocate.RIVAL_COS` of the own record's;
- it is free floor with CLEAR_M round it, at least LEG_M from every leg the
  session drives between viewpoints and home, so the rover is never blocked,
  and STOP_M from every viewpoint, whose position check a thing standing
  beside it upsets (session.disagrees);
- it is in the camera's sight, past no wall, from two of the loop's
  living-room viewpoints within VIEW_M, as the pairing needs looks 20 s apart.

The crops stand for the arrival's pictures. A record that holds one of them
would score 1.0 against it, so an exemplar that is one of the crops is left
out of every record's likeness: what is compared is how the records look from
their own pictures.
"""
import base64
import json
import math
import sqlite3
import sys
import zlib

import numpy as np

sys.path.insert(0, __file__.rsplit("experiments", 1)[0])
from world_state import relocate  # noqa: E402

import session  # noqa: E402

WIDTH = 384 * 4
NEAR_M, FAR_M = 1.0, 3.0
CLEAR_M = 0.45
LEG_M = 0.9
STOP_M = 1.2
VIEW_M = 4.0
STEP_M = 0.1
LIVING = (-21.6, -14.0, -18.6, -10.0)


def unit(blob):
    v = np.frombuffer(blob, dtype=np.float32).astype(np.float64)
    n = np.linalg.norm(v)
    return v / n if n else v


def records(db):
    """Every placed record of the map session the store is on, with its
    masked exemplars."""
    session_now = int(dict(db.execute("SELECT key, value FROM meta").fetchall())["map_session"])
    out = {}
    for rid, placement, sess, blob in db.execute(
            "SELECT id, placement_json, placement_map_session, exemplars_alone FROM entities"
            " WHERE placement_json IS NOT NULL"):
        p = json.loads(placement)
        if p.get("x_m") is None or sess != session_now or not blob:
            continue
        kept = np.stack([unit(blob[k:k + WIDTH]) for k in range(0, len(blob) - WIDTH + 1, WIDTH)])
        out[rid] = (p["x_m"], p["y_m"], kept)
    return out


def grid(path):
    g = json.load(open(path))
    cells = np.frombuffer(zlib.decompress(base64.b64decode(g["data"])), dtype=np.int8)
    cells = cells.reshape(g["height"], g["width"])
    return g, cells


def legs():
    """The session's drives as straight segments: home and the loop's
    viewpoints in order, and the drive home from the living room's far end."""
    stops = [session.HOME[:2]] + [v[1:] for v in session.VIEWS]
    out = list(zip(stops, stops[1:]))
    living = [v[1:] for v in session.VIEWS if v[0].startswith("living")]
    out.append((living[-1], session.HOME[:2]))
    return out


def in_sight(free, g, a, b, near_m=0.4):
    """Whether the straight line from a to b crosses no occupied cell, short of
    near_m from b (the thing's own outline)."""
    res, ox, oy = g["resolution_m"], g["origin_x_m"], g["origin_y_m"]
    n = int(math.hypot(b[0] - a[0], b[1] - a[1]) / (res / 2))
    for k in range(n):
        t = k / max(n, 1)
        x, y = a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])
        if math.hypot(b[0] - x, b[1] - y) < near_m:
            break
        if not free[int((y - oy) / res), int((x - ox) / res)]:
            return False
    return True


def likeness(crops, kept):
    """The crops' best cosine against a record's exemplars, leaving out any
    exemplar that is one of the crops. A record whose every exemplar is one of
    them is made of the thing's own pictures, and is as alike as can be."""
    sims = crops @ kept.T
    own = (sims > 0.999).any(axis=0)
    if own.all():
        return 1.0
    return float(sims[:, ~own].max())


def seg_gap(p, a, b):
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    t = max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(p[0] - ax - t * dx, p[1] - ay - t * dy)


def main(world, grid_path, things_path, out_png):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from scipy import ndimage

    db = sqlite3.connect("file:%s?mode=ro" % world, uri=True)
    held = records(db)
    things = json.load(open(things_path))
    g, cells = grid(grid_path)
    res, ox, oy = g["resolution_m"], g["origin_x_m"], g["origin_y_m"]
    occupied = cells >= 50
    unknown = cells < 0
    free = ~(occupied | unknown)
    clear = ndimage.distance_transform_edt(free) * res
    views = [v[1:] for v in session.VIEWS if v[0].startswith("living")]
    segments = legs()
    x0, x1, y0, y1 = LIVING
    xs = np.arange(x0, x1, STEP_M)
    ys = np.arange(y0, y1, STEP_M)

    fig, axes = plt.subplots(1, len(things), figsize=(6.5 * len(things), 7.5))
    axes = np.atleast_1d(axes)
    extent = (ox, ox + g["width"] * res, oy, oy + g["height"] * res)
    report = {}
    for ax, (name, thing) in zip(axes, things.items()):
        crops = [unit(r[0]) for r in db.execute(
            "SELECT dino_alone_blob FROM observations WHERE dino_alone_blob IS NOT NULL"
            " AND id IN (%s)" % ",".join(map(str, thing["regions"])))]
        crops = np.stack(crops)
        own_ids = set(thing.get("records") or [])
        mine_cos = max([likeness(crops, held[r][2]) for r in own_ids if r in held], default=1.0)
        alike = []
        for rid, (rx, ry, kept) in held.items():
            cos = likeness(crops, kept)
            if rid not in own_ids and cos >= max(relocate.PAIR_COS, mine_cos - relocate.RIVAL_COS):
                alike.append((rid, rx, ry, cos))
        tx, ty = thing["at"]
        own = [held[r][:2] for r in own_ids if r in held] or [(tx, ty)]
        ok = []
        for x in xs:
            for y in ys:
                d = math.hypot(x - tx, y - ty)
                if not NEAR_M <= d <= FAR_M:
                    continue
                j = int((x - ox) / res)
                i = int((y - oy) / res)
                if not (0 <= i < cells.shape[0] and 0 <= j < cells.shape[1]) or clear[i, j] < CLEAR_M:
                    continue
                if min(seg_gap((x, y), a, b) for a, b in segments) < LEG_M:
                    continue
                if min(math.hypot(x - vx, y - vy) for vx, vy in views) < STOP_M:
                    continue
                if sum(math.hypot(x - vx, y - vy) <= VIEW_M and in_sight(free, g, (vx, vy), (x, y))
                       for vx, vy in views) < 2:
                    continue
                mine = min(math.hypot(x - ax_, y - ay_) for ax_, ay_ in own)
                rivals = [r for r in alike if math.hypot(x - r[1], y - r[2]) < mine - relocate.NEARER_M]
                if rivals:
                    continue
                ok.append((round(float(x), 2), round(float(y), 2), round(d, 2),
                           round(float(clear[i, j]), 2)))
        report[name] = {"own_likeness": round(mine_cos, 3), "rivals": len(alike), "open_spots": len(ok),
                        "roomiest": sorted(ok, key=lambda s: -s[3])[:8]}
        ax.imshow(np.where(occupied, 0.0, np.where(unknown, 0.6, 1.0)), cmap="gray",
                  origin="lower", extent=extent, vmin=0, vmax=1)
        for a, b in segments:
            ax.plot([a[0], b[0]], [a[1], b[1]], color="tab:blue", lw=1)
        ax.scatter([v[0] for v in views], [v[1] for v in views], marker="^", color="tab:blue", s=40)
        ax.scatter([r[1] for r in alike], [r[2] for r in alike], color="tab:red", s=12,
                   label="rivals: look as alike as its own (%d)" % len(alike))
        if ok:
            ax.scatter([s[0] for s in ok], [s[1] for s in ok], color="tab:green", s=10, marker="s",
                       label="open spots (%d)" % len(ok))
        ax.scatter([tx], [ty], color="black", marker="*", s=160, label="where it stands")
        ax.set_xlim(x0, x1)
        ax.set_ylim(y0, y1)
        ax.set_aspect("equal")
        ax.set_title(name)
        ax.legend(loc="lower left", fontsize=8)
    fig.tight_layout()
    fig.savefig(out_png, dpi=110)
    print(json.dumps(report, indent=1))


if __name__ == "__main__":
    main(*sys.argv[1:5])
