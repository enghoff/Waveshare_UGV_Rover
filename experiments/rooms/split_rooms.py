"""Split one saved occupancy grid into rooms, and say which room each taped thing falls in.

    python experiments/rooms/split_rooms.py GRID.json [--walls WALLS.npz]
        [--radius 0.35 0.45 0.55] [--targets TARGETS.json] [--png OUT.png]

A first look at whether the rover's own 2D map can say which room a thing is
in (docs/plans/rooms-and-things-within-things.md). Rooms are what is left of
the free floor once it is shrunk by half a doorway (`--radius`): each
connected piece of at least `MIN_SEED_M2` is a room's seed, and every free
cell then takes the seed it reaches first through free floor, the flood-fill
Hydra's room step and Bormann et al.'s distance segmentation both use.

GRID.json is a grid as the daemon's `nav_grid` sends it, or as a session
script saved it. With `--walls`, only the walls in that file (cells the
mapper called occupied in every session, `world_state/visit_model/walls.py`)
stand in the way, and anything else the grid calls occupied counts as
floor. TARGETS.json is a taped-things file with a `where` per thing
(`experiments/perception_bench/ground_truth/m4_targets.json`); a thing takes
the room of the nearest roomed cell within `REACH_M`, since paintings and
wardrobes stand on wall or furniture cells.

Nothing here reads the store or touches the rover.
"""
import argparse
import base64
import json
import zlib

import numpy as np
from scipy import ndimage

#: The smallest piece of shrunken floor that seeds a room. Below it, a gap
#: between a chair and a wall would become a room of its own.
MIN_SEED_M2 = 0.5
#: How far from a taped point the nearest room may be.
REACH_M = 0.8


def grid(path):
    g = json.load(open(path))
    cells = np.frombuffer(zlib.decompress(base64.b64decode(g["data"])), dtype=np.int8)
    return g, cells.reshape(g["height"], g["width"])


def split(free, res, radius):
    """Room labels (0 for none) over `free`, and how many rooms."""
    clearance = ndimage.distance_transform_edt(free) * res
    pieces, count = ndimage.label(clearance > radius)
    areas = ndimage.sum(np.ones_like(pieces), pieces, range(1, count + 1)) * res * res
    labels = np.zeros_like(pieces)
    rooms = 0
    for piece, area in enumerate(areas, start=1):
        if area >= MIN_SEED_M2:
            rooms += 1
            labels[pieces == piece] = rooms
    while True:
        grown = ndimage.grey_dilation(labels, size=3)
        newly = free & (labels == 0) & (grown > 0)
        if not newly.any():
            return labels, rooms
        labels[newly] = grown[newly]


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("grid")
    p.add_argument("--walls")
    p.add_argument("--radius", type=float, nargs="+", default=[0.35, 0.45, 0.55])
    p.add_argument("--targets")
    p.add_argument("--png", help="draw the last radius's rooms (+y up)")
    a = p.parse_args()
    g, cells = grid(a.grid)
    res, ox, oy = g["resolution_m"], g["origin_x_m"], g["origin_y_m"]
    known = cells >= 0
    if a.walls:
        w = np.load(a.walls)
        if w["stable"].shape != cells.shape or abs(float(w["ox"]) - ox) > 1e-6:
            raise SystemExit("the walls were drawn on another grid")
        blocked = w["stable"]
    else:
        blocked = cells >= 50
    # One cell of opening closes the lidar's speckle under furniture.
    free = ndimage.binary_opening(known & ~blocked, iterations=1)
    targets = json.load(open(a.targets))["things"] if a.targets else {}
    for radius in a.radius:
        labels, rooms = split(free, res, radius)
        areas = [round(float((labels == r).sum()) * res * res, 1) for r in range(1, rooms + 1)]
        print("radius %.2f m: %d rooms, %s m2" % (radius, rooms, areas))
        ii, jj = np.nonzero(labels)
        xs, ys = ox + (jj + 0.5) * res, oy + (ii + 0.5) * res
        by_room = {}
        for name, t in sorted(targets.items(), key=lambda kv: int(kv[0].lstrip("NT"))):
            d = np.hypot(xs - t["x_m"], ys - t["y_m"])
            k = int(d.argmin())
            room = int(labels[ii[k], jj[k]]) if d[k] <= REACH_M else 0
            by_room.setdefault(t["where"], []).append((name, room))
        for where, got in by_room.items():
            print("  %-24s %s" % (where, " ".join("%s:%d" % one for one in got)))
    if a.png:
        from PIL import Image
        palette = np.array([[230, 159, 0], [86, 180, 233], [0, 158, 115], [240, 228, 66],
                            [0, 114, 178], [213, 94, 0], [204, 121, 167]], np.uint8)
        img = np.full(cells.shape + (3,), 128, np.uint8)
        img[free] = 255
        on = labels > 0
        img[on] = palette[(labels[on] - 1) % len(palette)]
        img[blocked] = 0
        Image.fromarray(img[::-1]).save(a.png)


if __name__ == "__main__":
    main()
