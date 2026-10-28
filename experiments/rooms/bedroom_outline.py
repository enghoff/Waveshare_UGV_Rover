"""Draw the held-out bedroom's outline on the map, for Perception's recordings.

    python experiments/rooms/bedroom_outline.py OUT.png OUT.json

The bedroom is held out of Perception's training
(docs/decisions/the-bedroom-is-held-out-for-perception.md), and every
session's manifest carries its outline so that a pose inside it can be found.
This draws it on map 7da19bef3888 from the room split of 2026-10-19
(split_rooms.py): the saved grid split at a half-doorway of 0.55 m, where the
bedroom came out whole; the region holding the owner's five bedroom things,
cut at the bedroom door and grown by MARGIN_M so the walls and furniture round
its floor are inside; traced as one polygon in map metres. The picture is the
grid with the outline over it (+y up), for checking by eye.

The outline is a floor plan of a room, so it goes in tmp/ or straight into a
manifest, not into the repository. Redraw it with a new grid if the map
changes.
"""
import json
import os
import sys

import cv2
import numpy as np
from scipy import ndimage

import split_rooms

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
#: The grid the M5 session of 2026-10-15 saved, on the map the rover still uses.
GRID = os.path.join(ROOT, "tmp", "session3-1015", "session3-1015-grid.json")
TARGETS = os.path.join(ROOT, "experiments", "perception_bench", "ground_truth", "m4_targets.json")
MAP_ID = "7da19bef3888"
#: The half-doorway at which the bedroom came out whole.
RADIUS_M = 0.55
#: Grown by this so the walls and furniture round its floor are inside.
MARGIN_M = 0.25
#: Where the drive of 2026-10-11 stood "at the bedroom door". The split runs
#: the bedroom on into the corridor, and that drive's "half way down the
#: corridor", (-11.42, -13.23), is on the far side of this, so the outline is
#: cut here.
DOOR = (-10.16, -12.85)
CORRIDOR = (-11.42, -13.23)
#: Where that drive parked in the bedroom.
PARKED = (-8.78, -11.63)


def main():
    out_png, out_json = sys.argv[1], sys.argv[2]
    g, cells = split_rooms.grid(GRID)
    res, ox, oy = g["resolution_m"], g["origin_x_m"], g["origin_y_m"]
    free = ndimage.binary_opening((cells >= 0) & ~(cells >= 50), iterations=1)
    labels, rooms = split_rooms.split(free, res, RADIUS_M)
    ii, jj = np.nonzero(labels)
    xs, ys = ox + (jj + 0.5) * res, oy + (ii + 0.5) * res
    things = json.load(open(TARGETS))["things"]
    votes = {}
    for t in things.values():
        d = np.hypot(xs - t["x_m"], ys - t["y_m"])
        k = int(d.argmin())
        room = int(labels[ii[k], jj[k]]) if d[k] <= split_rooms.REACH_M else 0
        votes.setdefault(t["where"], []).append(room)
    bed = max(set(votes["bedroom"]), key=votes["bedroom"].count)
    grown = ndimage.binary_dilation(labels == bed, iterations=int(round(MARGIN_M / res)))
    grown = ndimage.binary_fill_holes(grown)
    cols_x = ox + (np.arange(grown.shape[1]) + 0.5) * res
    grown[:, cols_x < DOOR[0]] = False
    floor = (labels == bed) & grown
    contours, _ = cv2.findContours(grown.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    # The largest piece is the room; a corner cut off by the door line is not.
    c = max(contours, key=cv2.contourArea)
    c = cv2.approxPolyDP(c, 2.0, True)[:, 0, :]  # within two cells, 10 cm
    poly = [[round(ox + (x + 0.5) * res, 2), round(oy + (y + 0.5) * res, 2)] for x, y in c]

    def inside(p):
        return cv2.pointPolygonTest(np.array(poly, np.float32), p, False) > 0

    print("bedroom: %d of %d bedroom things in its region, %.1f m2 of floor, %d vertices"
          % (votes["bedroom"].count(bed), len(votes["bedroom"]), floor.sum() * res * res, len(poly)))
    print("parked spot inside: %s; corridor point inside: %s" % (inside(PARKED), inside(CORRIDOR)))
    json.dump({"map": MAP_ID, "half_doorway_m": RADIUS_M, "margin_m": MARGIN_M,
               "cut_at_door_x_m": DOOR[0], "bedroom": poly}, open(out_json, "w"), indent=1)

    img = np.full(cells.shape + (3,), 128, np.uint8)
    img[free] = 255
    img[cells >= 50] = 0
    img[floor] = [200, 230, 255]
    cv2.polylines(img, [c.astype(np.int32)], True, (0, 0, 220), 1)
    marks = [((t["x_m"], t["y_m"]), (0, 160, 0) if t["where"] == "bedroom" else (200, 0, 200))
             for t in things.values()]
    marks += [(DOOR, (0, 0, 255)), (CORRIDOR, (255, 0, 0)), (PARKED, (0, 200, 255))]
    for (px, py), col in marks:
        cv2.circle(img, (int((px - ox) / res), int((py - oy) / res)), 2, col, -1)
    img = cv2.resize(cv2.flip(img, 0), None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST)
    cv2.imwrite(out_png, img)


if __name__ == "__main__":
    main()
