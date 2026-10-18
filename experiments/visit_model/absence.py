"""How often a visit that pointed the camera at a thing, close up, files nothing
to it -- for things that were there, and for every record.

    python absence.py LOOKS.npz FILING.json

A record the model holds may be a thing that is still there, one taken away,
or a stray: a record started from a region or two that was never anything.
Version 4 marks a record gone only when the lidar sees through its place, so a
thing above the lidar's plane, or a stray, stays present forever. The question
here is whether a visit whose pictures had a record's place in view, close
enough for the detector, and that filed nothing to it, is evidence enough to
say so -- judged first on the eleven objects that were never staged
(labels/stayed-put.json), which were there throughout and must never be called
gone. A picture's pose is its regions' camera place and pointing; a picture
with no region is not in the looks and is not counted.
"""
import collections
import json
import math
import sys

import numpy as np

import model
import truth

#: The camera's half-width across, in degrees, and how far inside it a place
#: must be to count as in view: measured from the regions' own bearings below.
MARGIN_DEG = 8.0
NEAR_M = 0.5


def wrap(deg):
    return (deg + 180.0) % 360.0 - 180.0


def main():
    looks, filing_path = sys.argv[1:3]
    a = np.load(looks, allow_pickle=True)
    filing = {int(k): v for k, v in json.load(open(filing_path)).items()}
    rid = a["id"].astype(int)
    t = a["t"]
    axis = a["heading"] - a["pan"]
    off = wrap(a["bearing"] - axis)
    ok = np.isfinite(off) & np.isfinite(a["cx"])
    half = float(np.percentile(np.abs(off[ok]), 98))
    print("camera: regions' bearing off its axis, 98th percentile %.1f deg; median %.1f" % (
        half, float(np.median(off[ok]))))
    cone = {"deg": half - MARGIN_DEG}

    # One pose per picture: its regions' camera place and pointing.
    pics = {}
    for i in np.flatnonzero(ok):
        f = a["frame"][i]
        if f not in pics:
            pics[f] = (t[i], a["cx"][i], a["cy"][i], axis[i])
    frame_of = a["frame"]

    # Visits, as the model cuts them.
    visits = []
    for w in model.windows(t):
        frames = sorted({frame_of[i] for i in w if frame_of[i] in pics})
        filed = {filing.get(int(rid[i])) for i in w} - {None}
        visits.append((float(t[w].min()), float(t[w].max()), frames, filed))
    print("visits", len(visits), "pictures", len(pics))

    def in_view(x, y, frames, far):
        n = 0
        for f in frames:
            _t, cx, cy, ax = pics[f]
            d = math.hypot(x - cx, y - cy)
            if NEAR_M <= d <= far and abs(wrap(math.degrees(math.atan2(y - cy, x - cx)) - ax)) <= cone["deg"]:
                n += 1
        return n

    placed = np.isfinite(a["px"]) & np.isfinite(a["py"])
    index = {int(r): i for i, r in enumerate(rid)}

    # --- the eleven that stayed put ---------------------------------------
    by_obj = truth.regions_by_object({int(r): float(t[i]) for i, r in enumerate(rid)})
    stayed = set(truth.stayed_put())
    print("\nnever staged, so there throughout: per visit with the place in view of N pictures within D m,"
          " the share that filed nothing to the object's records")
    rows = collections.defaultdict(lambda: [0, 0])
    per_obj = collections.defaultdict(lambda: [0, 0])
    for obj in sorted(stayed):
        labelled = [index[r] for r in by_obj.get(obj, {}) if r in index]
        pl = [i for i in labelled if placed[i]]
        if not pl:
            continue
        x, y = float(np.median(a["px"][pl])), float(np.median(a["py"][pl]))
        recs = collections.Counter(filing.get(int(rid[i])) for i in labelled) - collections.Counter({None: 10**9})
        recs = {r for r, n in recs.items() if n >= 2}
        first = float(t[labelled].min())
        for t0, t1, frames, filed in visits:
            if t1 < first:
                continue
            for far in (2.5, 3.5):
                n = in_view(x, y, frames, far)
                for need in (1, 2, 3, 5):
                    if n >= need:
                        key = (far, need)
                        rows[key][0] += 1
                        rows[key][1] += not (filed & recs)
                if far == 2.5 and n >= 2:
                    per_obj[obj][0] += 1
                    per_obj[obj][1] += not (filed & recs)
    # The longest run of such visits in a row, per object and cone: what a
    # rule "gone after K empty visits in a row" would have got wrong.
    print("  longest run of empty visits in a row (2.5 m, 3+ pictures), by how far off the axis counts as in view:")
    for width in (30.0, 45.0, half - MARGIN_DEG):
        cone["deg"] = width
        worst, runs = {}, collections.Counter()
        for obj in sorted(stayed):
            labelled = [index[r] for r in by_obj.get(obj, {}) if r in index]
            pl = [i for i in labelled if placed[i]]
            if not pl:
                continue
            x, y = float(np.median(a["px"][pl])), float(np.median(a["py"][pl]))
            recs = collections.Counter(filing.get(int(rid[i])) for i in labelled)
            recs = {r for r, n in recs.items() if n >= 2 and r is not None}
            first = float(t[labelled].min())
            run = best = n_vis = n_empty = 0
            for t0, t1, frames, filed in visits:
                if t1 < first or in_view(x, y, frames, 2.5) < 3:
                    continue
                n_vis += 1
                if filed & recs:
                    run = 0
                else:
                    n_empty += 1
                    run += 1
                    best = max(best, run)
            worst[obj] = (best, n_empty, n_vis)
        print("    within %2.0f deg: %s" % (width, {k: v for k, v in worst.items() if v[2]}))
    cone["deg"] = half - MARGIN_DEG
    for (far, need), (n, empty) in sorted(rows.items()):
        print("  within %.1f m, %d+ pictures: %4d visits, %4d filed nothing (%.0f%%)" % (
            far, need, n, empty, 100.0 * empty / max(n, 1)))
    print("  by object (2.5 m, 2+ pictures):")
    for obj, (n, empty) in sorted(per_obj.items()):
        print("    %-28s %4d visits, %3d empty" % (obj, n, empty))

    # --- every record -----------------------------------------------------
    members = collections.defaultdict(list)
    for r, o in filing.items():
        if o is not None and r in index:
            members[o].append(index[r])
    print("\nevery record: visits since it was last filed to that had its place in view"
          " (2.5 m, 2+ pictures) and filed nothing to it")
    tab = collections.defaultdict(collections.Counter)
    for o, idx in members.items():
        pl = [i for i in idx if placed[i]]
        if not pl:
            continue
        x, y = float(np.median(a["px"][pl])), float(np.median(a["py"][pl]))
        times = sorted(float(t[i]) for i in idx)
        seen = sum(1 for k, tt in enumerate(times) if k == 0 or tt - times[k - 1] > model.GAP_S)
        cls = "1" if seen == 1 else "2" if seen == 2 else "3-5" if seen <= 5 else "6+"
        last = times[-1]
        empty = sum(1 for t0, t1, frames, filed in visits
                    if t0 > last and o not in filed and in_view(x, y, frames, 2.5) >= 2)
        tab[cls]["records"] += 1
        tab[cls]["0 empty"] += empty == 0
        tab[cls]["1 empty"] += empty == 1
        tab[cls]["2 empty"] += empty == 2
        tab[cls]["3+ empty"] += empty >= 3
    for cls in ("1", "2", "3-5", "6+"):
        print("  seen on %-3s visits: %s" % (cls, dict(tab[cls])))


if __name__ == "__main__":
    main()
