"""Which records a "set aside" rule would hide at the end of a filing, judged
on the labels. The rule: seen on at most MAX_VISITS visits, last seen at least
MIN_DAYS before the end, and looked for since -- visits whose pictures had its
place within 2.5 m and CONE degrees of the camera's axis in three or more of
them, and that filed nothing to it -- at least LOOKED_FOR times. Nothing is
called gone; the filing is the model's as it stands.

    python set_aside.py LOOKS.npz FILING.json

Honesty here: every object that was there at the end keeps a standing record
holding the most of its labelled pictures, and how many of its other records
are set aside is listed.
"""
import collections
import json
import math
import sys

import numpy as np

import model
import truth

MAX_VISITS, MIN_DAYS, LOOKED_FOR, CONE, FAR = 2, 7.0, 3, 45.0, 2.5


def wrap(d):
    return (d + 180.0) % 360.0 - 180.0


def main():
    looks, filing_path = sys.argv[1:3]
    a = np.load(looks, allow_pickle=True)
    rid = a["id"].astype(int)
    t = a["t"]
    index = {int(r): i for i, r in enumerate(rid)}
    filing = {int(k): v for k, v in json.load(open(filing_path)).items()}
    decisions = json.load(open(filing_path + ".decisions.json"))
    gone = set()
    for d in decisions:
        if "gone" in d:
            gone.add(d["gone"])
        for key in ("new", "here", "moved"):
            if isinstance(d.get(key), str):
                gone.discard(d[key])
    axis = a["heading"] - a["pan"]
    ok = np.isfinite(axis) & np.isfinite(a["cx"])
    pics = {}
    for i in np.flatnonzero(ok):
        pics.setdefault(a["frame"][i], (a["cx"][i], a["cy"][i], axis[i]))
    visits = []
    for w in model.windows(t):
        frames = sorted({a["frame"][i] for i in w if a["frame"][i] in pics})
        filed = {filing.get(int(rid[i])) for i in w} - {None}
        visits.append((float(t[w].min()), frames, filed))
    end = float(t.max())
    placed = np.isfinite(a["px"]) & np.isfinite(a["py"])
    members = collections.defaultdict(list)
    for r, o in filing.items():
        if o is not None and r in index:
            members[o].append(index[r])
    aside, standing = set(), set()
    for o, idx in members.items():
        if o in gone:
            continue
        pl = [i for i in idx if placed[i]]
        times = sorted(float(t[i]) for i in idx)
        seen = sum(1 for k, x in enumerate(times) if k == 0 or x - times[k - 1] > model.GAP_S)
        last = times[-1]
        if not pl or seen > MAX_VISITS or end - last < MIN_DAYS * 86400:
            standing.add(o)
            continue
        x, y = float(np.median(a["px"][pl])), float(np.median(a["py"][pl]))
        looked = 0
        for t0, frames, filed in visits:
            if t0 <= last or o in filed:
                continue
            n = 0
            for f in frames:
                cx, cy, ax = pics[f]
                d = math.hypot(x - cx, y - cy)
                if 0.5 <= d <= FAR and abs(wrap(math.degrees(math.atan2(y - cy, x - cx)) - ax)) <= CONE:
                    n += 1
            looked += n >= 3
        (aside if looked >= LOOKED_FOR else standing).add(o)
    print("standing records %d; set aside %d" % (len(standing) + len(aside), len(aside)))

    by_obj = truth.regions_by_object({int(r): float(x) for r, x in zip(rid, t)})
    stayed = set(truth.stayed_put())
    print("objects never staged: their main record, and their other records set aside")
    for obj in sorted(by_obj):
        held = collections.Counter(filing.get(r) for r in by_obj[obj] if filing.get(r))
        if not held:
            continue
        main_rec, n = held.most_common(1)[0]
        others = [(r, c) for r, c in held.items() if r in aside]
        flag = "STAYED" if obj in stayed else "staged"
        print("  %-6s %-28s main %-6s (%3d pictures) %s; set aside: %s" % (
            flag, obj, main_rec, n, "SET ASIDE" if main_rec in aside else "standing" if main_rec in standing else "gone",
            others or "none"))


if __name__ == "__main__":
    main()
