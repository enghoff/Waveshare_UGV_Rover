"""Filings side by side on every session, by the agreed counts.

    python compare.py LOOKS.npz NAME=FILING.json ... [deployed]

Per filing: wrong merges counted across sessions (pass/fail), and, measured
only, moves of distinguishable things kept by the strict reading, unmoved
objects held in two records or fewer, and the share of labelled pictures
filed, per role. "deployed" scores the store's own filing.
"""
import json
import sys

import data
import score
import truth

ROLES = ("development", "test", "session 5")


def main():
    looks = data.load(sys.argv[1])
    rows = []
    for arg in sys.argv[2:]:
        if arg == "deployed":
            filing, base, name = score.filing_of(looks)[0], True, "deployed"
        else:
            name, path = arg.split("=", 1)
            filing, base = score.filing_of(looks, path)[0], False
        out = score.score(filing, looks, baseline=base, roles=ROLES)
        cells = []
        for role in ROLES:
            trials = [t for t in out["trials"] if t["role"] == role
                      and score.ALIKE.get(t["object"], t["object"]) != "armchair"]
            merges = [m for m in out["wrong_merges"] if m["role"] == role]
            sp = [v[role]["keys_with_two"] for v in out["stayed_put"].values()
                  if role in v and v[role]["labelled"] >= 2]
            cells.append("%d merges, %d/%d strict, unmoved %d/%d, filed %.0f%%" % (
                len(merges), sum(bool(t.get("strict")) for t in trials), len(trials),
                sum(1 for k in sp if k <= 2), len(sp), 100 * out["filed"].get(role, 0)))
        rows.append((name, cells))
    for role_k, role in enumerate(ROLES):
        print(role)
        for name, cells in rows:
            print("   %-10s %s" % (name, cells[role_k]))


if __name__ == "__main__":
    main()
