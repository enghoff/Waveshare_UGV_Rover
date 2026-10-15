"""The plan's test, as frozen in docs/plans/a-world-model-kept-per-visit.md.

    python gate.py LOOKS.npz FILING.json

Scores the filing and the deployed store on the test sessions (3 and 4) and
says whether each condition holds:

1. no key holds labelled regions of two objects (wrong merges);
2. more staged-move trials kept than the deployed store, and at least half;
3. every object that stayed put held by no more keys than in the deployed
   store, counting at least one, and by two or fewer for three in four;
4. at least the deployed store's share of labelled regions filed.
"""
import json
import sys

import data
import score


def main():
    looks = data.load(sys.argv[1])
    deployed, _ = score.filing_of(looks)
    mine, _ = score.filing_of(looks, sys.argv[2])
    base = score.score(deployed, looks, baseline=True, roles=("test",))
    new = score.score(mine, looks, roles=("test",))
    score.report(new, ("test",))
    b, n = score.summary(base, "test"), score.summary(new, "test")
    print("deployed:", b)
    checks = []
    checks.append(("no wrong merges", n["wrong_merges"] == 0, "%d" % n["wrong_merges"]))
    checks.append(("kept more than deployed, and half", n["kept"] > b["kept"] and 2 * n["kept"] >= n["trials"],
                   "%d of %d against %d" % (n["kept"], n["trials"], b["kept"])))
    each = all(n["stayed_put_keys"][o] <= max(1, b["stayed_put_keys"][o]) for o in n["stayed_put_keys"])
    few = sum(v <= 2 for v in n["stayed_put_keys"].values())
    checks.append(("unmoved: no more records than deployed (at least one)", each,
                   json.dumps({o: [n["stayed_put_keys"][o], b["stayed_put_keys"][o]]
                               for o in n["stayed_put_keys"]})))
    checks.append(("unmoved: two or fewer for three in four", 4 * few >= 3 * len(n["stayed_put_keys"]),
                   "%d of %d" % (few, len(n["stayed_put_keys"]))))
    checks.append(("filed share at least deployed", n["filed_share"] >= b["filed_share"],
                   "%.3f against %.3f" % (n["filed_share"], b["filed_share"])))
    for name, ok, why in checks:
        print("%-55s %s  %s" % (name, "PASS" if ok else "FAIL", why))
    print("TEST %s" % ("PASSES" if all(c[1] for c in checks) else "DOES NOT PASS"))


if __name__ == "__main__":
    main()
