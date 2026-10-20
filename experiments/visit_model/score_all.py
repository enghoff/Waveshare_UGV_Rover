"""score.py's counts for several filings, every role side by side.

    python score_all.py LOOKS.npz NAME=FILING.json ..."""
import sys

import data
import score
import truth

looks = data.load(sys.argv[1])
roles = []
for s in truth.sessions():
    if s.role not in roles:
        roles.append(s.role)
print("%-8s %-12s %6s %5s %6s %5s %5s %6s %6s %s" % ("filing", "role", "trials", "kept", "strict", "split",
                                                    "unres", "merges", "filed", "stayed-put keys"))
for arg in sys.argv[2:]:
    name, path = arg.split("=", 1)
    filing, _ = score.filing_of(looks, path)
    out = score.score(filing, looks, baseline=False, roles=tuple(roles))
    for role in roles:
        s = score.summary(out, role)
        print("%-8s %-12s %6d %5d %6d %5d %5d %6d %6.3f %d" % (
            name, role, s["trials"], s["kept"], s["strict"], s["split"], s["unresolved"], s["wrong_merges"],
            s["filed_share"], sum(s["stayed_put_keys"].values())))
    for m in out["wrong_merges"]:
        print("    wrong merge %s: %s took %s, holding before %s" % (m["session"], m["key"], m["objects"],
                                                                m.get("held_before", {})))
