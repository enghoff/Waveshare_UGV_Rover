"""The benchmark's counts for one filing of the looks: which object each region
was given to.

    python score.py LOOKS.npz [FILING.json]

With no FILING the deployed store's own (`entity` in LOOKS) is scored: the
baseline. FILING is {region id: object key or null}.

Counted, by the rules frozen in docs/plans/a-world-model-kept-per-visit.md
before the new model was run:

- **Trials**: each physical object staged in a session that had at least two
  labelled regions filed before its move (in that session or earlier). Kept
  when the key holding most of its labelled regions after the move held one
  of them before (count_all.py --across); split when not; unresolved with
  fewer than two filed after. `strict` also reports whether that key is the
  one that held most of them before.
- **Wrong merges**: a key holding labelled regions of two physical objects
  filed after a session's staging. For the deployed store only regions filed
  as ordinary matches count, as in criterion 4 (`matched`); every filing of
  the new model counts.
- **Stayed put**: for each object never staged, the keys holding at least two
  of its labelled regions over the test sessions together, against the
  deployed store's.
- **Filed share**: of all labelled regions, the share given to any key, which
  must be at least the deployed store's.
Regions in a session's bad-pose windows are left out throughout.
"""
import json
import sys
from collections import Counter

import data
import truth


def filing_of(looks, path=None):
    if path is None:
        return {int(i): e for i, e in zip(looks["id"], looks["entity"])}, True
    raw = json.load(open(path))
    return {int(k): v for k, v in raw.items()}, False


def score(filing, looks, baseline=False, roles=("development", "test")):
    ss = truth.sessions()
    by_name = {s.name: s for s in ss}
    t_of = dict(zip(looks["id"].tolist(), looks["t"].tolist()))
    objs = truth.regions_by_object(t_of, ss)
    matched = dict(zip(looks["id"].tolist(), looks["matched"].tolist()))

    def usable(rid, sess):
        return rid in t_of and not by_name[sess].is_bad(t_of[rid])

    out = {"trials": [], "wrong_merges": [], "stayed_put": {}}
    for k, s in enumerate(ss):
        if s.role not in roles:
            continue
        earlier = {x.name for x in ss[:k + 1]}
        for obj, mt, next_t in s.events():
            mine = objs.get(obj, {})
            before = Counter(filing.get(r) for r, sess in mine.items()
                             if sess in earlier and usable(r, sess) and t_of[r] < mt
                             and filing.get(r))
            until = next_t if next_t is not None else float("inf")
            after = Counter(filing.get(r) for r, sess in mine.items()
                            if sess == s.name and usable(r, sess) and mt <= t_of[r] < until
                            and filing.get(r))
            unfiled = sum(1 for r, sess in mine.items() if sess == s.name and usable(r, sess)
                          and mt <= t_of[r] < until and not filing.get(r))
            row = {"session": s.name, "role": s.role, "object": obj,
                   "before": dict(before.most_common(6)), "after": dict(after.most_common(6)),
                   "unfiled_after": unfiled}
            if sum(before.values()) < 2:
                continue
            if sum(after.values()) < 2:
                row["outcome"] = "unresolved"
            else:
                top = after.most_common(1)[0][0]
                row["outcome"] = "kept" if top in before else "split"
                row["strict"] = top == before.most_common(1)[0][0]
            out["trials"].append(row)
        # Wrong merges after the staging.
        held = {}
        for obj, mine in objs.items():
            for r, sess in mine.items():
                if sess != s.name or not usable(r, sess) or t_of[r] < s.staged_at:
                    continue
                key = filing.get(r)
                if not key or (baseline and not matched.get(r)):
                    continue
                held.setdefault(key, Counter())[obj] += 1
        for key, by in held.items():
            if len(by) > 1:
                out["wrong_merges"].append({"session": s.name, "role": s.role, "key": key,
                                            "objects": dict(by)})
    # The share of all labelled regions filed at all, per role.
    out["filed"] = {}
    for role_name in sorted({s.role for s in ss}):
        names = {s.name for s in ss if s.role == role_name}
        rows = [r for mine in objs.values() for r, sess in mine.items()
                if sess in names and usable(r, sess)]
        out["filed"][role_name] = round(sum(1 for r in rows if filing.get(r)) / max(1, len(rows)), 3)
    for obj in truth.stayed_put():
        per = {}
        for label in sorted({s.role for s in ss}):
            role_set = {s.name for s in ss if s.role == label}
            c = Counter(filing.get(r) for r, sess in objs.get(obj, {}).items()
                        if sess in role_set and usable(r, sess) and filing.get(r))
            n = sum(1 for r, sess in objs.get(obj, {}).items() if sess in role_set and usable(r, sess))
            per[label] = {"keys_with_two": sum(1 for v in c.values() if v >= 2), "labelled": n,
                          "filed": sum(c.values()), "top_share": round(c.most_common(1)[0][1] / n, 2)
                          if c and n else 0.0}
        out["stayed_put"][obj] = per
    return out


def summary(out, role):
    trials = [t for t in out["trials"] if t["role"] == role]
    kept = sum(t["outcome"] == "kept" for t in trials)
    merges = [m for m in out["wrong_merges"] if m["role"] == role]
    sp = {k: v[role]["keys_with_two"] for k, v in out["stayed_put"].items()}
    return {"trials": len(trials), "kept": kept, "strict": sum(bool(t.get("strict")) for t in trials),
            "split": sum(t["outcome"] == "split" for t in trials),
            "unresolved": sum(t["outcome"] == "unresolved" for t in trials),
            "wrong_merges": len(merges), "filed_share": out["filed"][role], "stayed_put_keys": sp}


def report(out, roles=("development", "test")):
    for t in out["trials"]:
        print("  %-10s %-15s %-10s strict=%-5s before %s after %s, %d unfiled" % (
            t["session"], t["object"], t["outcome"], t.get("strict"), t["before"], t["after"],
            t["unfiled_after"]))
    for m in out["wrong_merges"]:
        print("  %-10s wrong merge: %s holds %s" % (m["session"], m["key"], m["objects"]))
    for obj, per in out["stayed_put"].items():
        print("  stayed put %-22s %s" % (obj, per))
    for role in roles:
        print("  %s: %s" % (role, summary(out, role)))


if __name__ == "__main__":
    args = sys.argv[1:]
    roles = ("development", "test") if "--test" in args else ("development",)
    # --role NAME scores that role alone (session 5 is its own: "session 5").
    if "--role" in args:
        k = args.index("--role")
        roles = (args[k + 1],)
        args = args[:k] + args[k + 2:]
    args = [x for x in args if x != "--test"]
    looks = data.load(args[0])
    filing, baseline = filing_of(looks, args[1] if len(args) > 1 else None)
    out = score(filing, looks, baseline=baseline, roles=roles)
    if "test" not in roles:
        out["stayed_put"] = {k: {r: v[r] for r in roles if r in v} for k, v in out["stayed_put"].items()}
        out["filed"].pop("test", None)
    report(out, roles)
