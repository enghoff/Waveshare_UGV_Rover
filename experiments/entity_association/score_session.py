"""Splits and wrong looks on a replayed session, against three frozen label sets.

R-WS-17 asks that each object be one thing: an object is split when a second
thing holds two or more of its looks. R-WS-18 asks that a thing hold only its
object's looks: a labelled look is wrong when it is filed under a thing whose
labelled looks are mostly of another object, or, for a look labelled as no single
object, under a thing that holds a labelled object at all.

The three label sets are scored separately, because they were labelled
separately, by the coding agents, at different times and to different rules:

    development   map session 67 on 2026-10-03, 25 things, 412 looks
                  (captures/2026-10-03-merge-review/look_labels.json)
    depth drive   2026-10-05 (captures/2026-10-05-depth-drive-2/physical-subjects-corrected.json)
    trial         2026-10-07 (captures/visibility-independent-20261007-12/physical-subjects-frozen.json)

`CROSS_TIME` joins objects named in more than one set where they are certainly
the same object, frozen before any variant was scored on 2026-10-07; it measures
whether a later look reaches the thing an earlier one made. Dining chairs, the
owner and the floor are left out of every set.

    python experiments/entity_association/score_session.py --result <replay>/result.json
        [--against <other-replay>/result.json] --output <new-file>
"""
from __future__ import annotations

import argparse
import itertools
import json
from collections import Counter, defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

#: Development things that are one physical object, from the label notes
#: ("a duplicate of object:249" and so on). Chairs, the owner and the floor
#: are not scored.
DEVELOPMENT_OBJECTS = {
    "black-cabinet": ["object:242"],
    "green-landscape-painting": ["object:246", "object:351"],
    "cow-painting": ["object:249", "object:340"],
    "portrait-of-a-man": ["object:275"],
    "still-life-on-its-corner": ["object:276"],
    "light-wooden-panel": ["object:307"],
    "grey-painting-gold-frame": ["object:335"],
    "dark-painting-on-its-corner": ["object:343"],
    "metal-desk-with-monitor": ["object:374"],
    "dark-armchair": ["object:377"],
    "blue-painting-building-by-sea": ["object:383", "object:400"],
    "dark-door": ["object:386"],
    "blue-picture-on-its-corner": ["object:387"],
    "floor-lamp": ["object:402"],
    "ceiling-fan": ["object:407"],
    "snowy-winter-painting": ["object:413"],
}
LEFT_OUT = {"object:247", "object:302", "object:350", "object:332", "object:385",
            "object:259"}

#: The same object under different names in different sets, joined only where
#: there is no doubt. Frozen 2026-10-07, before any variant was scored.
CROSS_TIME = {
    "green-landscape-painting": [("development", "green-landscape-painting"),
                                 ("depth drive", "painting-behind-chairs"),
                                 ("trial", "painting-behind-chairs")],
    "snowy-painting-over-cabinet": [("development", "snowy-winter-painting"),
                                    ("depth drive", "snowy-forest-painting-over-cabinet"),
                                    ("trial", "snowy-forest-painting-over-cabinet")],
    "black-cabinet": [("development", "black-cabinet"), ("trial", "black-cabinet")],
    "purple-armchair": [("depth drive", "purple-armchair"), ("trial", "purple-armchair")],
}


def label_sets() -> dict:
    """Each set as {"objects": {name: [ids]}, "other": [ids]}: looks of a named
    object, and labelled looks that are of no single scored object."""
    sets = {}
    dev = json.loads((ROOT / "captures/2026-10-03-merge-review/look_labels.json")
                     .read_text())["things"]
    objects, other = defaultdict(list), []
    for name, things in DEVELOPMENT_OBJECTS.items():
        for thing in things:
            objects[name].extend(dev[thing]["main_looks"])
            other.extend(dev[thing]["odd"])
    sets["development"] = {"objects": dict(objects), "other": other}
    for key, path in (("depth drive", "captures/2026-10-05-depth-drive-2/"
                                      "physical-subjects-corrected.json"),
                      ("trial", "captures/visibility-independent-20261007-12/"
                                "physical-subjects-frozen.json")):
        doc = json.loads((ROOT / path).read_text())
        other = [int(i) for i in doc.get("mixed", {})]
        other += [int(i) for i in doc.get("non_object", {})]
        sets[key] = {"objects": {k: list(v) for k, v in doc["subjects"].items()},
                     "other": other}
    return sets


def score_set(owners: dict, objects: dict, other: list) -> dict:
    owner = lambda i: owners.get(str(i))  # noqa: E731
    split, records, cohesion, waiting = [], {}, {}, 0
    majority: dict = {}
    by_record = defaultdict(Counter)
    for name, ids in objects.items():
        held = Counter(owner(i) for i in ids if owner(i) is not None)
        waiting += sum(1 for i in ids if owner(i) is None)
        for record, count in held.items():
            by_record[record][name] += count
        if len(ids) >= 2:
            records[name] = len(held)
            cohesion[name] = round(max(held.values()) / len(ids), 3) if held else 0.0
            if sum(1 for n in held.values() if n >= 2) >= 2:
                split.append(name)
    for record, counts in by_record.items():
        majority[record] = counts.most_common(1)[0][0]
    filed = wrong = 0
    for name, ids in objects.items():
        for i in ids:
            if owner(i) is not None:
                filed += 1
                wrong += majority[owner(i)] != name
    stray = [i for i in other if owner(i) is not None and owner(i) in majority]
    same = [(a, b) for ids in objects.values() for a, b in itertools.combinations(ids, 2)]
    cross = [(a, b) for (_, x), (_, y) in itertools.combinations(objects.items(), 2)
             for a in x for b in y]
    linked = lambda pairs: sum(1 for a, b in pairs  # noqa: E731
                               if owner(a) is not None and owner(a) == owner(b))
    scored = [n for n in objects if len(objects[n]) >= 2]
    return {"objects": len(scored), "split_objects": sorted(split),
            "split_share": round(len(split) / max(len(scored), 1), 3),
            "records_per_object": round(sum(records.values()) / max(len(records), 1), 2),
            "mean_cohesion": round(sum(cohesion.values()) / max(len(cohesion), 1), 3),
            "labelled_looks": sum(len(v) for v in objects.values()),
            "waiting": waiting, "filed": filed,
            "wrong_object_looks": wrong, "stray_other_looks": len(stray),
            "wrong_share": round((wrong + len(stray)) / max(filed + len(stray), 1), 3),
            "same_pairs": len(same), "same_linked": linked(same),
            "cross_pairs": len(cross), "cross_linked": linked(cross)}


def still_ids(database: Path, session: int) -> set:
    """Observations from looks taken standing still (`replay_session.moving`)."""
    import replay_session
    return {row["id"] for group in replay_session.looks_in_order(database, session)
            if not replay_session.moving(group) for row in group}


def score(result: dict, only: set | None = None) -> dict:
    owners = result["owners"]
    sets = label_sets()
    if only is not None:
        sets = {key: {"objects": {name: [i for i in ids if i in only]
                                  for name, ids in s["objects"].items()},
                      "other": [i for i in s["other"] if i in only]}
                for key, s in sets.items()}
    report = {"variant": result.get("variant"), "things": result.get("things"),
              "only_still_looks": only is not None,
              "sets": {k: score_set(owners, v["objects"], v["other"])
                       for k, v in sets.items()}}
    cross_time = {}
    for name, parts in CROSS_TIME.items():
        ids = [i for key, sub in parts for i in sets[key]["objects"].get(sub, [])]
        held = Counter(owners.get(str(i)) for i in ids if owners.get(str(i)))
        cross_time[name] = {"looks": len(ids), "records": len(held),
                            "largest_share": round(max(held.values()) / len(ids), 3)
                            if held else 0.0,
                            "waiting": sum(1 for i in ids if not owners.get(str(i)))}
    report["cross_time"] = cross_time
    return report


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--result", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--only-still", type=Path, metavar="STORE",
                   help="score only labelled looks taken standing still in this store")
    a = p.parse_args()
    assert not a.output.exists(), "choose a new output file"
    only = still_ids(a.only_still, 67) if a.only_still else None
    report = score(json.loads(a.result.read_text()), only)
    a.output.write_text(json.dumps(report, indent=1) + "\n")
    for key, s in report["sets"].items():
        print("%-12s split %d/%d objects, %.2f records each, cohesion %.2f, "
              "wrong %d+%d of %d filed (%.1f%%), waiting %d, cross links %d"
              % (key, len(s["split_objects"]), s["objects"], s["records_per_object"],
                 s["mean_cohesion"], s["wrong_object_looks"], s["stray_other_looks"],
                 s["filed"], 100 * s["wrong_share"], s["waiting"], s["cross_linked"]))
    for name, c in report["cross_time"].items():
        print("  across days, %-28s %3d looks in %2d records, largest %.0f%%, waiting %d"
              % (name, c["looks"], c["records"], 100 * c["largest_share"], c["waiting"]))


if __name__ == "__main__":
    main()
