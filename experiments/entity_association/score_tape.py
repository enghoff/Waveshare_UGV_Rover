"""Where a replayed session leaves the six taped targets, against the owner's tape.

For each target taped on 2026-10-03, the replay's record holding most of the
target's 126 eye-matched looks is taken as its record; its final placement is
compared with the tape, beside the uncertainty it states.

    python experiments/entity_association/score_tape.py <replay>/result.json [...]
"""
import json
import math
import sqlite3
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "captures/2026-10-03-targets"))
from frame import TRUTH, W2M  # noqa: E402

TARGETS = {"T2": ["object:5"], "T3": ["object:3"], "T4": ["object:26", "object:37", "object:30"],
           "T7": ["object:4", "object:16"], "T8": ["object:6", "object:10"], "T9": ["object:22"]}


def score(result):
    accept = sqlite3.connect((ROOT / "captures/2026-10-03-targets/world-2026-10-03-acceptance.db")
                             .resolve().as_uri() + "?mode=ro", uri=True)
    owners = result["owners"]
    out = {}
    for target, entities in TARGETS.items():
        ids = [r[0] for e in entities for r in accept.execute(
            "select id from observations where entity_id=?", (e,))]
        held = Counter(owners.get(str(i)) for i in ids if owners.get(str(i)))
        if not held:
            out[target] = None
            continue
        record, n = held.most_common(1)[0]
        p = result["placements"].get(record)
        tx, ty = W2M(*TRUTH[target][:2])
        out[target] = {"record": record, "holds": n, "of": len(ids),
                       "off_m": round(math.hypot(p["x_m"] - tx, p["y_m"] - ty), 3) if p else None,
                       "stated_m": (p.get("stated_uncertainty_m") or p.get("uncertainty_m"))
                       if p else None,
                       "records": len(held)}
    return out


if __name__ == "__main__":
    for path in sys.argv[1:]:
        s = score(json.loads(Path(path).read_text()))
        inside = sum(1 for v in s.values() if v and v["off_m"] is not None and v["off_m"] <= v["stated_m"])
        offs = sorted(v["off_m"] for v in s.values() if v and v["off_m"] is not None)
        print("%-40s median off %.2f m; inside stated %d/6; %s" % (
            path, offs[len(offs) // 2], inside,
            " ".join("%s %.2f/%.2f" % (k, v["off_m"], v["stated_m"]) if v and v["off_m"] is not None
                     else "%s -" % k for k, v in s.items())))
