"""desk_answers' ranking for some questions at one day's end, with what each
record holds: python diag_rank.py DAY LOOKS PHRASES DB QUESTION,... RECORDING ..."""
import base64
import json
import os
import shutil
import sys
import tempfile
import time
from collections import Counter

import data
import desk_answers as da
import model
import recorded
import truth
from world_state import search, visit_things


def main():
    day, looks, phrases, db, qs = sys.argv[1:6]
    recs = [recorded.Recording(p) for p in sys.argv[6:]]
    a = data.load(looks)
    vectors = {k: base64.b64decode(v) for k, v in json.load(open(phrases)).items() if k in qs.split(",")}
    filing, mem, _ = model.run(model._until(a, da.DAY_ENDS[day]), recs)
    recs_payload = da.things(filing, mem)
    t_of = {int(i): float(t) for i, t in zip(a["id"], a["t"])}
    label = {}
    for obj, regions in truth.regions_by_object(t_of).items():
        for rid in regions:
            label[rid] = obj
    payload = {"generation": "desk", **da.FILLING, "written_at": time.time(), "things": recs_payload}
    tmp = tempfile.mkdtemp()
    try:
        os.makedirs(os.path.join(tmp, "visit_model"))
        with open(os.path.join(tmp, visit_things.THINGS_FILE), "w") as f:
            json.dump(payload, f)
        records = visit_things.VisitStore(da.Store(tmp, payload, db))
        index = search.ThingIndex(records)
        by_id = {r["id"]: r for r in recs_payload}
        for q, v in vectors.items():
            ans = search.rank_things(v, index)
            print("%s, day %s (floor %.3f):" % (q, day, ans["floor"]))
            for m in (ans.get("matches") or [])[:int(os.environ.get("DIAG_N", 6))]:
                r = by_id.get(m["entity_id"]) or {}
                held = Counter(label.get(i, "-") for i in r.get("observations", []))
                print("  %-6s %.3f at (%.2f, %.2f) %-8s seen %s, %d pictures, labels %s" % (
                    m["entity_id"], float(m["score"]), r.get("x_m", 0), r.get("y_m", 0), r.get("state"),
                    time.strftime("%m-%d %H:%M", time.localtime(r.get("seen_at") or 0)),
                    len(r.get("observations", [])), dict(held.most_common(4))))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    main()
