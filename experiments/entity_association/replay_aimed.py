"""The recorded aimed looks put through `world_state.aimed`, as if each had named its target.

A working copy of the 2026-10-06 store (the targeted-look audit's) is filed into
by the production code, look by look in time order, and the reply each would
have carried is kept: what was filed, the records it also fitted, and the
target's claim before and after. The copy holds placements as they stood when it
was taken, not as each goal saw them, so this exercises the code on real looks
rather than reproducing the runs. Nothing outside the copy is written.

    python experiments/entity_association/replay_aimed.py --output <new-file>
"""
import argparse
import json
import sqlite3
import sys
import tempfile
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from world_state import aimed  # noqa: E402
from world_state.store import WorldStore  # noqa: E402

AUDIT = ROOT / "captures/entity-target-audit-20261006"


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    assert not a.output.exists(), "choose a new output file"
    rows = [r for r in json.loads((AUDIT / "audit.json").read_text())["rows"]
            if r.get("observations")]
    rows.sort(key=lambda r: r["call_at"])
    out, tally = [], Counter()
    with tempfile.TemporaryDirectory(prefix="ugv-aimed-") as tmp:
        store = WorldStore(tmp)
        with sqlite3.connect((AUDIT / "world.db").resolve().as_uri() + "?mode=ro",
                             uri=True) as source:
            source.backup(store.db)
        store._create()
        try:
            for row in rows:
                got = aimed.file_by_aim(store, row["target"], row["frame_id"])
                tally["filed" if got.get("filed") else got.get("why", "")[:60]] += 1
                if got.get("filed"):
                    tally["  of which ranged"] += bool(got.get("ranged"))
                    tally["  of which taken from another record"] += bool(
                        got.get("was") and got["was"] != row["target"])
                    tally["  of which with a same-object suspect"] += bool(
                        got.get("same_object_suspects"))
                    before, after = got.get("claim_before_m"), got.get("claim_after_m")
                    if before is not None and after is not None:
                        tally["  of which the claim fell"] += after < before - 0.02
                placed = next((e for e in store.placed(map_session=store.map_session())
                               if e["id"] == row["target"]), None)
                where = (placed or {}).get("placement") or {}
                out.append({"episode": row["episode"], "target": row["target"],
                            "frame_id": row["frame_id"], **got,
                            "after_xy": [where.get("x_m"), where.get("y_m")]})
        finally:
            store.close()
    a.output.write_text(json.dumps({"looks": len(rows), "tally": dict(tally),
                                    "independent_acceptance": False,
                                    "looks_detail": out}, indent=1) + "\n")
    for key, value in tally.most_common():
        print("%4d  %s" % (value, key))


if __name__ == "__main__":
    main()
