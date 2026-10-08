"""Does an aimed look that files nothing say the record is not where it claims?

Most aimed looks of 2026-10-08 filed nothing because the record they were sent to
improve is not recognisably in front of the camera
(docs/progress/2026-10-08-why-aimed-looks-miss.md). The world state already has
a test for "nothing stands there" that does not trust a detector's silence:
`hypothesis_check.check` calls a claim contradicted only when depth was measured
past the place across the whole of its uncertainty. This puts every aimed look
of the day's sessions through it, with the claim as the goal saw it -- the place
the look was aimed at (`aim_at`), the record's height, and its matching
tolerance -- and tallies the verdicts for looks that filed (the record was
there: a contradiction would be a false absence) and looks that did not.

`hypothesis_check` declines to test a claim placed more loosely than
`LOOSEST_M` (0.5 m): one look cannot see through a wide place. Geometry goals aim
at loose records by design, so the tally is also given with that limit at 1.0
and 2.0 m, to see what a looser test would assert and how often it would be
wrong.

    python experiments/entity_association/aimed_absence.py --store <dir with world.db
        and frames/> --episodes <episodes.db> --looks <aimed-looks.json> [...]
        --output <new-file>

Read-only: the store is copied, and depth is read from the given frames.
"""
from __future__ import annotations

import argparse
import json
import shutil
import sqlite3
import sys
import tempfile
from collections import Counter
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from world_state import hypothesis_check  # noqa: E402
from world_state.store import WorldStore  # noqa: E402

LENS = ROOT / "captures/2026-09-30-oak-rail/oak-health.json"
LIMITS = (0.5, 1.0, 2.0)
#: The centre test: the claim's uncertainty replaced by this, so that the
#: question is whether anything stands at the record's centre -- where the look
#: was aimed -- rather than anywhere it might be.
CENTRES = (0.15, 0.3)


def aim_points(episodes: Path) -> dict[int, dict]:
    """episode -> the look's aim point, from the world_inspect call's params."""
    db = sqlite3.connect(episodes.resolve().as_uri() + "?mode=ro", uri=True)
    out = {}
    for episode, body in db.execute(
            "SELECT episode_id, body_json FROM events WHERE kind='call'"):
        call = json.loads(body)
        if call.get("call") == "world_inspect" and (call.get("params") or {}).get("aim_at"):
            out[episode] = call["params"]["aim_at"]
    return out


def main():
    a = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    a.add_argument("--store", type=Path, required=True)
    a.add_argument("--episodes", type=Path, required=True)
    a.add_argument("--looks", type=Path, action="append", required=True)
    a.add_argument("--output", type=Path, required=True)
    args = a.parse_args()
    assert not args.output.exists(), "choose a new output file"
    lens = SimpleNamespace(**json.loads(LENS.read_text())["colour"]["intrinsics"])
    aims = aim_points(args.episodes)
    tmp = tempfile.mkdtemp(prefix="aimed-absence-")
    shutil.copy(args.store / "world.db", Path(tmp) / "world.db")
    store = WorldStore(tmp)
    store.frames_dir = str(args.store / "frames")
    rows = []
    for path in args.looks:
        for look in json.loads(path.read_text()):
            if look.get("status") != "ok" or not look.get("frame_id"):
                continue
            aim = aims.get(look["episode"])
            entity = store.entity(look["target"]) or {}
            placement = entity.get("placement") if isinstance(entity.get("placement"), dict) else None
            if not aim or not placement:
                continue
            claim = {"x_m": aim["x_m"], "y_m": aim["y_m"],
                     "height_m": placement.get("height_m"),
                     "uncertainty_m": placement.get("uncertainty_m")}
            regions = store.observations(frame_id=look["frame_id"], limit=200, vectors=True)
            depth = store.depth(look["frame_id"])
            # The lens saved with the depth map, as the rover's own check reads it
            # (`hypothesis_check._lens_from`); the bench calibration only where
            # none was saved.
            lens_here = (None if hypothesis_check._lens_from(depth[1]) is not None
                         else lens)
            verdicts = {}
            for limit in LIMITS:
                hypothesis_check.LOOSEST_M = limit
                got = hypothesis_check.check(regions, claim, depth=depth, lens=lens_here)
                verdicts[str(limit)] = [got["outcome"], got.get("code") or got.get("why", "")[:60]]
            hypothesis_check.LOOSEST_M = LIMITS[0]
            for radius in CENTRES:
                got = hypothesis_check.check(regions, {**claim, "uncertainty_m": radius},
                                             depth=depth, lens=lens_here)
                verdicts["centre %s" % radius] = [got["outcome"], got.get("code"),
                                                  (got.get("evidence") or {}).get("depth")]
            from world_state import resolve
            alike = [resolve.appearance(store, look["target"], one.get("dino_blob") or b"")
                     for one in regions]
            alike = [one for one in alike if one is not None]
            rows.append({"session": path.parent.name, "episode": look["episode"],
                         "target": look["target"], "frame_id": look["frame_id"],
                         "filed": bool((look.get("filing") or {}).get("filed")),
                         "claim": claim, "depth_kept": depth[0] is not None,
                         "most_alike": round(max(alike), 3) if alike else None,
                         "verdicts": verdicts})
    store.close()
    args.output.write_text(json.dumps(rows, indent=1) + "\n")
    print(len(rows), "aimed looks with an aim point and a placed target")
    for limit in [*LIMITS, *("centre %s" % r for r in CENTRES)]:
        for filed in (True, False):
            part = [r for r in rows if r["filed"] == filed]
            print("%-10s %-10s %s" % (limit, "filed" if filed else "not filed",
                  dict(Counter(r["verdicts"][str(limit)][0] for r in part))))


if __name__ == "__main__":
    main()
