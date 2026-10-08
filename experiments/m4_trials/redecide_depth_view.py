"""Today's geometry decisions made again, with viewpoints the depth camera can see from.

Every deliberation recorded the situation it was made from (`situation.restore`).
This restores each one from the day's runs and asks the scorer again with the
code as it stands, then compares with what was chosen at the time:

- whether the chosen geometry goal's thing was inside the depth camera's view
  from the viewpoint (recomputed for the recorded choice with the same rule,
  `goals._tilt_for`);
- how many geometry candidates the new rule refuses for being outside it;
- how often nothing at all would be chosen.

Read-only: the episode store is copied first.

    python experiments/m4_trials/redecide_depth_view.py --episodes <episodes.db>
        --since "2026-10-08 11:50" --output <new-file>
"""
from __future__ import annotations

import argparse
import json
import shutil
import sqlite3
import sys
import tempfile
import time
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "autonomy"))
import goals  # noqa: E402
import scoring  # noqa: E402
import situation as situation_mod  # noqa: E402
from store import EpisodeStore  # noqa: E402


def recorded_choices(db, since):
    """(episode, chosen candidate id, inputs digest) for each decision since."""
    out = []
    for episode, body in db.execute(
            "SELECT episode_id, body_json FROM events WHERE kind='decision' AND at > ?"
            " ORDER BY at", (since,)):
        b = json.loads(body)
        out.append((episode, b.get("chose"), b.get("inputs")))
    return out


def in_view_of(chosen_id, here):
    """The recorded choice's depth-view verdict, by the new rule."""
    if not chosen_id or not chosen_id.startswith("improve_geometry:") or "@" not in chosen_id:
        return None
    target, at = chosen_id.split(":", 1)[1].split("@")
    if at == "nowhere":
        return None
    vx, vy = (float(v) for v in at.split(","))
    thing = next((e for e in here.body.get("entities") or [] if e.get("id") == target), None)
    placement = (thing or {}).get("placement") or {}
    if "x_m" not in placement:
        return None
    got = goals._from_viewpoint(placement, thing, vx, vy)
    return got["in_depth_view"]


def main():
    a = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    a.add_argument("--episodes", type=Path, required=True)
    a.add_argument("--since", required=True)
    a.add_argument("--output", type=Path, required=True)
    args = a.parse_args()
    assert not args.output.exists(), "choose a new output file"
    since = time.mktime(time.strptime(args.since, "%Y-%m-%d %H:%M"))
    tmp = tempfile.mkdtemp(prefix="redecide-")
    shutil.copy(args.episodes, Path(tmp) / "episodes.db")
    store = EpisodeStore(tmp)
    db = sqlite3.connect((Path(tmp) / "episodes.db").as_uri() + "?mode=ro", uri=True)
    rows, tally = [], Counter()
    for episode, chose, inputs in recorded_choices(db, since):
        here = situation_mod.restore(store, inputs) if inputs else None
        if here is None:
            tally["situation not restorable"] += 1
            continue
        was_geometry = bool(chose and chose.startswith("improve_geometry:"))
        got = scoring.consider(here, scoring.DEFAULT, authority=False)
        # Without authority nothing is chosen, but the preferred goal is named.
        now = (got["preferred"]["candidate"]["id"] if got.get("preferred") else None)
        geometry = [one for one in got["considered"]
                    if one["candidate"]["type"] == "improve_geometry"]
        refused_view = sum(1 for one in geometry if any(
            v["veto"] == "outside the depth camera's view" for v in one["vetoes"]))
        new_view = None
        if now and now.startswith("improve_geometry:"):
            pick = got["preferred"]["candidate"]["constraints"]
            new_view = pick.get("in_depth_view")
        row = {"episode": episode, "was": chose, "was_in_view": in_view_of(chose, here),
               "now": now, "now_in_view": new_view, "geometry_candidates": len(geometry),
               "refused_for_view": refused_view}
        rows.append(row)
        if was_geometry:
            tally["geometry decisions"] += 1
            tally["...whose thing was in view: %s" % row["was_in_view"]] += 1
            tally["...made again: %s" % ("geometry" if now and now.startswith("improve_geometry") else (now or "nothing").split(":")[0])] += 1
            if new_view is not None:
                tally["...made again, thing in view: %s" % new_view] += 1
    store.close()
    args.output.write_text(json.dumps({"tally": dict(tally), "rows": rows}, indent=1) + "\n")
    for key, value in sorted(tally.items()):
        print("%4d  %s" % (value, key))


if __name__ == "__main__":
    main()
