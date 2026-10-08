"""Which region of an aimed look is the thing it was aimed at? An offline rule.

The autonomy executive drives to a viewpoint facing a thing and takes a look, and
the look's regions are then filed by the resolver like any other -- which, when
the thing has duplicates or sits beyond the lidar's reach, leaves the region of
the thing waiting or files it under another record
([2026-10-06](../../docs/progress/2026-10-06-targeted-look-provenance.md),
[2026-10-07](../../docs/progress/2026-10-07-identity-trial-scored.md)). The rule
tested here uses what the resolver does not know: which thing the look was for.

    RULE (fixed 2026-10-08, before any look was scored):
    a region is eligible for the target when it has a bearing, points at the
    target's placement as it stood when the goal was chosen within the resolver's
    own allowance (`resolve._allowance_used`, with the map's reach left out), agrees
    on height (`locate.stands_as_high`) and, when ranged, on range
    (`locate.stands_at_range`), and looks like the target (`DIFFERENT_THING`)
    wherever appearance can be asked. The eligible region using the least of its
    allowance is chosen. Where another eligible region uses within 0.25 of the
    same allowance, appearance must put the chosen one 0.05 ahead, or nothing is
    chosen.

    python experiments/entity_association/aimed_attachment.py
        --directory captures/entity-target-audit-20261006 --output <new-file>

Reads the frozen audit (`audit.json`, from `audit_targeted_looks.py`) and the
world store copied with it. Appearance is asked against the target's exemplars as
the copied store holds them, which is after the look; a target no longer in the
store is judged on geometry alone. This says which region the rule would choose
and what the resolver did with it. Whether the chosen region shows the target is
a question for the photographs.
"""
from __future__ import annotations

import argparse
import json
import sqlite3
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from world_state import locate, resolve  # noqa: E402
from world_state.store import _readable  # noqa: E402

CLOSE_ALLOWANCE = 0.25
APPEARANCE_LEAD = 0.05

#: RULE "window" (fixed 2026-10-08 after rule "allowance" found the nearest region
#: of 69 unmatched looks a median 6.2 degrees off, before any choice was judged):
#: an aimed look is pointed with the same heading it is measured with, so the
#: target can sit anywhere within the heading's error of where it is predicted.
#: Within `WINDOW_DEG` of the predicted bearing, with height and range agreeing,
#: the region that looks most like the target is chosen if it reaches
#: `RECOGNISED` and leads every other region in the window by `APPEARANCE_LEAD`.
WINDOW_DEG = 15.0


def choose_window(placement, regions, exemplars, appearance_between):
    """Rule "window" above. Returns (chosen region id or None, reason, candidates)."""
    import math
    inside = []
    for region in regions:
        ray = resolve.ray_of(region, None)
        if ray is None:
            continue
        to = math.degrees(math.atan2(placement["y_m"] - ray["y_m"],
                                     placement["x_m"] - ray["x_m"]))
        miss = abs((ray["bearing_deg"] - to + 180.0) % 360.0 - 180.0)
        if miss > WINDOW_DEG:
            continue
        if not locate.stands_as_high(placement, ray):
            continue
        if not locate.stands_at_range(placement, ray):
            continue
        seen = (appearance_between(exemplars, [region.get("dino_blob") or b""])
                if exemplars and region.get("dino_blob") else None)
        inside.append((miss, seen, region["id"]))
    if not inside:
        return None, "no region within the window", []
    scored = sorted((one for one in inside if one[1] is not None), key=lambda one: -one[1])
    if not scored:
        return None, "appearance cannot be asked", inside
    best = scored[0]
    if best[1] < resolve.RECOGNISED:
        return None, "nothing in the window looks like it", inside
    if len(scored) > 1 and best[1] - scored[1][1] < APPEARANCE_LEAD:
        return None, "two regions in the window look alike", inside
    return best[2], "chosen", inside


def choose(placement, regions, exemplars, appearance_between):
    """The rule above. Returns (chosen region id or None, reason, candidates)."""
    eligible = []
    for region in regions:
        ray = resolve.ray_of(region, None)
        if ray is None:
            continue
        used = resolve._allowance_used(placement, ray)
        if used is None:
            continue
        if not locate.stands_as_high(placement, ray):
            continue
        if not locate.stands_at_range(placement, ray):
            continue
        seen = (appearance_between(exemplars, [region.get("dino_blob") or b""])
                if exemplars and region.get("dino_blob") else None)
        if seen is not None and seen < resolve.DIFFERENT_THING:
            continue
        eligible.append((used, seen, region["id"]))
    if not eligible:
        return None, "no region points at it", []
    eligible.sort(key=lambda one: one[0])
    best = eligible[0]
    close = [one for one in eligible[1:] if one[0] - best[0] <= CLOSE_ALLOWANCE]
    if close:
        rivals = [one for one in [best, *close] if one[1] is not None]
        if len(rivals) != 1 + len(close):
            return None, "two regions point at it and appearance cannot be asked", eligible
        rivals.sort(key=lambda one: -one[1])
        if rivals[0][1] - rivals[1][1] < APPEARANCE_LEAD:
            return None, "two regions point at it and look alike", eligible
        best = rivals[0]
    return best[2], "chosen", eligible


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--directory", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--rule", choices=("allowance", "window"), default="allowance")
    a = p.parse_args()
    rule = choose if a.rule == "allowance" else choose_window
    assert not a.output.exists(), "choose a new output file"
    from world_state.appearance import between
    audit = json.loads((a.directory / "audit.json").read_text())
    world = sqlite3.connect((a.directory / "world.db").resolve().as_uri() + "?mode=ro",
                            uri=True)
    world.row_factory = sqlite3.Row

    def exemplars_of(entity_id, width):
        """As `WorldStore.exemplars` reads them: float32 vectors end to end."""
        row = world.execute("SELECT exemplars FROM entities WHERE id = ?",
                            (entity_id,)).fetchone()
        blob = None if row is None else row["exemplars"]
        if not blob or width <= 0:
            return []
        return [blob[i:i + width] for i in range(0, len(blob) - width + 1, width)]

    out, tally = [], Counter()
    for row in audit["rows"]:
        if not row.get("observations"):
            continue
        before = row.get("target_before") or {}
        placement = before.get("placement")
        ids = [o["id"] for o in row["observations"]]
        regions = [_readable(dict(r), vectors=True) for r in world.execute(
            "SELECT * FROM observations WHERE id IN (%s)" % ",".join("?" * len(ids)), ids)]
        with_bearing = [r for r in regions if r.get("bearing_deg") is not None
                        and r.get("pose")]
        if placement is None:
            verdict, chosen, why, cands = "no target placement", None, "", []
        elif not with_bearing:
            verdict, chosen, why, cands = "no region has a bearing", None, "", []
        else:
            width = max((len(r.get("dino_blob") or b"") for r in with_bearing), default=0)
            chosen, why, cands = rule(placement, with_bearing,
                                        exemplars_of(row["target"], width), between)
            verdict = why
        owner = next((o["entity_id"] for o in row["observations"] if o["id"] == chosen), None)
        resolver_did = (None if chosen is None else
                        "filed under the target" if owner == row["target"] else
                        "left waiting" if owner is None else "filed under another record")
        tally[verdict] += 1
        if resolver_did:
            tally["chosen, and the resolver " + resolver_did] += 1
        out.append({"episode": row["episode"], "target": row["target"],
                    "frame_id": row["frame_id"], "chosen": chosen, "verdict": verdict,
                    "resolver_did": resolver_did, "owner_in_copy": owner,
                    "target_in_copy": row["target_exists_in_snapshot"],
                    "candidates": [[round(u, 3), None if s is None else round(s, 3), i]
                                   for u, s, i in cands]})
    report = {"rule": {"name": a.rule, "close_allowance": CLOSE_ALLOWANCE,
                       "appearance_lead": APPEARANCE_LEAD, "window_deg": WINDOW_DEG},
              "looks": len(out), "tally": dict(tally), "independent_acceptance": False,
              "limits": ["Whether a chosen region shows the target is not judged here.",
                         "Appearance uses the copied store's exemplars, written after the look.",
                         "Targets no longer in the copied store are judged on geometry alone."],
              "looks_detail": out}
    a.output.write_text(json.dumps(report, indent=1) + "\n")
    for key, value in sorted(tally.items(), key=lambda kv: -kv[1]):
        print("%4d  %s" % (value, key))


if __name__ == "__main__":
    main()
