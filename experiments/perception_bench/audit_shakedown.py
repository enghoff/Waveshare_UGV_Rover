"""Summarize reviewed regions from a consistent, read-only world-store export.

No rover writes, database edits, or resolver changes. Region labels are visual
judgments, not labels inferred from the resolver's own entity assignments.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from datetime import datetime, timedelta, timezone
import gzip
import hashlib
import json
from pathlib import Path

IST = timezone(timedelta(hours=5, minutes=30))


def epoch(clock):
    return datetime.fromisoformat("2026-10-12T" + clock).replace(tzinfo=IST).timestamp()


def phase(at):
    if at < epoch("07:48:05"):
        return "before_staging"
    if at < epoch("08:00:58"):
        return "after_staging_before_pose_fault"
    if at < epoch("08:11:00"):
        return "pose_fault_window"
    return "after_refit"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("metadata", type=Path)
    parser.add_argument("--labels", type=Path, default=Path(__file__).with_name("shakedown_labels.json"))
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    data = json.load(gzip.open(args.metadata, "rt", encoding="utf-8"))
    labels = json.loads(args.labels.read_text(encoding="utf-8"))
    observations = {o["id"]: o for o in data["observations"]}
    entities = {e["id"]: e for e in data["entities"]}
    frames = {f["id"]: f for f in data["frames"]}
    result = {"requirements": ["R-WS-17", "R-WS-18"],
              "metadata_sha256": hashlib.sha256(args.metadata.read_bytes()).hexdigest(),
              "labels_sha256": hashlib.sha256(args.labels.read_bytes()).hexdigest(),
              "meta": {r["key"]: r["value"] for r in data["meta"]}, "subjects": {}}
    seen = set()
    for name, ids in labels["subjects"].items():
        rows = []
        for oid in ids:
            if oid in seen:
                raise ValueError(f"region {oid} has two subject labels")
            seen.add(oid)
            o = observations[oid]
            rows.append({"observation_id": oid, "entity_id": o["entity_id"],
                         "frame_id": o["frame_id"], "bbox": json.loads(o["bbox_json"]),
                         "at_ist": datetime.fromtimestamp(o["observed_at"], IST).isoformat(),
                         "phase": phase(o["observed_at"]), "pose_recorded": bool(o["observer_pose_json"]),
                         "range_m": o["range_m"], "note": o["note"],
                         "jpeg_exists_on_rover": frames[o["frame_id"]]["jpeg_exists"],
                         "depth_exists_on_rover": frames[o["frame_id"]]["depth_exists"]})
        record_ids = sorted({r["entity_id"] for r in rows if r["entity_id"]})
        records = {}
        for eid in record_ids:
            e = entities[eid]
            records[eid] = {"total_memberships_now": e["observation_count"],
                            "created_at_ist": datetime.fromtimestamp(e["created_at"], IST).isoformat(),
                            "placement_now": json.loads(e["placement_json"] or "null"),
                            "placement_updated_at_ist": datetime.fromtimestamp(e["placement_updated_at"], IST).isoformat(),
                            "reviewed_subject_memberships": sum(r["entity_id"] == eid for r in rows)}
        counts = {}
        for state in sorted({r["phase"] for r in rows}):
            part = [r for r in rows if r["phase"] == state]
            counts[state] = {"reviewed_regions": len(part), "unassigned": sum(r["entity_id"] is None for r in part),
                             "no_recorded_pose": sum(not r["pose_recorded"] for r in part),
                             "memberships": dict(Counter(r["entity_id"] or "unassigned" for r in part))}
        result["subjects"][name] = {"counts": counts, "records": records, "regions": rows}
    run = [o for o in observations.values() if epoch("07:52:15") <= o["observed_at"] <= epoch("08:01:32")]
    result["run"] = {"regions": len(run), "frames": len({o["frame_id"] for o in run}),
                     "without_pose": sum(not o["observer_pose_json"] for o in run),
                     "unassigned": sum(o["entity_id"] is None for o in run)}
    result["review_limits"] = labels["review_limits"]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({k: v["counts"] for k, v in result["subjects"].items()}, indent=2))
    print("run", result["run"])


if __name__ == "__main__":
    main()
