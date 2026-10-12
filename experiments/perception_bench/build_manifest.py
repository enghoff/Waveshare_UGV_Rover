"""Assemble evidence references and scoped truth, without contacting the rover.

Inputs are already-copied world metadata and a read-only remote inventory. The
manifest is an evidence catalogue, not a claim to have rerun current perception.
"""
from __future__ import annotations

import argparse
import ast
from collections import Counter, defaultdict
from datetime import datetime, timedelta, timezone
import gzip
import hashlib
import json
from pathlib import Path
import sqlite3

from PIL import Image

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
IST = timezone(timedelta(hours=5, minutes=30))


def read(path):
    return json.loads(path.read_text(encoding="utf-8"))


def sha(path):
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def table(db, name):
    cols = [r[1] for r in db.execute(f"pragma table_info({name})") if r[2] != "BLOB"]
    return [dict(r) for r in db.execute(f"select {','.join(cols)} from {name}")]


def clock(day, hour):
    return datetime.fromisoformat(day + "T" + hour).replace(tzinfo=IST).timestamp()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--metadata", type=Path, required=True)
    p.add_argument("--inventory", type=Path, required=True)
    args = p.parse_args()
    world = json.load(gzip.open(args.metadata, "rt", encoding="utf-8"))
    inventory = read(args.inventory)
    snapshots = {"current_20261012": world}
    snapshot_sources = {"current_20261012": {"metadata_source": str(args.metadata),
        "metadata_sha256": sha(args.metadata), "backup_on_rover": "/tmp/codex-m5-audit-world.db",
        "local_backup": "tmp/codex-m5-audit/world.db", "taken_at_ist": "2026-10-12T11:44:00+05:30"}}
    for key, path in [("tape_20261003", "captures/2026-10-03-targets/world-2026-10-03-acceptance.db"),
                      ("merge_20261003", "captures/2026-10-03-merge-review/world_now.db")]:
        db = sqlite3.connect((ROOT / path).as_uri() + "?mode=ro", uri=True)
        db.row_factory = sqlite3.Row
        snapshots[key] = {name: table(db, name) for name in ["meta", "entities", "observations", "frames"]}
        db.close()
        snapshot_sources[key] = {"database": path, "sha256": sha(ROOT / path)}
    frame_index = {}
    observation_index = {}
    entity_index = {}
    image_checks = {}
    for key, data in snapshots.items():
        source = snapshot_sources[key]
        source["meta"] = {r["key"]: r["value"] for r in data["meta"]}
        source["counts"] = {k: len(data[k]) for k in ["frames", "entities", "observations"]}
        entity_index[key] = {e["id"]: e for e in data["entities"]}
        observation_index[key] = {o["id"]: o for o in data["observations"]}
        for f in data["frames"]:
            fid = f["id"]
            candidates = [ROOT / "tmp/codex-m5-audit/frames" / (fid + ".jpg"),
                          ROOT / "captures/2026-10-03-targets/frames-2026-10-03-acceptance" / (fid + ".jpg")]
            local = [str(x.relative_to(ROOT)).replace("\\", "/") for x in candidates if x.exists()]
            local_depth = [x.with_suffix(".depth.gz") for x in candidates if x.with_suffix(".depth.gz").exists()]
            for path in local:
                if path not in image_checks:
                    try:
                        with Image.open(ROOT / path) as im:
                            size = list(im.size)
                            im.verify()
                        image_checks[path] = {"decodable": True, "raster_size": size}
                    except Exception as error:
                        image_checks[path] = {"decodable": False, "error": type(error).__name__}
            current = next((r for r in world["frames"] if r["id"] == fid), None)
            frame_index[key + "/" + fid] = {"frame_id": fid, "snapshot": key,
                "recorded_path_on_rover": f["path"], "local_jpeg_paths": local,
                "local_jpeg_checks": {path: image_checks[path] for path in local},
                "stored_raster_width": f.get("width"), "stored_raster_height": f.get("height"),
                "stored_jpeg_bytes": f.get("bytes"), "sensor_native_resolution_retained": "not established",
                "jpeg_exists_in_current_world_on_rover": current["jpeg_exists"] if current else None,
                "depth_exists_in_current_world_on_rover": current["depth_exists"] if current else None,
                "local_depth_paths": [str(x.relative_to(ROOT)).replace("\\", "/") for x in local_depth],
                "recorded_depth_path_on_rover": f["path"].replace(".jpg", ".depth.gz")}

    def evidence(key, record_ids=None, observation_ids=None, start=None, end=None):
        rows = list(observation_index[key].values())
        if observation_ids is not None:
            missing = set(observation_ids) - observation_index[key].keys()
            if missing:
                raise ValueError(f"{key}: missing ground-truth observations {missing}")
            rows = [observation_index[key][oid] for oid in observation_ids]
        elif record_ids is not None:
            rows = [o for o in rows if o["entity_id"] in record_ids]
        if start is not None:
            rows = [o for o in rows if start <= o["observed_at"] <= end]
        fids = sorted({o["frame_id"] for o in rows if o["frame_id"]})
        return {"snapshot": key, "observation_ids": [o["id"] for o in rows],
                "frame_refs": [key + "/" + fid for fid in fids],
                "regions": len(rows), "pictures": len(fids),
                "recorded_poses": sum(bool(o["observer_pose_json"]) for o in rows),
                "stored_ranges": sum(o.get("range_m") is not None for o in rows),
                "raw_depth_pictures_on_rover": sum(frame_index[key + "/" + fid]["depth_exists_in_current_world_on_rover"] is True for fid in fids),
                "raw_depth_pictures_local": sum(bool(frame_index[key + "/" + fid]["local_depth_paths"]) for fid in fids),
                "jpeg_pictures_local": sum(bool(frame_index[key + "/" + fid]["local_jpeg_paths"]) for fid in fids),
                "poses_and_boxes": "SQLite observations.observer_pose_json and bbox_json in the named snapshot",
                "vectors_and_masks": "SQLite observation BLOB columns; metadata export omits their bytes; use the full database"}

    def bags(names):
        return [inventory["bags"].get(n, {"path": "/home/jetson/ugv/ros_nav/recordings/bags/" + n,
                                        "exists": False}) for n in names]

    items = []
    source_paths = set()

    def add(kind, name, truth, sources, ev, **extra):
        source_paths.update(sources)
        items.append({"id": kind + "/" + name, "kind": kind, "ground_truth": truth,
                      "source_files": sources, "evidence": ev, **extra})

    label_source = "experiments/m5_changes/labels-2026-10-10.json"
    labels = read(ROOT / label_source)
    for eid, (category, note) in ((k, v) for k, v in labels.items() if not k.startswith("_")):
        add("record_review", eid, {"category": category, "description": note,
            "authority": "analyst crop-strip review on 2026-10-10; a few strays allowed; not owner-confirmed"},
            [label_source], evidence("current_20261012", [eid]))
        if category == "OBJ":
            phrase = note.replace(" (duplicate)", "").replace(" (one stray edge)", "")
            add("find_by_words", "label-" + eid, {"query": phrase, "present": True,
                "known_matching_record_at_label_time": eid, "exhaustive_correct_answer_set": False,
                "authority": "same analyst label; subsequent memberships may include other objects"},
                [label_source], evidence("current_20261012", [eid]),
                scoring_status="positive anchor, not an exhaustive top-1 accuracy label")

    source = "world_state/bench_search.py"
    sets = None
    for node in ast.parse((ROOT / source).read_text(encoding="utf-8")).body:
        if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "SETS" for t in node.targets):
            sets = ast.literal_eval(node.value)
    assert sets is not None
    for split, (real, nonsense) in sets.items():
        for present, phrases in [(True, real), (False, nonsense)]:
            for i, query in enumerate(phrases):
                add("find_by_words", f"{split}-{'real' if present else 'absent'}-{i:02}",
                    {"query": query, "present": present,
                     "correct_record_ids": None if present else [],
                     "authority": "frozen presence-only phrase set; real phrases based on 2026-09-08 labels"},
                    [source, "world_state/labels/m0-2026-09-08.json"],
                    {"snapshot": "current_20261012", "pictures": "all searchable assigned regions in full current database",
                     "text_vector_source": "must use the same model/backend as stored SigLIP vectors"},
                    split=split, scoring_status="presence/refusal only; positive exact identity needs annotation")

    truth_source = "experiments/perception_bench/ground_truth/m4_targets.json"
    score_source = "experiments/perception_bench/ground_truth/m4_recorded_scores.json"
    truth = read(ROOT / truth_source)
    scores = read(ROOT / score_source)["attempts"]
    for name, thing in truth["things"].items():
        add("find_by_words", "m4-" + name,
            {"query": thing["point"].split(",")[0], "present": True,
             "correct_record_ids_at_label_time": thing["records"],
             "known_map_point": {"x_m": thing["x_m"], "y_m": thing["y_m"]},
             "authority": "owner tape and frozen analyst record labels before M4 acceptance",
             "label_scope": "2026-10-09, map session 67; join through frozen observations after any rebuild"},
            [truth_source], evidence("current_20261012", thing["records"]),
            scoring_status="known target anchor; reject current records contaminated since the frozen label review")
        observed_scores = [{"episode": a["episode"], "target": a["target"],
                           "chosen": a["chosen"], "relook": a["relook"]} for a in scores if a["thing"] == name]
        add("where_things_are", "m4-" + name, thing,
            [truth_source, score_source, "captures/2026-10-09-m4-targets/TARGETS.txt"],
            evidence("current_20261012", thing["records"]),
            baseline={"kind": "historical recorded/replay M4 scores, not a fresh run of today's stack",
                      "attempts": observed_scores},
            target_reference_photo="captures/2026-10-09-m4-targets/photos/" + name + ".jpg",
            movement_caveat="N4 moved on later M5 days; restrict placement scoring to frozen M4 episodes" if name == "N4" else None,
            world_snapshots=[inventory["m4_snapshots"].get(a["episode"].replace("au/", "au_").replace("/episode:", "_episode_") + ".db") for a in observed_scores])
        if len(thing["records"]) > 1:
            add("one_object_several_records", "m4-" + name,
                {"subject": thing["point"], "records": thing["records"],
                 "scope": "frozen pre-acceptance record-label list; not proof of every member's purity"},
                [truth_source], evidence("current_20261012", thing["records"]))

    old_source = "experiments/perception_bench/ground_truth/old_tape_targets.json"
    for name, thing in read(ROOT / old_source)["things"].items():
        placements = []
        for eid in thing["records"]:
            ent = entity_index["tape_20261003"][eid]
            place = json.loads(ent["placement_json"] or "null")
            off = None if not place else ((place["x_m"] - thing["x_m"]) ** 2 + (place["y_m"] - thing["y_m"]) ** 2) ** .5
            placements.append({"record": eid, "placement": place, "point_error_m": off,
                               "note": "point error; cabinet centre differs from visible range surface"})
        add("where_things_are", "old-" + name, thing,
            [old_source, "captures/2026-10-03-targets/frame.py", "captures/2026-10-03-targets/score.py",
             "captures/2026-10-03-targets/residuals.json"],
            evidence("tape_20261003", thing["records"]),
            baseline={"kind": "placements preserved in the 2026-10-03 acceptance database", "placements": placements},
            scoring_status="development evidence; do not pool with held-out M4 acceptance")

    merge_source = "experiments/perception_bench/ground_truth/merge_look_labels.json"
    merge = read(ROOT / merge_source)
    for eid, g in merge["things"].items():
        add("per_look_identity", eid, g, [merge_source, "experiments/entity_association/label_review.json"],
            evidence("merge_20261003", observation_ids=g["main_looks"] + g["odd"] + g["unsure"]),
            excluded_observation_ids=[61656],
            label_caveat="19 uncertain looks were reviewed by owner; other labels analyst-only. Dining chairs are kind-level, not individual identity.")
    for i, ids in enumerate([["object:246", "object:351"], ["object:383", "object:400"], ["object:249", "object:340"]]):
        looks = [oid for eid in ids for oid in merge["things"][eid]["main_looks"] if oid != 61656]
        add("one_object_several_records", "merge-look-" + str(i),
            {"subject": merge["things"][ids[0]]["main"], "records": ids, "same_subject_observation_ids": looks,
             "completeness": "annotated subset; not all duplicates in the flat"},
            [merge_source, "experiments/entity_association/label_review.json"],
            evidence("merge_20261003", observation_ids=looks),
            scoring_status="disputed 61656 excluded; do not claim entire object:340 is pure")
    pair_source = "experiments/perception_bench/ground_truth/merge_pairs_20261004.json"
    for pair in read(ROOT / pair_source)["pairs"]:
        add("merge_pair_review", f"20261004-{pair['proposal']:02}", pair,
            [pair_source, "docs/progress/2026-10-04-merge-step-on-the-rover.md",
             "captures/2026-10-04-merge-review/merge-20261004-130218-1.jpg",
             "captures/2026-10-04-merge-review/merge-20261004-130218-2.jpg"],
            evidence("current_20261012", [pair["a"], pair["b"]]),
            scoring_status="sampled-crop analyst pair judgment; not automatic merge authority")

    staged_source = "experiments/m5_changes/staged/shakedown-1012.json"
    staged = read(ROOT / staged_source)
    changes = ([{"action": "move", **v} for v in staged["moves"]] +
               [{"action": "add", **v} for v in staged["added"]] +
               [{"action": "move_again", **v} for v in staged["also_moved"]])
    audit = read(HERE / "shakedown_audit.json")
    subject_keys = {"black armchair by the charger": "charger_armchair", "black armchair by the balcony": "balcony_armchair",
                    "red bean bag": "red_bean_bag", "black bean bag": "black_bean_bag"}
    for i, change in enumerate(changes):
        subject = audit["subjects"][subject_keys[change["what"]]]
        add("what_changed", "shakedown-" + str(i),
            {**change, "time_ist": "about 08:08" if change["action"] == "move_again" else "07:48-07:52",
             "position_authority": "owner reports moves; positions read from lidar, not tape",
             "visually_confirmed_observation_ids": [r["observation_id"] for r in subject["regions"]]},
            [staged_source, "docs/progress/2026-10-12-m5-shakedown.md",
             "experiments/perception_bench/shakedown_labels.json"],
            evidence("current_20261012", observation_ids=[r["observation_id"] for r in subject["regions"]]),
            recordings=bags(["session-shakedown-1012"]),
            pose_caveat="08:00:58 until refit about 08:11 is bad-pose evidence; do not fit static position across a move")

    scene_sources = ["docs/progress/2026-10-11-m5-scene-1.md", "docs/progress/2026-10-11-m5-scene-1-replayed.md"]
    scene = [
        {"what": "two armchairs, first move", "time_ist": "after 13:17, before 14:13; exact time unrecorded",
         "old_places_reported_gone": [[-16.97, -16.42], [-18.59, -16.61]],
         "new_places": [[-17.12, -15.99], None], "individual_chair_correspondence": "not established"},
        {"what": "two armchairs, second move", "time_ist": "14:18-14:24",
         "new_places": [[-17.84, -15.28], [-17.31, -14.96]], "individual_chair_correspondence": "not established"},
        {"what": "pink bucket added", "time_ist": "with first living-room staging", "place": None},
        {"what": "pillows added beside bed", "time_ist": "before bedroom visit at 14:18",
         "scan_returns": [[-8.12, -11.20], [-8.11, -11.67]]},
        {"what": "stool tipped beside wardrobe", "time_ist": "before bedroom visit at 14:18",
         "scan_returns": [[-10.00, -11.80], [-9.60, -12.02]]},
    ]
    for i, change in enumerate(scene):
        add("what_changed", "scene1-" + str(i), {**change,
            "authority": "owner staging description and analyst scan/photo positions; positions await owner confirmation",
            "correction": "13:10 staging time in the original entry is superseded by the replay entry"},
            scene_sources, evidence("current_20261012", start=clock("2026-10-11", "13:12:00"), end=clock("2026-10-11", "14:33:00")),
            recordings=bags(["m5-scene1-141241", "m5-scene1-bedroom-141818", "m5-scene1-living-142211", "m5-scene1-living-142501"]),
            scoring_status="qualitative/interval truth; exact object-level move scoring incomplete; parked gap has no recording")
    controls = [(1, "08:04:21", "08:19:21", "m5-control1-080421"), (2, "08:49:46", "08:59:46", "m5-control2-084946"),
                (3, "09:14:31", "09:24:31", "m5-control3-091431"), (4, "09:33:12", "09:48:12", "m5-control4-093312"),
                (5, "12:46:48", "12:54:48", "m5-control5-124648"), (6, "12:55:42", "12:57:42", "m5-control6-125542"),
                (7, "13:01:00", "13:07:00", "m5-control7-130100")]
    control_sources = ["docs/progress/2026-10-11-m5-control-1.md", "docs/progress/2026-10-11-m5-controls-2-to-4.md",
                       "docs/progress/2026-10-11-m5-controls-5-to-7.md", scene_sources[0]]
    for n, start, end, bag in controls:
        add("what_changed_control", str(n), {"furniture_moved": False, "scope": "owner unchanged-room statement; four doors had been closed before control 1",
            "run": f"run/710476ae/{n}", "approximate_window_ist": [start, end],
            "minimum_furniture_span_m": .6, "too_short_to_count_for_acceptance": n == 6,
            "live_false_alarm": {"kind": "gone", "span_m": .7, "at": [-10.04, -10.83]} if n == 3 else None},
            control_sources, evidence("current_20261012", start=clock("2026-10-11", start), end=clock("2026-10-11", end)),
            recordings=bags([bag]), scoring_status="recorded window includes margin; trim using run/bag timestamps; keep deployed-version differences")

    used_frames = {ref for item in items for ref in item["evidence"].get("frame_refs", [])}
    frame_index = {k: v for k, v in frame_index.items() if k in used_frames}
    sources = {path: {"exists_local": (ROOT / path).exists(), "sha256": sha(ROOT / path) if (ROOT / path).is_file() else None}
               for path in sorted(source_paths)}
    result = {"version": 1, "assembled_at_ist": datetime.now(IST).isoformat(),
        "requirements": ["R-WS-10", "R-WS-13", "R-WS-17", "R-WS-18"],
        "purpose": "Frozen evidence catalogue for evaluating candidate perception against the same observations",
        "identity_scope": "Every record and observation ID is scoped to its source snapshot. Even generation equality does not prove IDs survived a rebuild.",
        "current_deployed_components": inventory["deploy_state"]["components"],
        "baseline_rerun_performed": False, "snapshots": snapshot_sources, "sources": sources,
        "counts": dict(Counter(i["kind"] for i in items)), "items": items,
        "frame_index_file": "frame_index.json", "raw_evidence_is_committed": False,
        "gaps": ["Most raw pictures remain on rover or in ignored captures; the catalogue does not ship them.",
                 "Sensor-native camera resolution retention is not established; stored frame metadata usually says 640x480.",
                 "Most frozen real search phrases have presence truth, not an exhaustive correct-record answer set.",
                 "Record-level descriptions permit stray looks; they are not per-pixel or per-look pure labels.",
                 "Identical dining chairs are labelled by kind; individual physical identity is unresolved.",
                 "61656 is disputed and excluded; remaining analyst labels lack full owner confirmation.",
                 "Scene 1 has an unrecorded staging gap, approximate positions and unknown individual-chair correspondence.",
                 "Historical scores are not a reproduction of the current deployed baseline.",
                 "The full episode-db transfer failed; complete consistent backup remains on rover and the audit's selected events were exported successfully."]}
    corrupt = [path for path, check in image_checks.items() if not check["decodable"]]
    result["local_corrupt_jpegs"] = corrupt
    if corrupt:
        result["gaps"].append("At least one retained picture is an empty JPEG; file existence does not prove decodability. See local_corrupt_jpegs.")
    (HERE / "manifest.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    (HERE / "frame_index.json").write_text(json.dumps(frame_index, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result["counts"], indent=2))
    print("items", len(items), "scoped frames", len(frame_index), "source files", len(sources))


if __name__ == "__main__":
    main()
