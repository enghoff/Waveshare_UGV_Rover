"""Check replay conservation and produce compact, reproducible measured results."""
import argparse
import hashlib
import json
from pathlib import Path

from audit import Evidence, RECORDING


def compact(score):
    return {k:v for k,v in score.items() if k != "per_object"}


def combined(scores):
    keys = ("same_together", "same_apart", "different_together", "different_apart",
            "main_looks", "main_waiting", "odd_kept_in_original_home", "odd_count",
            "objects_scored", "objects_split")
    total = {k:sum(s[k] for s in scores) for k in keys}
    tp,fp,fn = (total[k] for k in ("same_together", "different_together", "same_apart"))
    total.update(precision=tp/(tp+fp),recall=tp/(tp+fn),f1=2*tp/(2*tp+fp+fn))
    total["macro_object_pair_recall"] = sum(s["macro_object_pair_recall"]*s["objects_scored"]
                                              for s in scores)/total["objects_scored"]
    return total


def summarize(directory, output):
    load = lambda name: json.loads((directory/(name+".json")).read_text())
    baseline = load("baseline-0")
    original = Evidence()
    reviewed = Evidence(excluded=[61656])
    person = Evidence(excluded=[61656], same_person=True)
    saved = json.loads((RECORDING/"owners_none_0.json").read_text())
    assert baseline["owners_final"] == saved, "baseline does not reproduce the previous replay"
    result = {"source_revision":"30c3bce5c50f73ffbc3ec9271f7c9a14044dc919",
              "database_sha256":baseline["database_sha256"], "labels_sha256":baseline["labels_sha256"],
              "baseline_exact_memberships_reproduced":len(saved),
              "audit":original.audit(), "reviewed_audit":reviewed.audit(),
              "appearance":load("model-validation"),
              "reviewed_appearance":load("reviewed-model-validation"),
              "original_label_replays":{}, "reviewed_label_replays":{}, "traces":{}}
    result["person_policy_replays"] = {}
    result["person_policy_audit"] = person.audit()
    result["person_policy_appearance"] = load("person-model-validation")
    for prefix,evidence,section in (("",original,"original_label_replays"),
                                    ("reviewed-",reviewed,"reviewed_label_replays"),
                                    ("person-",person,"person_policy_replays")):
        groups = [load(prefix+f"groups-{fold}") for fold in (0,1)]
        assert all(g["owners_before_end"] == saved for g in groups), "grouping changed resolver outcome"
        result[section]["baseline"] = compact(evidence.score(saved))
        result[section]["groups_after_final_look"] = combined([g["final"] for g in groups])
        checkpoints = []
        for a,b in zip(groups[0]["checkpoints"],groups[1]["checkpoints"]):
            assert a["frame"]==b["frame"]
            scores = {k:combined([a[k],b[k]]) for k in ("resolver","reader_groups","fresh_groups")}
            checkpoints.append({"frame":a["frame"],**scores,
                                "added_wrong_pairs":scores["reader_groups"]["different_together"]
                                                      -scores["resolver"]["different_together"]})
        result[section]["checkpoints"] = checkpoints
        result[section]["reader_groups_pass_all_checkpoints"] = all(
            s["reader_groups"]["same_together"]>=s["resolver"]["same_together"]
            and s["added_wrong_pairs"]<=0 for s in checkpoints)
    # Resolve the apparent transient failures down to labelled records, rather
    # than attributing 44 pair errors to 44 independent object mistakes.
    extra = []
    old_groups = [load(f"reviewed-groups-{fold}") for fold in (0,1)]
    for i,j,same,fold,t,u in reviewed.pairs:
        if same:
            continue
        snap = next(s for s in old_groups[fold]["checkpoints"] if s["frame"]==413)
        base,reader = snap["resolver_owners"],snap["reader_owners"]
        i,j = str(i),str(j)
        if i not in base or j not in base:
            continue
        if reader[i] is not None and reader[i]==reader[j] and (base[i] is None or base[i]!=base[j]):
            extra.append({"observations":[int(i),int(j)],"records":[t,u]})
    result["transient_apparent_errors"] = extra
    assert len(extra)==44 and all(set(p["records"])=={"object:332","object:385"} for p in extra)
    result["original_label_replays"]["live_every_5min"] = combined([
        load(f"live-{fold}")["final"] for fold in (0,1)])
    result["reviewed_label_replays"]["live_fold_1_only"] = compact(load("reviewed-live-1")["final"])
    base_trace = load("reviewed-groups-0")["trace"]
    live_trace = load("reviewed-live-1")["trace"]
    for oid,entity in ((62091,"object:6"),(62893,"object:9")):
        a = next(r for r in base_trace if r["observation"]==oid and
                 (r["decision"] or {}).get("entity")==entity)
        b = next(r for r in live_trace if r["observation"]==oid and r["frame"]==a["frame"])
        result["traces"][str(oid)] = {
            "baseline":a,"live":b,
            "baseline_candidate":next(c for c in a["candidates"] if c["entity"]==entity),
            "live_candidate":next((c for c in b["candidates"] if c["entity"]==entity),None)}
    result["raw_output_sha256"] = {p.name:hashlib.sha256(p.read_bytes()).hexdigest()
                                   for p in directory.glob("*.json") if p.resolve()!=output.resolve()}
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(result,indent=2)+"\n")
    for name in ("original_label_replays","reviewed_label_replays","person_policy_replays"):
        print(name)
        for method in ("baseline","groups_after_final_look","live_every_5min","live_fold_1_only"):
            if method in result[name]:print(method,result[name][method])
        for s in result[name]["checkpoints"]:
            print("checkpoint",s["frame"],"same",s["reader_groups"]["same_together"],
                  "wrong",s["reader_groups"]["different_together"],"extra wrong",s["added_wrong_pairs"])
    for oid,case in result["traces"].items():
        print("trace",oid,"frame",case['live']['frame'])
        for mode in ("baseline_candidate","live_candidate"):
            print(mode,{k:v for k,v in (case[mode] or {}).items() if k!='placement'})


if __name__ == "__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("--directory",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    summarize(args.directory,args.output)
